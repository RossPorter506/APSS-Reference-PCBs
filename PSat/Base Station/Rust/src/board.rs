// An example board support package for a stack using the MCU and Beacon boards.

#![allow(dead_code)]
use embedded_hal::{digital::InputPin, pwm::SetDutyCycle};
use msp430fr2x5x_hal::{ 
    clock::{Clock, ClockConfig, DcoclkFreqSel, MclkDiv, REFOCLK, SmclkDiv}, 
    delay::SysDelay, 
    ehal::digital::OutputPin, 
    fram::Fram, 
    gpio::{Batch, P1, P2, P3, P4, P5}, 
    pac::{self, PMM, TB0, TB1, TB3}, 
    pmm::Pmm, 
    pwm::{CCR1, Pwm, PwmParts3, PwmParts7, TimerConfig, TimerDiv, TimerExDiv}, 
    spi::SpiConfig, 
    timer::{Timer, TimerParts3}, 
    watchdog::Wdt,
};
use msp430fr2355::P6;
use crate::{gps::Gps, lora::Radio, pin_mappings::*};

/// Top-level object representing the board.
/// 
/// Not all peripherals are configured. If you need more, add their configuration code to ::configure().
pub struct Board {
    pub delay: SysDelay,
    pub gps: Gps,
    pub radio: Radio,
    pub radio_delay_timer: Timer<TB1>,
    pub audio_pulse_timer: Timer<TB0>,
    pub audio_pwm:   Pwm<TB3, CCR1>,
}
impl Board {
    /// Make the buzzer buzz for 100ms every 3 seconds.
    pub fn manage_buzzer(&mut self) {
        if self.audio_pulse_timer.count() > AUDIO_TIMER_TOGGLE_POINT {
            self.audio_pwm.set_duty_cycle_percent(50); // Buzzer on
        } else {
            self.audio_pwm.set_duty_cycle_fully_off(); // Buzzer off
        }
    }
}

// Radio delay timer running off Aclk = Refoclk = 32768 Hz, so this gives a 1 sec timer.
pub const RADIO_DELAY_MAX: u16 = REFOCLK;

// With Aclk = Refoclk = 32768Hz, with a /3 divider on a timer counting to 32768 takes 3 sec
const AUDIO_TIMER_MAX: u16 = REFOCLK;
// When the timer is above this value we are between 2.9 and 3 sec. Used to turn on the buzzer for a short beep
const AUDIO_TIMER_TOGGLE_POINT: u16 = ((AUDIO_TIMER_MAX as u32 * 29) / 30) as u16;

/// Call this function ONCE at the beginning of your program.
pub fn configure() -> Board {
    // Take hardware registers and disable watchdog
    let regs = msp430fr2355::Peripherals::take().unwrap();
    let _wdt = Wdt::constrain(regs.WDT_A);

    // Configure GPIO.
    let pins = Gpio::configure(regs.P1, regs.P2, regs.P3, regs.P4, regs.P5, regs.P6, regs.PMM);
    
    // Configure clocks to get accurate delay timing, and used by other peripherals
    let mut fram = Fram::new(regs.FRCTL);
    let (smclk, aclk, delay) = ClockConfig::new(regs.CS)
        .mclk_dcoclk(DcoclkFreqSel::_16MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk() // 32768 Hz
        .freeze(&mut fram);

    // // Spare UART, useful for debug printing to a computer
    // crate::serial::configure_debug_serial(used.debug_tx_pin, &smclk, regs.E_USCI_A0);
    // println!("Serial init"); // Like this!
    
    // SPI, used by the LoRa radio
    const SPI_FREQ_HZ: u32 = 250_000; // 250kHz is arbitrary
    let clk_div = (smclk.freq() / SPI_FREQ_HZ) as u16;
    let spi_bus = SpiConfig::new(regs.E_USCI_B0, embedded_hal::spi::MODE_0, true)
        .to_master_using_smclk(&smclk, clk_div)
        .single_master_bus(pins.miso, pins.mosi, pins.sclk);
    
    // LoRa radio
    let radio = crate::lora::new(spi_bus, pins.lora_cs, pins.lora_reset, delay);

    // GPS
    let gps = crate::gps::Gps::new(regs.E_USCI_A1, &smclk, pins.gps_tx, pins.gps_rx, pins.gps_en);

    // Audio beep timer - used to generate ~100ms pulse every ~3sec
    let timer_parts = TimerParts3::new(regs.TB0, TimerConfig::aclk(&aclk).clk_div(TimerDiv::_1, TimerExDiv::_3));
    let mut audio_pulse_timer = timer_parts.timer;
    audio_pulse_timer.start(AUDIO_TIMER_MAX);

    // Audio PWM - generates a 4kHz 50% duty cycle
    const AUDIO_FREQUENCY_HZ: u32 = 4_000;
    let divider = (smclk.freq() / AUDIO_FREQUENCY_HZ) as u16;
    let timer_parts = PwmParts7::new(regs.TB3, TimerConfig::smclk(&smclk), divider);
    let audio_pwm = timer_parts.pwm1.init(pins.audio_pwm);

    // LoRa timer - enforces a minimum time between transmissions
    let timer_parts = TimerParts3::new(regs.TB1, TimerConfig::aclk(&aclk));
    let mut radio_delay_timer = timer_parts.timer;
    radio_delay_timer.start(RADIO_DELAY_MAX);

    Board {delay, gps, radio, audio_pulse_timer, audio_pwm, radio_delay_timer}
}

#[derive(Debug, Default, PartialEq, Eq, Copy, Clone)]
#[repr(u8)]
/// The mode that the beacon board is in. Driven by the bctrl0 and bctrl1 pins.
pub enum Mode {
    /// Do nothing
    AutoSleep = 0b00,
    /// Buzzer active
    AutoBeep = 0b01,
    /// Buzzer active and forwards GPS packets to radio
    #[default]
    AutoActive = 0b10,
    /// Exposes peripherals to the stack to be driven by something else.
    Manual = 0b11,
}
impl Mode {
    pub fn beeps(&self) -> bool {
        *self != Mode::AutoSleep
    }
    pub fn is_auto(&self) -> bool {
        *self != Mode::Manual 
    }
    pub fn transmits(&self) -> bool {
        *self == Mode::AutoActive
    }
    pub fn is_idle(&self) -> bool {
        matches!(self, Mode::AutoSleep | Mode::Manual)
    }
}

// Pins used by other peripherals.
struct Gpio {
    miso:           SpiMisoPin,
    mosi:           SpiMosiPin,
    sclk:           SpiSclkPin,
    lora_reset:     RadioResetPin,
    lora_cs:        RadioCsPin,
    gps_tx:         GpsTxPin,
    gps_rx:         GpsRxPin,
    gps_reset:      GpsResetPin,
    gps_en:         GpsEnPin,
    audio_pwm:      AudioPwmPin,
}
impl Gpio {
    fn configure(p1: P1, p2: P2, p3: P3, p4 :P4, p5: P5, p6: P6, pmm: PMM) -> Self {
        // Configure GPIO
        let pmm = Pmm::new(pmm);
        let port1 = Batch::new(p1).split(&pmm);
        let port2 = Batch::new(p2).split(&pmm);
        let port3 = Batch::new(p3).split(&pmm);
        let port4 = Batch::new(p4).split(&pmm);
        let port5 = Batch::new(p5).split(&pmm);
        let port6 = Batch::new(p6).split(&pmm);

        let gps_tx = port4.pin3.to_alternate1();
        let gps_rx = port4.pin2.to_alternate1();
        let mut gps_reset = port6.pin2.to_output();
        gps_reset.set_high();
        let mut gps_en = port4.pin0.to_output();
        gps_en.set_high();

        let sclk = port1.pin1.to_alternate1();
        let mosi = port1.pin2.to_alternate1();
        let miso = port1.pin3.to_alternate1();
        let mut lora_cs = port4.pin4.to_output();
        lora_cs.set_high();
        let mut lora_reset = port6.pin3.to_output();
        lora_reset.set_high();

        let audio_pwm = port6.pin0.to_output().to_alternate1();

        // Pins consumed by other perihperals
        Gpio {mosi, miso, sclk, lora_cs, lora_reset, gps_rx, gps_tx, gps_reset, gps_en, audio_pwm}
    }
}
