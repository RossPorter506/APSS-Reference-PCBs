// An example board support package for a stack using the MCU and Beacon boards.

#![allow(dead_code)]
use msp430fr2x5x_hal::{ 
    clock::{Clock, ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv}, 
    delay::SysDelay, 
    ehal::digital::OutputPin, 
    fram::Fram, 
    gpio::{Batch, P1, P2, P3, P4, P5}, 
    pac::PMM, 
    pmm::Pmm, 
    spi::SpiConfig, 
    watchdog::Wdt,
};
use msp430fr2355::P6;
use crate::{gps::Gps, lora::Radio, pin_mappings::*, println};

/// Top-level object representing the board.
/// 
/// Not all peripherals are configured. If you need more, add their configuration code to ::configure().
pub struct Board {
    pub delay: SysDelay,
    pub gps: Gps,
    pub radio: Radio,
}

/// Call this function ONCE at the beginning of your program.
pub fn configure() -> Board {
    // Take hardware registers and disable watchdog
    let regs = msp430fr2355::Peripherals::take().unwrap();
    let _wdt = Wdt::constrain(regs.WDT_A);

    // Configure GPIO.
    let pins = Gpio::configure(regs.P1, regs.P2, regs.P3, regs.P4, regs.P5, regs.P6, regs.PMM);
    
    // Configure clocks to get accurate delay timing, and used by other peripherals
    let mut fram = Fram::new(regs.FRCTL);
    let (smclk, _aclk, delay) = ClockConfig::new(regs.CS)
        .mclk_dcoclk(DcoclkFreqSel::_24MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk() // 32768 Hz
        .freeze(&mut fram);

    // Spare UART, useful for debug printing to a computer
    crate::serial::configure_debug_serial(pins.debug_tx, &smclk, regs.E_USCI_A0);
    println!("Serial init"); // Like this!
    
    // SPI, used by the LoRa radio
    const SPI_FREQ_HZ: u32 = 8_000_000; // Max speed
    let clk_div = (smclk.freq() / SPI_FREQ_HZ) as u16;
    let spi_bus = SpiConfig::new(regs.E_USCI_B0, embedded_hal::spi::MODE_0, true)
        .to_master_using_smclk(&smclk, clk_div)
        .single_master_bus(pins.miso, pins.mosi, pins.sclk);
    
    // LoRa radio
    let radio = crate::lora::new(spi_bus, pins.lora_cs, pins.lora_reset, delay);

    // GPS
    let gps = crate::gps::Gps::new(regs.E_USCI_A1, &smclk, pins.gps_tx, pins.gps_rx, pins.gps_en);

    Board {delay, gps, radio}
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
    debug_tx:       DebugTxPin,
    debug_rx:       DebugRxPin,
}
impl Gpio {
    fn configure(p1: P1, p2: P2, p3: P3, p4 :P4, p5: P5, p6: P6, pmm: PMM) -> Self {
        // Configure GPIO
        let pmm = Pmm::new(pmm);
        let port1 = Batch::new(p1).split(&pmm);
        let _port2 = Batch::new(p2).split(&pmm);
        let _port3 = Batch::new(p3).split(&pmm);
        let port4 = Batch::new(p4).split(&pmm);
        let _port5 = Batch::new(p5).split(&pmm);
        let port6 = Batch::new(p6).split(&pmm);

        let gps_tx = port4.pin3.to_alternate1();
        let gps_rx = port4.pin2.to_alternate1();
        let debug_tx = port1.pin7.to_alternate1();
        let debug_rx = port1.pin6.to_alternate1();
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
        Gpio {mosi, miso, sclk, lora_cs, lora_reset, gps_rx, gps_tx, gps_reset, gps_en, audio_pwm, debug_rx, debug_tx}
    }
}
