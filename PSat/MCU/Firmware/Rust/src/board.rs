// An example board support package for a stack using the MCU and Beacon boards.

#![allow(dead_code)]
use bmp390::sync::Bmp390;
use core::{cell::RefCell, convert::Infallible};
use msp430fr2355::{E_USCI_A1, E_USCI_B1, Peripherals};
use msp430fr2x5x_hal::{
    adc::{Adc, AdcConfig, ClockDivider, Predivider, Resolution, SampleTime, SamplingRate}, 
    clock::{Aclk, Clock, ClockConfig, DcoclkFreqSel, MclkDiv, Smclk, SmclkDiv}, 
    delay::SysDelay, 
    fram::Fram, 
    gpio::{Batch, Floating, Input, P1, P2, P3, P4, P5, P6, Pin, Pin0, Pin1, Pin3, Pin4, Pin5, Pin6, Pin7}, 
    i2c::{GlitchFilter, I2cConfig}, 
    pac::{E_USCI_B0, PMM, TB0}, 
    pmm::Pmm, 
    pwm::TimerConfig, 
    serial::{BitCount, BitOrder, Loopback, Parity, SerialConfig, StopBits}, 
    spi::{Spi, SpiConfig}, 
    timer::{Timer, TimerParts3}, 
    watchdog::Wdt
};
use embedded_hal::{delay::DelayNs, digital::{OutputPin, StatefulOutputPin}};
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
use static_cell::StaticCell;
use crate::{gps::Gps, icm42670::Imu, lora::Radio, pin_mappings::*, println};

type SpiMaster = RefCell<Spi<E_USCI_B1>>;
type SharedI2c = I2cRefCellDevice<'static, SensorI2c>;

/// Top-level object representing the MCU board.
/// 
/// Not all peripherals are configured. If you need more, add their configuration code to ::configure().
pub struct McuBoard {
    pub barometer: Bmp390<SharedI2c>,
    pub imu: Imu<SharedI2c>,
    pub delay: SysDelay,
    pub i2c: SharedI2c,
    pub spi: &'static SpiMaster,
    pub adc: Adc,
    pub gpio: Gpio,
    pub timer_b0: Timer<TB0>,
}
// This is where you should implement top-level functionality. 
impl McuBoard {
    pub fn battery_voltage_mv(&mut self) -> u16 {
        self.adc.read_voltage_mv(&mut self.gpio.half_vbat, 3300).unwrap() * 2
    }
}

/// Top-level object representing the a stack of PCBs (namely MCU + Beacon).
/// 
/// Not all peripherals are configured. If you need more, add their configuration code to ::configure().
pub struct Stack {
    pub barometer: Bmp390<SharedI2c>,
    pub imu: Imu<SharedI2c>,
    pub delay: SysDelay,
    pub gps: Gps,
    pub i2c: SharedI2c,
    pub spi: &'static SpiMaster,
    pub adc: Adc,
    pub radio: Radio,
    pub gpio: Gpio,
    pub timer_b0: Timer<TB0>,
}
// This is where you should implement top-level functionality. 
impl Stack {
    pub fn battery_voltage_mv(&mut self) -> u16 {
        self.adc.read_voltage_mv(&mut self.gpio.half_vbat, 3300).unwrap() * 2
    }
}

/// Configure just the MCU board. The other PCBs need not be attached.
pub fn standalone(regs: Peripherals) -> McuBoard {
    let (board, ..) = board_config(regs);
    board
}

/// Configure the entire stack of PCBs (MCU + Beacon). The Beacon must be attached for this to succeed.
pub fn in_stack(regs: Peripherals) -> Stack {
    let (board, smclk, _aclk, mut used, eusci_a1) = board_config(regs);
    let McuBoard {barometer, mut delay, i2c, spi, adc, gpio, timer_b0, imu} = board;

    // LoRa radio
    used.lora_reset.set_low();
    delay.delay_ms(1); // > 100 us
    used.lora_reset.set_high();
    delay.delay_ms(5);
    let radio = crate::lora::new(board.spi, used.lora_cs, used.lora_reset, board.delay);

    // GPS
    used.gps_reset_pin.set_low();
    delay.delay_ms(1);
    used.gps_reset_pin.set_high();
    let (tx, rx) = SerialConfig::new(eusci_a1, 
        BitOrder::LsbFirst, 
        BitCount::EightBits, 
        StopBits::OneStopBit, 
        Parity::NoParity, 
        Loopback::NoLoop, 
        crate::gps::GPS_BAUDRATE)
        .use_smclk(&smclk)
        .split(used.gps_tx_pin, used.gps_rx_pin);
    let gps = crate::gps::Gps::new(tx, rx);

    Stack {barometer, delay, gps, i2c, spi, adc, radio, gpio, timer_b0, imu}
}

/// Configure the MCU board, plus give back some unused bits used by other PCBs if they need to be configured later.
fn board_config(regs: Peripherals) -> (McuBoard, Smclk, Aclk, ExternalUsedPins, E_USCI_A1) {
    // Disable watchdog
    let _wdt = Wdt::constrain(regs.WDT_A);

    // Configure GPIO. `used` are pins consumed by other peripherals.
    let (mut gpio, used, external_pins) = Gpio::configure(regs.P1, regs.P2, regs.P3, regs.P4, regs.P5, regs.P6, regs.PMM);
    
    // Configure clocks to get accurate delay timing, and used by other peripherals
    let mut fram = Fram::new(regs.FRCTL);
    let (smclk, aclk, delay) = ClockConfig::new(regs.CS)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk() // 32768 Hz
        .freeze(&mut fram);

    // Spare UART, useful for debug printing to a computer
    crate::serial::configure_debug_serial(used.debug_tx_pin, &smclk, regs.E_USCI_A0);
    println!("Serial init"); // Like this!
    
    // SPI, used by the LoRa radio
    const SPI_FREQ_HZ: u32 = 250_000; // 250kHz is arbitrary
    let clk_div = (smclk.freq() / SPI_FREQ_HZ) as u16;
    let spi_bus = SpiConfig::new(regs.E_USCI_B1, embedded_hal::spi::MODE_0, true)
        .to_master_using_smclk(&smclk, clk_div) 
        .single_master_bus(used.miso, used.mosi, used.sclk);
    
    // In case we have multiple devices that need to share the SPI bus, we wrap in a RefCell to tell compiler
    // we will have multiple mutable references but will ensure that we only ever use one at a time 
    // (MSP430 is single threaded, so this is equivalent to not sending in an interrupt).
    // StaticCell is just so we can get a 'static reference and don't have to add generic lifetimes to everything.
    static SPI: StaticCell<RefCell<Spi<E_USCI_B1>>> = StaticCell::new();
    let spi: &'static _ = SPI.init(RefCell::new(spi_bus));

    // Timer
    let timer_parts = TimerParts3::new(regs.TB0, TimerConfig::aclk(&aclk));
    let timer_b0 = timer_parts.timer;

    // I2C
    const I2C_FREQ_HZ: u32 = 100_000;
    let clk_div = (smclk.freq() / I2C_FREQ_HZ) as u16;
    let i2c = I2cConfig::new(
        regs.E_USCI_B0, 
        GlitchFilter::Max50ns)
        .as_single_master()
        .use_smclk(&smclk, clk_div)
        .configure(used.i2c_scl_pin, used.i2c_sda_pin);

    // Again, make a static reference so we don't have to deal with lifetimes
    static I2C: StaticCell<RefCell<SensorI2c>> = StaticCell::new();
    let i2c: &'static RefCell<SensorI2c> = I2C.init(RefCell::new(i2c));

    // ADC
    let adc = AdcConfig::new(
        ClockDivider::_1, 
        Predivider::_1, 
        Resolution::_10BIT, 
        SamplingRate::_50KSPS, 
        SampleTime::_8)
        .use_modclk()
        .configure(regs.ADC);

    // Barometer
    gpio.bmp390_adr_pin.set_low().ok();
    let config = bmp390::Configuration::default();
    let barometer = Bmp390::try_new(I2cRefCellDevice::new(i2c), bmp390::Address::Down, delay, &config).unwrap(); // TODO: unwrap

    // IMU
    gpio.imu_adr_pin.set_low().ok();
    let imu = Imu::new(I2cRefCellDevice::new(i2c), icm42670::Address::Primary).unwrap(); // TODO: unwrap

    (McuBoard {barometer, delay, i2c: I2cRefCellDevice::new(i2c), spi, imu, adc, timer_b0, gpio}, smclk, aclk, external_pins, regs.E_USCI_A1)
}

/// The RGB LEDs are active low, which can be a little confusing. A helper struct to reduce cognitive load.
pub struct Led<PIN: StatefulOutputPin<Error=Infallible>>(PIN);
impl<PIN: StatefulOutputPin<Error=Infallible>> Led<PIN> {
    pub fn new(pin: PIN) -> Self {
        Self(pin)
    }
    pub fn turn_on(&mut self) {
        self.0.set_low();
    }
    pub fn turn_off(&mut self) {
        self.0.set_high();
    }
    pub fn toggle(&mut self) {
        self.0.toggle();
    }
}
pub type RedLed     = Led<RedLedPin>;
pub type BlueLed    = Led<BlueLedPin>;
pub type GreenLed   = Led<GreenLedPin>;

pub struct Gpio {
    // LEDs
    pub red_led:   RedLed,
    pub green_led: GreenLed,
    pub blue_led:  BlueLed,
    
    pub gps_en:         GpsEnPin,
    pub half_vbat:      HalfVbatPin,

    // PSU monitoring and control pins
    pub power_good_1v8: PowerGood1v8Pin,
    pub power_good_3v3: PowerGood3v3Pin,
    pub enable_1v8:     Enable1v8Pin,
    pub enable_5v:      Enable5vPin,

    // Barometer and IMU pins
    bmp390_adr_pin: Bmp390AddressPin,
    bmp390_int_pin: Bmp390InterruptPin,

    imu_adr_pin:    ImuAddressPin,
    imu_int1_pin:   ImuInterrupt1Pin,
    imu_int2_pin:   ImuInterrupt2Pin,

    // Unused UCA0 pins
    pub pin1_4: Pin<P1, Pin4, Input<Floating>>,
    pub pin1_5: Pin<P1, Pin5, Input<Floating>>,
    pub pin1_6: Pin<P1, Pin6, Input<Floating>>,

    // Unused UCA1 pins
    pub pin4_0: Pin<P4, Pin0, Input<Floating>>,
    
    // Unused ADC pins
    pub pin5_1: Pin<P5, Pin1, Input<Floating>>,
    pub pin5_3: Pin<P5, Pin3, Input<Floating>>,

    // Unused GPIO pins
    pub pin2_3: Pin<P2, Pin3, Input<Floating>>,
    pub pin2_4: Pin<P2, Pin4, Input<Floating>>,
    pub pin2_5: Pin<P2, Pin5, Input<Floating>>,
    pub pin2_6: Pin<P2, Pin6, Input<Floating>>,
    pub pin2_7: Pin<P2, Pin7, Input<Floating>>,

    pub pin3_4: Pin<P3, Pin4, Input<Floating>>,
    pub pin3_5: Pin<P3, Pin5, Input<Floating>>,
    pub pin3_6: Pin<P3, Pin6, Input<Floating>>,
    pub pin3_7: Pin<P3, Pin7, Input<Floating>>,

    pub pin6_0: Pin<P6, Pin0, Input<Floating>>,
    pub pin6_1: Pin<P6, Pin1, Input<Floating>>,
    pub pin6_7: Pin<P6, Pin7, Input<Floating>>,
}
impl Gpio {
    fn configure(p1: P1, p2: P2, p3: P3, p4 :P4, p5: P5, p6: P6, pmm: PMM) -> (Self, InternalUsedPins, ExternalUsedPins) {
        // Configure GPIO
        let pmm = Pmm::new(pmm);
        let port1 = Batch::new(p1).split(&pmm);
        let port2 = Batch::new(p2).split(&pmm);
        let port3 = Batch::new(p3).split(&pmm);
        let port4 = Batch::new(p4).split(&pmm);
        let port5 = Batch::new(p5).split(&pmm);
        let port6 = Batch::new(p6).split(&pmm);

        let half_vbat = port5.pin0.to_alternate3(); // ADC pin. Connected to Vbat/2.

        // LEDs
        let mut red_led = RedLed::new(port2.pin0.to_output());
        let mut blue_led = BlueLed::new(port2.pin1.to_output());
        let mut green_led = GreenLed::new(port2.pin2.to_output());
        red_led.turn_off();
        green_led.turn_off();
        blue_led.turn_off();

        let miso = port4.pin7.to_alternate1();
        let mosi = port4.pin6.to_alternate1();
        let sclk = port4.pin5.to_alternate1();
        
        let mut lora_reset = port1.pin1.to_output();
        let mut lora_cs = port4.pin4.to_output();
        lora_reset.set_high();
        lora_cs.set_high();

        let gps_tx_pin = port4.pin3.to_alternate1();
        let gps_rx_pin = port4.pin2.to_alternate1();
        let mut gps_reset_pin = port1.pin0.to_output();
        gps_reset_pin.set_high();
        let mut gps_en = port4.pin1.to_output(); // active low
        gps_en.set_low();

        let debug_tx_pin = port1.pin7.to_alternate1();

        let i2c_sda_pin = port1.pin2.to_alternate1();
        let i2c_scl_pin = port1.pin3.to_alternate1();

        let bmp390_adr_pin = port6.pin2.to_output();
        let bmp390_int_pin = port6.pin3;

        let imu_int1_pin = port6.pin5;
        let imu_int2_pin = port6.pin6;
        let mut imu_adr_pin = port6.pin4.to_output();
        imu_adr_pin.set_low();

        // Pins consumed by other perihperals
        let used_internal = InternalUsedPins {mosi, miso, sclk, debug_tx_pin, i2c_scl_pin, i2c_sda_pin};
        let used_external = ExternalUsedPins {lora_cs, lora_reset, gps_rx_pin, gps_tx_pin, gps_reset_pin};

        let pin1_4 = port1.pin4;
        let pin1_5 = port1.pin5;
        let pin1_6 = port1.pin6;

        let pin2_3 = port2.pin3;
        let pin2_4 = port2.pin4;
        let pin2_5 = port2.pin5;
        let pin2_6 = port2.pin6;
        let pin2_7 = port2.pin7;

        let power_good_1v8 = port3.pin0.pullup();
        let power_good_3v3 = port3.pin1.pullup();
        let mut enable_1v8 = port3.pin2.to_output();
        enable_1v8.set_low();
        let mut enable_5v = port3.pin3.to_output();
        enable_5v.set_low();
        let pin3_4 = port3.pin4;
        let pin3_5 = port3.pin5;
        let pin3_6 = port3.pin6;
        let pin3_7 = port3.pin7;

        let pin4_0 = port4.pin0;

        let pin5_1 = port5.pin1;
        let pin5_3 = port5.pin3;

        let pin6_0 = port6.pin0;
        let pin6_1 = port6.pin1;
        let pin6_7 = port6.pin7;

        let gpio = Self {
            red_led, green_led, blue_led, 
            gps_en, 
            half_vbat, 
            power_good_1v8, power_good_3v3, 
            enable_1v8, enable_5v,
            bmp390_adr_pin, bmp390_int_pin,
            imu_adr_pin, imu_int1_pin, imu_int2_pin,
            pin1_4, pin1_5, pin1_6,
            pin2_3, pin2_4, pin2_5, pin2_6, pin2_7,
            pin3_4, pin3_5, pin3_6, pin3_7,
            pin4_0,
            pin5_1, pin5_3,
            pin6_0, pin6_1, pin6_7,
        };

        (gpio, used_internal, used_external)
    }
}

/// Pins used by other peripherals.
struct InternalUsedPins {
    miso:           SpiMisoPin,
    mosi:           SpiMosiPin,
    sclk:           SpiSclkPin,
    i2c_sda_pin:    I2cSdaPin,
    i2c_scl_pin:    I2cSclPin,
    debug_tx_pin:   DebugTxPin,
}

/// Pins used by things on other boards
struct ExternalUsedPins {
    lora_reset:     LoraResetPin,
    lora_cs:        LoraCsPin,
    gps_tx_pin:     GpsTxPin,
    gps_rx_pin:     GpsRxPin,
    gps_reset_pin:  GpsResetPin,
}