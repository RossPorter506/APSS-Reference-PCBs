// An example board support package for a stack using the MCU and Beacon boards.

#![allow(dead_code)]
use bmp390::sync::Bmp390;
use defmt::{debug, info, panic, println, trace, unwrap};
use embedded_storage::nor_flash::NorFlash;
use mx25v::blocking::MX25V1606;
use static_assertions::assert_type_eq_all;
use uom::si::{f32::Pressure, pressure::pascal};
use core::{cell::RefCell, convert::Infallible, ops::{Deref, DerefMut, Range}};
use msp430fr2355::Peripherals;
use msp430fr2x5x_hal::{
    adc::{Adc, AdcConfig, ClockDivider, Predivider, Resolution, SampleTime, SamplingRate}, 
    clock::{Aclk, Clock, ClockConfig, DcoclkFreqSel, MclkDiv, Smclk, SmclkDiv}, 
    delay::SysDelay, 
    fram::Fram, 
    gpio::{Batch, Floating, Input, Output, P1, P2, P3, P4, P5, P6, Pin, Pin0, Pin1, Pin3, Pin4, Pin5, Pin6, Pin7, Pulldown}, 
    i2c::{GlitchFilter, I2cConfig}, 
    info_mem::InfoMemory, 
    pac::TB0, 
    pmm::{InternalVRef, Pmm, ReferenceVoltage}, 
    pwm::TimerConfig, 
    rtc::{Rtc, RtcSmclk}, 
    serial::{BitCount, BitOrder, Loopback, Parity, SerialConfig, StopBits}, 
    spi::SpiConfig, 
    timer::{Timer, TimerParts3}, 
    watchdog::Wdt
};
use embedded_hal::{delay::DelayNs, digital::{InputPin, OutputPin, StatefulOutputPin}};
use embedded_hal_bus::{i2c::RefCellDevice as I2cRefCellDevice, spi::RefCellDevice as SpiRefCellDevice};
use static_cell::StaticCell;
use crate::{MAIN_LOOP_FREQ_HZ, SensorData, State, Timestamps, gps::{Gps, UtcTime}, icm42670::Imu, led::RgbLed, lora::Radio, pin_mappings::*};

/// CS pin automatically managed
type ManagedSpi<ChipSel> = SpiRefCellDevice<'static, SensorSpi, ChipSel, SysDelay>; 
/// No auto-managed CS pin
type BareSpi = RefCell<SensorSpi>;
type SharedI2c = I2cRefCellDevice<'static, SensorI2c>;

pub type FlashMem = MX25V1606<ManagedSpi<FlashCsPin>>;

/// Top-level object representing the MCU board. 
// Not all peripherals are configured. If you need more, add their configuration code to board_config().
pub struct McuBoard {
    pub barometer: Bmp390<SharedI2c>,
    pub flash_mem: FlashMem,
    pub imu: Imu<SharedI2c>,
    pub delay: SysDelay,
    pub i2c: SharedI2c,
    pub spi: &'static BareSpi,
    pub adc: Adc,
    pub gpio: Gpio,
    pub status_led: StatusLed,
    pub timer_b0: Timer<TB0>,
    pub vref: InternalVRef,
    pub nvmem: NonvolatileMemory,
    bctl0_pin: Bctl0Pin,
    bctl1_pin: Bctl1Pin,
    pub rtc: Rtc<RtcSmclk>,
}
// This is where you should implement top-level functionality. 
impl McuBoard {
    pub fn battery_voltage_mv(&mut self) -> u16 {
        nb::block!( self.adc.read_voltage_mv(&mut self.gpio.half_vbat, 3300) ).unwrap() * 2 // Safe to unwrap
    }
    pub fn beacon_mode(&mut self, mode: BeaconMode) { // Yes this involves the beacon board, but it uses the MCU GPIO pins
        // The following code relies on these assertions. If these pins are changed you will also have to change the code below.
        assert_type_eq_all!(Bctl0Pin, Pin<P4, Pin1, Output>);
        assert_type_eq_all!(Bctl1Pin, Pin<P4, Pin0, Output>);
        // Write to the pout register directly so the two pins are updated at the same instant
        let p4 = unsafe { msp430fr2355::Peripherals::steal().P4 }; // Safety: We already own P4

        match mode {
            BeaconMode::AutoSleep => { // 0b00
                p4.p4out.modify(|r,w| unsafe{ w.bits(r.bits() & 0xFC) });
            },
            BeaconMode::AutoBeep => { // 0b01
                p4.p4out.modify(|r,w| unsafe{ w.bits((r.bits() & 0xFC) | 0b01) });
            },
            BeaconMode::AutoActive => { // 0b10
                p4.p4out.modify(|r,w| unsafe{ w.bits((r.bits() & 0xFC) | 0b10) });
            },
            BeaconMode::Manual => { // 0b11
                p4.p4out.modify(|r,w| unsafe{ w.bits(r.bits() | 0b11) });
            },
        };
    }

    /// Whether the board is armed. Until the board is armed it's locked into the 'idle' state.
    pub fn is_armed(&mut self) -> bool {
        self.gpio.arm_pin.is_low().unwrap() // Infallible unwrap
    }

    /// Whether the board is disarmed. When disarmed the board is locked into the 'recovered' state.
    pub fn is_disarmed(&mut self) -> bool {
        self.gpio.disarm_pin.is_high().unwrap() // Infallible unwrap
    }

    /// Write to the flash memory, wrapping around the address space as needed.
    pub fn flash_write_wrapping(&mut self, data: &[u8], write_addr: u32) -> u32 {
        let capacity = FlashMem::CAPACITY;
        debug!("Flash write:    {:?}", data); 
        if write_addr + data.len() as u32 <= capacity {
            self.flash_mem.write(write_addr, data).unwrap(); // Unwrap safe
            
            let mut read_buf = [0u8; SensorData::MAX_SIZE]; // TODO: Remove readback
            let read_buff = &mut read_buf[0..data.len()];
            self.flash_mem.read(write_addr, read_buff).unwrap();
            debug!("Flash readback: {:?}", read_buff);
            write_addr + data.len() as u32
        } else { // write wraps around address space
            let remaining = (capacity - write_addr) as usize;
            let (data1, data2) = data.split_at(remaining);
            self.flash_mem.write(write_addr, data1).unwrap(); // Unwrap safe
            self.flash_mem.write(0, data2).unwrap(); //  Unwrap safe
            data2.len() as u32
        }
    }
}

pub struct NonvolatileMemory {
    info_mem: &'static mut [u8; 512]
}
impl NonvolatileMemory {
    pub fn new(info_mem: &'static mut [u8; 512]) -> Self {
        Self { info_mem }
    }

    // Info mem contents by address:
    // 0      - Current state
    // 1..6   - Current time
    // 6..31  - Transition timestamps
    // 31     - Reset counter
    // 32..36 - Current flash memory address
    // 36..40 - Reference pressure
    const STATE_ADDR: usize = 0;
    const CURRENT_TIME_ADDR: Range<usize> = 1..6;
    const TRANSITIONS_ADDR: Range<usize> = 6..31;
    const RESET_COUNTER_ADDR: usize = 31;
    const WRITE_ADDR_ADDR: Range<usize> = 32..36;
    const REF_PRESSURE_ADDR: Range<usize> = 36..40;

    pub fn try_get_state(&self) -> Option<State> {
        State::try_from_u8(self.info_mem[Self::STATE_ADDR])
    }
    pub fn store_state(&mut self, state: State) {
        self.info_mem[Self::STATE_ADDR] = state as u8;
    }

    pub fn try_get_current_time(&self) -> Option<UtcTime> {
        UtcTime::try_from_bytes(unwrap!(self.info_mem[Self::CURRENT_TIME_ADDR].try_into()))
    }
    pub fn store_current_time(&mut self, time: &UtcTime) {
        self.info_mem[Self::CURRENT_TIME_ADDR].copy_from_slice(&time.as_bytes());
    }

    pub fn get_transitions(&self) -> Timestamps {
        Timestamps::from_bytes(unwrap!(self.info_mem[Self::TRANSITIONS_ADDR].try_into()))
    }
    pub fn store_transitions(&mut self, transitions: &Timestamps) {
        self.info_mem[Self::TRANSITIONS_ADDR].copy_from_slice(&transitions.as_bytes())
    }

    pub fn get_num_resets(&self) -> u8 {
        self.info_mem[Self::RESET_COUNTER_ADDR]
    }
    pub fn store_num_resets(&mut self, n_resets: u8) {
        self.info_mem[Self::RESET_COUNTER_ADDR] = n_resets;
    }
    pub fn increment_resets(&mut self) {
        self.info_mem[Self::RESET_COUNTER_ADDR] += 1;
    }

    /// Reads the flash memory current write address from memory. If it's invalid then `0` is returned.
    pub fn get_flash_write_addr(&self) -> u32 {
        let addr = u32::from_le_bytes(unwrap!(self.info_mem[Self::WRITE_ADDR_ADDR].try_into()));
        if addr >= FlashMem::CAPACITY {0} else {addr}
    }
    pub fn store_flash_write_addr(&mut self, write_addr: u32) {
        self.info_mem[Self::WRITE_ADDR_ADDR].copy_from_slice(&write_addr.to_le_bytes())
    }

    pub fn try_get_calibration_pressure(&self) -> Option<Pressure> {
        let pressure = f32::from_le_bytes(unwrap!(self.info_mem[Self::REF_PRESSURE_ADDR].try_into()));
        if (10_000.0..150_000.0).contains(&pressure) { // Range is arbitrary
            Some(Pressure::new::<pascal>(pressure))
        } else {
            None
        }
    }
    pub fn store_calibration_pressure(&mut self, pressure: f32) {
        self.info_mem[Self::REF_PRESSURE_ADDR].copy_from_slice(&pressure.to_le_bytes())
    }

    pub fn erase_all(&mut self) {
        self.info_mem.fill(0);
    }
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// The mode that the beacon board is in. Driven by the bctrl0 and bctrl1 pins.
pub enum BeaconMode {
    /// Do nothing
    AutoSleep = 0b00,
    /// Buzzer active
    AutoBeep = 0b01,
    /// Buzzer active and forwards GPS packets to radio
    AutoActive = 0b10,
    /// Exposes peripherals to the stack to be driven by something else.
    Manual = 0b11,
}

/// Top-level object representing the a stack of PCBs (namely MCU + Beacon).
/// 
// If you need more, add their configuration code to in_stack().
pub struct Stack {
    pub board: McuBoard,
    pub gps: Gps,
    pub radio: Radio,
}
// This is where you should implement top-level functionality unique to a stack. 
impl Stack {

}
// Transparently implement all McuBoard methods and fields onto Stack
// This means we can specify `stack.imu` and have it coerced to `stack.board.imu`, for instance.
impl Deref for Stack {
    type Target = McuBoard;
    fn deref(&self) -> &Self::Target {
        &self.board
    }
}
impl DerefMut for Stack {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.board
    }
}

/// Configure just the MCU board. The other PCBs need not be attached.
pub fn standalone(regs: Peripherals) -> McuBoard {
    let (board, ..) = board_config(regs);
    board
}

/// Configure the entire stack of PCBs (MCU + Beacon). The Beacon must be attached for this to succeed.
pub fn in_stack(regs: Peripherals) -> Stack {
    let (mut board, smclk, _aclk, mut used, eusci_a1) = board_config(regs);
    board.beacon_mode(BeaconMode::Manual);
    // Wait for beacon board to initialise
    board.delay.delay_ms(200); //TODO: Figure out how long this actually needs to be

    // LoRa radio
    used.lora_reset.set_low();
    board.delay.delay_ms(1); // > 100 us
    used.lora_reset.set_high();
    board.delay.delay_ms(5);
    let radio_spi = unwrap!(SpiRefCellDevice::new(board.spi, used.lora_cs, board.delay));
    let radio = crate::lora::new(radio_spi, used.lora_reset, board.delay);

    // GPS
    used.gps_reset_pin.set_low();
    board.delay.delay_ms(1);
    used.gps_reset_pin.set_high();
    let (tx, mut rx) = SerialConfig::new(eusci_a1, 
        BitOrder::LsbFirst, 
        BitCount::EightBits, 
        StopBits::OneStopBit, 
        Parity::NoParity, 
        Loopback::NoLoop, 
        crate::gps::GPS_BAUDRATE)
        .use_smclk(&smclk)
        .split(used.gps_tx_pin, used.gps_rx_pin);
    rx.enable_rx_interrupts();
    let gps = crate::gps::Gps::new(tx, rx);

    Stack {board, gps, radio}
}

/// Configure the MCU board, plus give back some unused bits used by other PCBs if they need to be configured later.
fn board_config(regs: Peripherals) -> (McuBoard, Smclk, Aclk, ExternalUsedPins, GpsEusci) {
    // Disable watchdog
    let _wdt = Wdt::constrain(regs.WDT_A);

    // Configure GPIO. `used` are pins consumed by other peripherals.
    let mut pmm = Pmm::new(regs.PMM);
    let (mut gpio, used, external_pins, status_led) = Gpio::configure(regs.P1, regs.P2, regs.P3, regs.P4, regs.P5, regs.P6, &pmm);
    let (bctl0_pin, bctl1_pin) = (used.bctl0_pin, used.bctl1_pin);
    
    // Configure clocks to get accurate delay timing, and used by other peripherals
    let mut fram = Fram::new(regs.FRCTL);
    let (smclk, aclk, delay) = ClockConfig::new(regs.CS)
        .mclk_dcoclk(DcoclkFreqSel::_24MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_refoclk() // 32768 Hz
        .freeze(&mut fram);

    // Spare UART, useful for debug printing to a computer
    crate::serial::configure_debug_serial(used.debug_tx_pin, &smclk, regs.E_USCI_A0);
    println!("");
    trace!("Serial init"); // Like this!
    
    // SPI, used by the LoRa radio
    const SPI_FREQ_HZ: u32 = 8_000_000; // Max MSP430 speed
    let clk_div = (smclk.freq() / SPI_FREQ_HZ) as u16;
    let spi_bus = SpiConfig::new(regs.E_USCI_B1, embedded_hal::spi::MODE_0, true)
        .to_master_using_smclk(&smclk, clk_div) 
        .single_master_bus(used.miso, used.mosi, used.sclk);
    
    // In case we have multiple devices that need to share the SPI bus, we wrap in a RefCell to tell compiler
    // we will have multiple mutable references but will ensure that we only ever use one at a time 
    // (MSP430 is single threaded, so this is equivalent to not sending in an interrupt).
    // StaticCell is just so we can get a 'static reference and don't have to add generic lifetimes to everything.
    static SPI: StaticCell<BareSpi> = StaticCell::new();
    let spi: &'static BareSpi = SPI.init(RefCell::new(spi_bus));

    // Timer
    let timer_parts = TimerParts3::new(regs.TB0, TimerConfig::aclk(&aclk));
    let timer_b0 = timer_parts.timer;

    // I2C
    const I2C_FREQ_HZ: u32 = 400_000; // Max MSP430 speed
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

    let mut rtc = Rtc::new(regs.RTC).use_smclk(&smclk);
    rtc.set_clk_div(msp430fr2x5x_hal::rtc::RtcDiv::_64);
    rtc.enable_interrupts();
    rtc.start((smclk.freq() / (64*MAIN_LOOP_FREQ_HZ as u32)) as u16);
    const { assert!( MAIN_LOOP_FREQ_HZ < 1000 && MAIN_LOOP_FREQ_HZ > 10) }
    // const {assert!( (MCLK.freq() / (prescaler.value() * TARGET_LOOP_FREQ_HZ)) < u16::MAX )} // TODO: Make the necessary functions for this assert const in the HAL

    let vref = unwrap!(pmm.enable_internal_reference(ReferenceVoltage::_2V5)); // Safe

    let (info_mem, _) = InfoMemory::as_u8s(regs.SYS);
    let info_mem = NonvolatileMemory::new(info_mem);

    // Barometer
    gpio.bmp390_adr_pin.set_low().ok();
    use bmp390::{PowerControl, PowerMode, Osr, Oversampling, Odr, OdrSel, IirFilter, Config};
    let config = bmp390::Configuration { 
        power_control: PowerControl { enable_pressure: true, enable_temperature: true, mode: PowerMode::Normal }, 
        oversampling: Osr { pressure: Oversampling::X2, temperature: Oversampling::X2 }, 
        output_data_rate: Odr { odr_sel: OdrSel::ODR_100 }, 
        iir_filter: Config { iir_filter: IirFilter::coef_1 } };
    let barometer = Bmp390::try_new(I2cRefCellDevice::new(i2c), bmp390::Address::Down, delay, &config).unwrap(); // TODO: unwrap

    // IMU
    gpio.imu_adr_pin.set_low().ok();
    let imu = Imu::new(I2cRefCellDevice::new(i2c), icm42670::Address::Primary).unwrap(); // TODO: unwrap

    // Flash memory
    let flash_spi = unwrap!(SpiRefCellDevice::new(spi, used.flash_cs_pin, delay));
    let flash_mem = MX25V1606::new(flash_spi);

    (McuBoard {barometer, delay, i2c: I2cRefCellDevice::new(i2c), imu, spi, flash_mem, adc, status_led, timer_b0, gpio, vref, nvmem: info_mem, bctl0_pin, bctl1_pin, rtc}, smclk, aclk, external_pins, regs.E_USCI_A1)
}

pub struct Gpio {
    pub half_vbat:      HalfVbatPin,

    // PSU monitoring and control pins
    pub power_good_1v8: PowerGood1v8Pin,
    pub power_good_3v3: PowerGood3v3Pin,
    pub enable_1v8:     Enable1v8Pin,
    pub enable_5v:      Enable5vPin,

    // Barometer pins
    bmp390_adr_pin: Bmp390AddressPin,
    bmp390_int_pin: Bmp390InterruptPin,

    // IMU pins
    imu_adr_pin:    ImuAddressPin,
    imu_int1_pin:   ImuInterrupt1Pin,
    imu_int2_pin:   ImuInterrupt2Pin,

    // Flash pins
    pub flash_wp_pin: FlashWpPin,

    // Arm / disarm jumper pins
    pub pin3_6: Pin<P3, Pin6, Output>,
    pub arm_pin: Pin<P2, Pin3, Input<Pulldown>>,
    pub disarm_pin: Pin<P3, Pin7, Input<Pulldown>>,

    // Unused UCA0 pins
    pub pin1_4: Pin<P1, Pin4, Input<Floating>>,
    pub pin1_5: Pin<P1, Pin5, Input<Floating>>,
    pub pin1_6: Pin<P1, Pin6, Input<Floating>>,

    // Unused ADC pins
    pub pin5_1: Pin<P5, Pin1, Input<Floating>>,
    pub pin5_3: Pin<P5, Pin3, Input<Floating>>,

    // Unused GPIO pins
    pub pin2_6: Pin<P2, Pin6, Input<Floating>>,
    pub pin2_7: Pin<P2, Pin7, Input<Floating>>,

    pub pin3_4: Pin<P3, Pin4, Input<Floating>>,
    pub pin3_5: Pin<P3, Pin5, Input<Floating>>,

    pub pin6_0: Pin<P6, Pin0, Input<Floating>>,
    pub pin6_1: Pin<P6, Pin1, Input<Floating>>,
    pub pin6_7: Pin<P6, Pin7, Input<Floating>>,
}
impl Gpio {
    fn configure(p1: P1, p2: P2, p3: P3, p4 :P4, p5: P5, p6: P6, pmm: &Pmm) -> (Self, InternalUsedPins, ExternalUsedPins, StatusLed) {
        // Configure GPIO
        let port1 = Batch::new(p1).split(pmm);
        let port2 = Batch::new(p2).split(pmm);
        let port3 = Batch::new(p3).split(pmm);
        let port4 = Batch::new(p4).split(pmm);
        let port5 = Batch::new(p5).split(pmm);
        let port6 = Batch::new(p6).split(pmm);

        let half_vbat = port5.pin0.to_alternate3(); // ADC pin. Connected to Vbat/2.

        let mut bctl0_pin = port4.pin1.to_output();
        bctl0_pin.set_low(); 
        let mut bctl1_pin = port4.pin0.to_output();
        bctl1_pin.set_low(); 

        // LEDs
        let status_led = RgbLed::new(
            RedLed::new(port2.pin0.to_output()), 
            GreenLed::new(port2.pin2.to_output()), 
            BlueLed::new(port2.pin1.to_output())
        );

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

        let debug_tx_pin = port1.pin7.to_alternate1();

        let i2c_sda_pin = port1.pin2.to_alternate1();
        let i2c_scl_pin = port1.pin3.to_alternate1();

        let bmp390_adr_pin = port6.pin2.to_output();
        let bmp390_int_pin = port6.pin3;

        let imu_int1_pin = port6.pin5;
        let imu_int2_pin = port6.pin6;
        let mut imu_adr_pin = port6.pin4.to_output();
        imu_adr_pin.set_low();

        let mut flash_wp_pin = port2.pin4.to_output();
        flash_wp_pin.set_high();
        let mut flash_cs_pin = port2.pin5.to_output();
        flash_cs_pin.set_high();


        // Pins consumed by other perihperals
        let used_internal = InternalUsedPins {mosi, miso, sclk, debug_tx_pin, i2c_scl_pin, i2c_sda_pin, flash_cs_pin, bctl0_pin, bctl1_pin};
        let used_external = ExternalUsedPins {lora_cs, lora_reset, gps_rx_pin, gps_tx_pin, gps_reset_pin};

        let pin1_4 = port1.pin4;
        let pin1_5 = port1.pin5;
        let pin1_6 = port1.pin6;

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

        // Used as a common for disarm and arm
        let mut pin3_6 = port3.pin6.to_output();
        pin3_6.set_high();
        let disarm_pin = port3.pin7.pulldown();
        let arm_pin = port2.pin3.pulldown();

        let pin5_1 = port5.pin1;
        let pin5_3 = port5.pin3;

        let pin6_0 = port6.pin0;
        let pin6_1 = port6.pin1;
        let pin6_7 = port6.pin7;

        let gpio = Self {
            half_vbat, 
            power_good_1v8, power_good_3v3, 
            enable_1v8, enable_5v,
            bmp390_adr_pin, bmp390_int_pin,
            imu_adr_pin, imu_int1_pin, imu_int2_pin,
            flash_wp_pin,
            arm_pin, disarm_pin,
            pin1_4, pin1_5, pin1_6,
            pin2_6, pin2_7,
            pin3_4, pin3_5, pin3_6,
            pin5_1, pin5_3,
            pin6_0, pin6_1, pin6_7,
        };

        (gpio, used_internal, used_external, status_led)
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
    flash_cs_pin:   FlashCsPin,
    bctl0_pin:      Bctl0Pin,
    bctl1_pin:      Bctl1Pin
}

/// Pins used by things on other boards
struct ExternalUsedPins {
    lora_reset:     LoraResetPin,
    lora_cs:        LoraCsPin,
    gps_tx_pin:     GpsTxPin,
    gps_rx_pin:     GpsRxPin,
    gps_reset_pin:  GpsResetPin,
}