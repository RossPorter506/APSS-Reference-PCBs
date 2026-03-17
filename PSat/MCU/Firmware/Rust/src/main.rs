#![no_main]
#![no_std]
#![feature(asm_experimental_arch)]
#![feature(abi_msp430_interrupt)]

use core::{cell::RefCell, sync::atomic::Ordering};
use defmt::{Format, debug, error, expect, info, panic, println, trace, unwrap, warn};
use portable_atomic::AtomicBool;
use arrayvec::ArrayVec;
use ::icm42670::accelerometer::vector::{I16x3, I32x3};
use msp430::{critical_section, interrupt::{Mutex, enable as enable_interrupts}};
use msp430_rt::entry;
use msp430fr2x5x_hal::lpm::enter_lpm0;
use msp430fr2355::interrupt;
use ufmt::derive::uDebug;
use uom::si::{f32::Pressure, pressure::pascal, thermodynamic_temperature::degree_celsius};

// Internal modules
mod pin_mappings { include!("pin_mappings_v2_1.rs"); } // Import 'pin_mappings_v2_1' as 'pin_mappings'
mod board;
mod serial;
mod panic_handler;
mod lora;
mod gps;
mod icm42670;
mod defmt_impl;
mod led;

// Internal imports
use crate::{
    board::{BeaconMode, McuBoard, NonvolatileMemory, Stack}, 
    gps::{Altitude, Degrees, GGA_DATA, GPS_MSG_BUF, GgaMessage, GpsFixType, UtcTime},
    led::Led,
};

pub const TEAM_ID: u8 = 15;

// How often the main loop cycles through. If this is set faster than the main loop takes, then timekeeping will fail.
// Currently this period has to cleanly divide into 1000 due to `UtcTime::increment()`
const MAIN_LOOP_PERIOD_MS: u16 = 10;
const MAIN_LOOP_FREQ_HZ: u16 = 1000 / MAIN_LOOP_PERIOD_MS;

static CURRENT_TIME: Mutex<RefCell<UtcTime>> = Mutex::new(RefCell::new(UtcTime::new()));

type System = Stack;

// TODO: Make UtcTime::increment work for periods that don't evenly divide into 1000
// TODO: Turn on buzzer only when landed
// TODO: Test storing of GPS data (10Hz)
// TODO: Test transmission of GPS data (1Hz)
#[entry]
fn main() -> ! {
    let regs = unwrap!(msp430fr2355::Peripherals::take());
    
    // Configure the MCU board assuming no connections to other PCBs.
    let mut system = board::in_stack(regs); // Collect board elements, configure printing, etc.
    unsafe{ enable_interrupts(); }

    println!("Hello world!");

    // The 'fresh_start' feature will make the board reset from Idle every time.
    // Useful for testing, but for flight this should NOT be enabled
    // as otherwise any resets during flight will reset the state machine too.
    #[cfg(feature = "fresh_start")]
    system.nvmem.erase_all();

    // In case we just reset mid-flight, restore state, current time, any saved timestamps, etc.
    let current_time = system.nvmem.try_get_current_time().unwrap_or_default();
    // Load current time into global that's incremented by the RTC interrupt
    critical_section::with(|cs| CURRENT_TIME.replace(cs, current_time.clone()));
    // Load state machine data from non-volatile memory
    let mut state_machine = StateMachine::restore_from_nvmem(&mut system, current_time.clone());
    system.nvmem.increment_resets();
    
    loop {
        state_machine.process(&mut system);

        system.status_led.blue.on(); // Blue LED pin serves as a counter to show how often the MCU is idle
        enter_lpm0(); // Sleep until RTC wakes us up in MAIN_LOOP_PERIOD_MS.
        system.status_led.blue.off();

        // After RTC interrupt wakes us up the time is updated
        let new_time = critical_section::with(|cs| CURRENT_TIME.borrow_ref(cs).clone());
        system.nvmem.store_current_time(&new_time);
        state_machine.update_time(new_time);
    }
}

/// Top-level struct that manages mission status, states, actions, and transitions.
struct StateMachine {
    state: State,
    is_calibrated: bool,
    flash_write_addr: u32,
    ref_pressure: Pressure,
    current_time: UtcTime,
    timestamps: Timestamps,
    data: SensorData,
    gps_lock: bool,
}
impl StateMachine {
    /// Restore state machine data from non-volatile memory.
    pub fn restore_from_nvmem(system: &mut System, current_time: UtcTime) -> Self {
        let nvmem = &system.nvmem;
        let state            = nvmem.try_get_state().unwrap_or(State::Idle);
        info!("Initial state: {:?}", state);
        let is_calibrated = state > State::Calibration;
        let gps_lock = is_calibrated;

        let timestamps       = nvmem.get_transitions();
        let flash_write_addr = nvmem.get_flash_write_addr();
        let ref_pressure     = nvmem.try_get_calibration_pressure().unwrap_or(Pressure::new::<pascal>(101_325.0));

        let mut data = SensorData::new(current_time.clone());
        data.reset = true;

        let mut s = Self { state, is_calibrated, flash_write_addr, ref_pressure, current_time, timestamps, data, gps_lock};
        s.state_transition_actions(state, system);
        s
    }

    /// Update internal time and also clears data, ready for next iteration
    pub fn update_time(&mut self, current_time: UtcTime) {
        self.current_time = current_time.clone();
        self.data = SensorData::new(current_time);
    }

    /// Process state actions and update state
    pub fn process(&mut self, system: &mut System) {
        self.state_actions(system);
        let new_state = self.next_state(system);
        if new_state != self.state {
            self.state_transition_actions(new_state, system);
        }
        self.state = new_state;
    }

    /// Perform actions that should be executed while in a particular mode (Moore-style)
    fn state_actions(&mut self, system: &mut System) {
        use State::*;
        match self.state {
            Calibration | Preflight | Flight => {
                self.poll_gps();
            },
            _ => (),
        }
        match self.state {
            Calibration => {
                if !self.is_calibrated && let Ok(p) = system.barometer.pressure() {
                    unsafe{ info!("Reference pressure: {}", p.get::<pascal>().to_int_unchecked::<i32>()) };
                    self.ref_pressure = p;
                    system.nvmem.store_calibration_pressure(p.get::<pascal>());

                    system.nvmem.store_num_resets(0); // Reset count

                    info!("Starting flash erase");
                    system.flash_mem.erase_chip().unwrap(); // Unwrap safe
                    system.flash_mem.wait_wip().unwrap(); // Unwrap safe
                    info!("Flash erase complete");
                    self.flash_write_addr = 0;

                    self.is_calibrated = true;
                }
                self.gps_lock = critical_section::with(|cs| {
                    if let Some(ref gga) = *GGA_DATA.borrow_ref(cs) {
                        gga.fix_type != GpsFixType::None
                    } else { false }
                });
            },
            Preflight => {
                self.read_sensors(system);
            }
            Flight => {
                self.read_sensors(system);
                if self.data.is_some() {
                    self.flash_write_addr = system.flash_write_wrapping(self.data.encode().as_slice(), self.flash_write_addr);
                    system.nvmem.store_flash_write_addr(self.flash_write_addr);
                }
            },
            _ => (),
        };
    }

    fn poll_gps(&mut self) {
        if MSG_READY.load(Ordering::Relaxed) {
            critical_section::with(|cs| {
                let mut msg_buf = *GPS_MSG_BUF.borrow_ref_mut(cs);
                if let Ok(gga) =  GgaMessage::try_from(&msg_buf) {
                    debug!("Decoded GGA message: {:?}", gga);
                    GGA_DATA.replace(cs, Some(gga));
                }
                msg_buf.clear();
            });
            MSG_READY.store(false, Ordering::Relaxed);
        }
    }

    /// Perform actions that should occur once when transitioning between states (Mealy-style)
    fn state_transition_actions(&mut self, new_state: State, system: &mut System) {
        use State::*;
        match new_state {
            Idle | Recovered => system.beacon_mode(BeaconMode::AutoSleep),
            Calibration | Preflight | Flight => system.beacon_mode(BeaconMode::Manual),
            Landed => system.beacon_mode(BeaconMode::AutoActive),
        };
        info!("New state: {:?}", new_state);
        system.nvmem.store_state(new_state);
        system.nvmem.store_transitions(&self.timestamps);
    }

    /// Determine what the next state ought to be
    fn next_state(&mut self, system: &mut System) -> State {
        use State::*;

        // Short circuit to Recovered if disarmed
        if self.state != Landed && self.state != Recovered && system.is_disarmed() {
            self.data.transition = Some((self.state, Recovered));
            return Recovered;
        }

        match self.state {
            Idle => {
                if system.is_armed() {
                    info!("Moving to 'calibration'");
                    self.data.transition = Some((Idle, Calibration));
                    self.timestamps.calibration = Some(self.current_time.clone());
                    Calibration
                } else {
                    trace!("Still idle");
                    Idle
                }
            },
            Calibration => {
                const CALIBRATION_TIMEOUT_SEC: i32 = 2*60;
                let timeout = if let Some(calib_start) = &self.timestamps.calibration {
                    self.current_time.seconds_since(calib_start) > CALIBRATION_TIMEOUT_SEC
                } else { false };

                if (self.is_calibrated && self.gps_lock) || timeout {
                    if timeout {
                        warn!("Calibration timeout");
                    }
                    info!("Moving to preflight");
                    self.timestamps.preflight = Some(self.current_time.clone());
                    self.data.transition = Some((Calibration, Preflight));
                    Preflight
                } else {
                    trace!("Still calibrating");
                    Calibration
                }
            },
            Preflight => {
                let above_10m = self.data.baro.unwrap_or_default().2 > 10.0;
                let above_2gs = self.data.imu.unwrap_or_default().0.z < -2_000;

                if above_10m || above_2gs {
                    info!("Moving to flight");
                    self.timestamps.flight = Some(self.current_time.clone());
                    self.data.transition = Some((Preflight, Flight));
                    Flight
                } else {
                    trace!("Still preflight");
                    Preflight
                }
            },
            Flight => {
                let below_20m = if let Some(data) = self.data.baro {data.2 < 20.0} else { false };

                // Gyro should read zero degrees/sec in all directions if landed / staionary
                let not_rotating = if let Some(imu) = self.data.imu {
                    const TOLERANCE: u32 = 1_500; // 1.5 degrees per sec. Datasheet states zero rate output (ZRO) is +- 1 d/sec
                    let (x,y,z) = (imu.1.x, imu.1.y, imu.1.z);

                    x.unsigned_abs() < TOLERANCE && 
                    y.unsigned_abs() < TOLERANCE && 
                    z.unsigned_abs() < TOLERANCE 
                } else { false }; 
                
                // Accelerometer should read 1g if landed / stationary.
                let not_accelerating = if let Some(imu) = self.data.imu {
                    // Acceleration vectors (in milli-gees: 1000 = 1g)
                    let (x,y,z) = (imu.0.x as i32, imu.0.y as i32, imu.0.z as i32);

                    // 0.95g < sqrt(x^2 + y^2 + z^2) < 1.05g, but avoiding sqrt
                    let acceleration_squared = x*x + y*y + z*z;
                    const ONE_G: i32 = 1_000; // 1g
                    const TOLERANCE: i32 = 50; // +-0.05g
                    const LOWER_BOUND_SQUARED: i32 = (ONE_G-TOLERANCE)*(ONE_G-TOLERANCE);
                    const UPPER_BOUND_SQUARED: i32 = (ONE_G+TOLERANCE)*(ONE_G+TOLERANCE);

                    (LOWER_BOUND_SQUARED..UPPER_BOUND_SQUARED).contains(&acceleration_squared)
                } else { false };

                debug!("Below 20m: {}, not accelerating: {}, not rotating: {}", below_20m, not_accelerating, not_rotating);

                const TIMEOUT_DURATION_SEC: i32 = 60*8;
                let timeout = if let Some(flight_start) = &self.timestamps.flight {
                    self.current_time.seconds_since(flight_start) > TIMEOUT_DURATION_SEC
                } else { false };

                if (below_20m && not_accelerating && not_rotating) || timeout {
                    if timeout {
                        warn!("Flight timeout");
                    }
                    info!("Moving to 'landed'");
                    self.timestamps.landed = Some(self.current_time.clone());
                    self.data.transition = Some((Flight, Landed));
                    Landed
                } else {
                    trace!("Still flight");
                    Flight
                }
            },
            Landed => {
                if system.is_disarmed() {
                    info!("Moving to 'recovered'");
                    self.timestamps.recovered = Some(self.current_time.clone());
                    self.data.transition = Some((Landed, Recovered));
                    Recovered
                } else {
                    trace!("Still 'landed'");
                    Landed
                }
            },
            Recovered => Recovered,
        }
    }

    fn read_sensors(&mut self, system: &mut System) {
        const IMU_POLL_FREQ_HZ:    u16 = MAIN_LOOP_FREQ_HZ / 1; // poll as fast as possible
        const BARO_POLL_FREQ_HZ:   u16 = MAIN_LOOP_FREQ_HZ / 1;
        const GPS_POLL_FREQ_HZ:    u16 = 10;
        const BATT_POLL_FREQ_HZ:   u16 = 1;
        const RADIO_TX_FREQ_HZ:    u16 = 1;

        const IMU_POLL_PERIOD_MS:  u16 = 1000 / IMU_POLL_FREQ_HZ;
        const BARO_POLL_PERIOD_MS: u16 = 1000 / BARO_POLL_FREQ_HZ;
        const GPS_POLL_PERIOD_MS:  u16 = 1000 / GPS_POLL_FREQ_HZ;
        const BATT_POLL_PERIOD_MS: u16 = 1000 / BATT_POLL_FREQ_HZ;
        const RADIO_TX_PERIOD_MS:  u16 = 1000 / RADIO_TX_FREQ_HZ;

        if self.current_time.millis.is_multiple_of(IMU_POLL_PERIOD_MS)
        && let Ok((acc, gyro, temp)) = system.imu.measure_millis() {
                self.data.imu = Some((acc, gyro, temp));
                debug!("gyro x: {} md/s, y: {} md/s, z: {} md/s,", gyro.x, gyro.y, gyro.z);
                debug!("accel x: {} mgees, y: {} mgees, z: {} mgees,", acc.x, acc.y, acc.z);
            }
        if self.current_time.millis.is_multiple_of(BARO_POLL_PERIOD_MS)
        && let Ok((temperature, pressure)) = system.barometer.temperature_pressure() {
            let altitude = fast_altitude(pressure, self.ref_pressure);
            self.data.baro = Some((pressure.get::<pascal>(), temperature.get::<degree_celsius>(), altitude));
            debug!(
                "pressure: {} Pa, temperature: {} C, altitude: {} dm", 
                unsafe { pressure.get::<pascal>().to_int_unchecked::<i32>() }, 
                unsafe { temperature.get::<degree_celsius>().to_int_unchecked::<i32>() }, 
                unsafe { (10.0*altitude).to_int_unchecked::<i32>() }
            )
        }
        if self.current_time.millis.is_multiple_of(BATT_POLL_PERIOD_MS) {
            let bv = system.battery_voltage_mv();
            debug!("Battery voltage: {}mV", bv);
            self.data.battery = Some(bv);
        }
        if self.current_time.millis.is_multiple_of(GPS_POLL_PERIOD_MS) {
            let gga = critical_section::with(|cs| GGA_DATA.borrow_ref(cs).clone());
            debug!("Checking for gps data: {:?}", gga);
            self.data.gps = gga;
        }
        if self.current_time.millis.is_multiple_of(RADIO_TX_PERIOD_MS)
        && let Some(ref gga) = critical_section::with(|cs| GGA_DATA.borrow_ref(cs).clone())
        && system.radio.transmit_is_complete().is_ok() {
            debug!("Transmitting GPS data: {:?}", gga);
            let _ = system.radio.transmit_start(&gga.encode_binary());
        }
    }
}

#[derive(PartialEq, Eq, PartialOrd, Ord, Copy, Clone, Format)]
enum State {
    Idle = 0, // Powered on, but not armed.
    Calibration = 1, // Any pre-flight calibration, like BMP390 ref altitude, GPS lock, etc. 
    Preflight = 2, // On launchpad
    Flight = 3,
    Landed = 4, // On ground, beacon enabled
    Recovered = 5, // After recovery
}
impl State {
    pub fn try_from_u8(val: u8) -> Option<Self> {
        use State::*;
        Some(match val {
            0 => Idle,
            1 => Calibration,
            2 => Preflight,
            3 => Flight,
            4 => Landed,
            5 => Recovered,
            _ => return None,
        })
    }
}

/// Approximate pressure -> altitude conversion.
/// Based on the barometric formula. Assumes constant temperature across altitudes
pub fn fast_altitude(pressure: Pressure, reference_pressure: Pressure) -> f32 {
    const T: f32 = 273.15 + 25.0; // temperature (kelvin)
    const R: f32 = 8.31432; // universal gas constant J/(mol*K)
    const M: f32 = 0.0289644; // Molar mass of air (kg/mol)
    const G: f32 = 9.806; // gravitational acceleration (m/s^2)

    const COEFF: f32 = -R*T / (M*G); 

    if pressure >= reference_pressure {
        COEFF*fast_ln((pressure / reference_pressure).value)
    } else {
        -COEFF*fast_ln((reference_pressure / pressure).value)
    }
}

/// Fast natural log approximation for x >= 1
pub fn fast_ln(x: f32) -> f32 {
    // Decompose float
    let bits = x.to_bits();
    let exp = ((bits >> 23) & 0xff) as i32 - 127;

    // Normalize mantissa to [1,2)
    let mant_bits = (bits & 0x007f_ffff) | 0x3f80_0000;
    let m = f32::from_bits(mant_bits);

    // atanh-based log approximation
    let y = (m - 1.0) / (m + 1.0);
    // let y2 = y * y;

    // Increase accuracy with 2.0 * (y + y^3/3 + y^5/5 + ...)
    let ln_m = 2.0 * y;

    ln_m + (exp as f32) * core::f32::consts::LN_2
}

#[derive(Default)]
struct Timestamps {
    calibration: Option<UtcTime>,
    preflight: Option<UtcTime>,
    flight: Option<UtcTime>,
    landed: Option<UtcTime>,
    recovered: Option<UtcTime>,
}
impl Timestamps {
    pub fn from_bytes(bytes: &[u8;25] ) -> Self {
        let calibration = UtcTime::try_from_bytes(unwrap!(bytes[0..5].try_into()));
        let preflight   = UtcTime::try_from_bytes(unwrap!(bytes[5..10].try_into()));
        let flight      = UtcTime::try_from_bytes(unwrap!(bytes[10..15].try_into()));
        let landed      = UtcTime::try_from_bytes(unwrap!(bytes[15..20].try_into()));
        let recovered   = UtcTime::try_from_bytes(unwrap!(bytes[20..25].try_into()));

        Timestamps { calibration, preflight, flight, landed, recovered } 
    }
    pub fn as_bytes(&self) -> [u8; 25] {
        let mut arr = [0xFFu8; 25];
        if let Some(time) = &self.calibration { arr[0..5].copy_from_slice(&time.as_bytes()); }
        if let Some(time) = &self.preflight   { arr[5..10].copy_from_slice(&time.as_bytes()); }
        if let Some(time) = &self.flight      { arr[10..15].copy_from_slice(&time.as_bytes()); }
        if let Some(time) = &self.landed      { arr[15..20].copy_from_slice(&time.as_bytes()); }
        if let Some(time) = &self.recovered   { arr[20..25].copy_from_slice(&time.as_bytes()); }

        arr
    }
}

#[derive(Default)]
struct SensorData {
    pub time: UtcTime, // 5B
    pub imu: Option<(I16x3, I32x3, i16)>, // 20B
    pub baro: Option<(f32, f32, f32)>, // 12B
    pub battery: Option<u16>, // 2B
    pub gps: Option<GgaMessage>, // 23B
    pub reset: bool, // 0B (bitflag byte only)
    pub transition: Option<(State, State)>, // 1B
}
impl SensorData {
    pub const MAX_SIZE: usize = 5 + 20 + 12 + 2 + 23 + 1; // size_of() includes alignment which we don't care about
    pub fn new(time: UtcTime) -> Self {
        Self { time, ..Default::default() }
    }
    pub fn is_some(&self) -> bool {
        self.imu.is_some() || 
        self.baro.is_some() || 
        self.battery.is_some() || 
        self.gps.is_some() || 
        self.reset || 
        self.transition.is_some()
    }
    pub fn encode(&self) -> ArrayVec<u8, {Self::MAX_SIZE}> {
        let mut vec = ArrayVec::new();

        vec.extend(self.time.as_bytes());

        let data_present_bitflags = 
            u8::from(self.imu.is_some())        << 0 |
            u8::from(self.baro.is_some())       << 1 |
            u8::from(self.battery.is_some())    << 2 |
            u8::from(self.gps.is_some())        << 3 |
            u8::from(self.reset)                << 4 |
            u8::from(self.transition.is_some()) << 5;

        vec.push(data_present_bitflags);

        if let Some((I16x3 { x: acc_x, y: acc_y, z: acc_z }, I32x3 { x: gyro_x, y: gyro_y, z: gyro_z }, temp)) = &self.imu {
            vec.extend(acc_x.to_le_bytes());
            vec.extend(acc_y.to_le_bytes());
            vec.extend(acc_z.to_le_bytes());
            vec.extend(gyro_x.to_le_bytes());
            vec.extend(gyro_y.to_le_bytes());
            vec.extend(gyro_z.to_le_bytes());
            vec.extend(temp.to_le_bytes());
        }
        if let Some((pressure, temperature, altitude)) = &self.baro {
            vec.extend(pressure.to_le_bytes());
            vec.extend(temperature.to_le_bytes());
            vec.extend(altitude.to_le_bytes());
        }
        if let Some(batt_mv) = &self.battery {
            vec.extend(batt_mv.to_le_bytes());
        }
        if let Some(GgaMessage { utc_time, latitude, longitude, fix_type, num_satellites, altitude_msl }) = &self.gps {
            vec.extend(utc_time.as_bytes());
            vec.extend(latitude.degrees.to_le_bytes());
            vec.extend(latitude.degrees_millionths.to_le_bytes());
            vec.extend(longitude.degrees.to_le_bytes());
            vec.extend(longitude.degrees_millionths.to_le_bytes());
            vec.push(*fix_type as u8);
            vec.push(*num_satellites);
            vec.extend(altitude_msl.decimetres.to_le_bytes());
        }
        if let Some((state1, state2)) = self.transition {
            vec.push((state1 as u8) << 4 | (state2 as u8) << 0);
        }

        vec
    }
}

#[interrupt(wake_cpu)] // `wake_cpu` returns the CPU to active mode after servicing interrupt 
fn RTC() {
    critical_section::with(|cs| CURRENT_TIME.borrow_ref_mut(cs).increment());
    unsafe { msp430fr2355::Peripherals::steal().RTC.rtciv.read(); } // Clear interrupt flag
}

static MSG_READY: AtomicBool = AtomicBool::new(false);
#[interrupt]
fn EUSCI_A1() {
    let a1 = unsafe { msp430fr2355::Peripherals::steal().E_USCI_A1 };
    // If we've received a character, append it to the message (provided there isn't already a complete message waiting).
    if a1.uca1iv().read().uciv().is_ucrxifg() {
        let chr = a1.uca1rxbuf().read().ucrxbuf().bits();
        println!("Received GPS character '{}'", chr as char);
        
        if !MSG_READY.load(Ordering::Relaxed) {
            critical_section::with(|cs| GPS_MSG_BUF.borrow_ref_mut(cs).push(chr as char) );
            if chr == b'\n' {
                MSG_READY.store(true, Ordering::Relaxed);
            }
        }
    }
}