#![no_main]
#![no_std]
#![feature(asm_experimental_arch)]
#![feature(abi_msp430_interrupt)]

// External imports
use arrayvec::ArrayVec;
use embedded_hal::digital::InputPin;
use embedded_storage::nor_flash::NorFlash;
use ::icm42670::accelerometer::vector::I16x3;
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

// Internal imports
use crate::{board::{BeaconMode, McuBoard}, gps::{Altitude, Degrees, UtcTime}};

// How often the main loop cycles through. If this is set faster than the main loop takes, then timekeeping will fail.
// Currently this period has to cleanly divide into 1000 due to `UtcTime::increment()`
const MAIN_LOOP_PERIOD_MS: u16 = 10;
const MAIN_LOOP_FREQ_HZ: u16 = 1000 / MAIN_LOOP_PERIOD_MS;

// TODO: Make UtcTime::increment work for periods that don't evenly divide into 1000
// TODO: RTC interrupt not triggering
// TODO: Readback from flash memory not working
// TODO: Calibrate IMU data for accel flight condition
#[entry]
fn main() -> ! {
    let regs = msp430fr2355::Peripherals::take().unwrap();
    
    // Configure the MCU board assuming no connections to other PCBs.
    let mut system = board::standalone(regs); // Collect board elements, configure printing, etc.
    // Or if the Beacon board is attached:
    // let mut system = board::in_stack(regs);

    // Printing can be expensive in terms of executable size. We only have 32kB on the MSP430, use it sparingly.
    // Prints over eUSCI A0. See board::configure() for details.
    println!("Hello world!");

    #[cfg(feature = "fresh_start")]
    system.nvmem.erase_all();

    // In case we just reset mid-flight, restore state, current time, any saved timestamps, etc.
    let mut state        = system.nvmem.try_get_state().unwrap_or(State::Idle);
    let mut current_time = system.nvmem.try_get_current_time().unwrap_or_default();
    let mut timestamps   = system.nvmem.get_transitions();
    let mut write_addr   = system.nvmem.get_flash_write_addr();
    let mut ref_pressure = system.nvmem.try_get_calibration_pressure().unwrap_or(Pressure::new::<pascal>(101_325.0));

    println!("Initial state: {:?}", state);

    let mut data = SensorData::new(current_time.clone());
    let mut is_calibrated = state > State::Calibration;

    system.nvmem.increment_resets();
    data.reset = true;
    
    loop {
        state_actions(state, &mut system, &mut data, &mut write_addr, &mut is_calibrated, &mut ref_pressure, &current_time);

        let new_state = update_state(state, &mut system, &mut timestamps, &mut data, &current_time, is_calibrated);
        state_transition_actions(new_state, state, &mut system, &mut timestamps);
        state = new_state;

        system.gpio.blue_led.turn_on(); // Blue LED pin serves as a counter to show how often the MCU is idle
        nb::block!(system.rtc.wait());
        // enter_lpm0(); // Sleep until RTC wakes us up in MAIN_LOOP_PERIOD_MS.

        system.gpio.blue_led.turn_off();
        current_time.increment();
        system.nvmem.store_current_time(&current_time);
        data = SensorData::new(current_time.clone());
    }
}

/// Process (Moore-style) state-dependent actions
fn state_actions(state: State, system: &mut McuBoard, data: &mut SensorData, write_addr: &mut u32, is_calibrated: &mut bool, ref_pressure: &mut Pressure, current_time: &UtcTime) {
    match state {
        State::Calibration => {
            if let Ok(p) = system.barometer.pressure() {
                *ref_pressure = p;
                system.nvmem.store_calibration_pressure(p.get::<pascal>());
                system.nvmem.store_num_resets(0); // Reset count
                *is_calibrated = true;
            }
        },
        State::Preflight | State::Flight => {
            // println!("Reading sensors");
            read_sensors(data, system, current_time, ref_pressure);
            // println!("Reading sensors done");
            if data.is_some() {
                // println!("Writing to flash");
                *write_addr = write_to_flash_mem(system, data.encode(), *write_addr);
                // println!("Writing to flash done");
                system.nvmem.store_flash_write_addr(*write_addr);
            }
        },
        _ => (),
    };
}

/// Process actions that should occur once when transitioning between states (Mealy-style)
fn state_transition_actions(state:State, prev_state: State, system: &mut McuBoard, timestamps: &mut Timestamps) {
    if prev_state != state {
        match state {
            State::Idle | State::Recovered => system.beacon_mode(BeaconMode::AutoSleep),
            _ => system.beacon_mode(BeaconMode::AutoActive),
        };
        println!("New state: {:?}", state);
        system.nvmem.store_state(state);
        system.nvmem.store_transitions(timestamps);
    }
}

fn write_to_flash_mem(system: &mut McuBoard, data: ArrayVec<u8, 51>, write_addr: u32) -> u32 {
    let capacity = board::FlashMem::CAPACITY;
    let data = data.as_slice();
    if write_addr + data.len() as u32 <= capacity {
        system.flash_mem.write(write_addr, data).unwrap(); // TODO: unwrap
        // println!("Write: {:?}", data);
        let mut read_buf = [0u8; 51];
        let read_buff = &mut read_buf[0..data.len()];
        system.flash_mem.read(write_addr, read_buff).unwrap();
        // println!("Read back: {:?}", read_buff);
        write_addr + data.len() as u32
    } else { // write wraps around address space
        let remaining = (capacity - write_addr) as usize;
        let (data1, data2) = data.split_at(remaining);
        system.flash_mem.write(write_addr, data1).unwrap(); // TODO: unwrap
        system.flash_mem.write(0, data2).unwrap(); // TODO: unwrap
        data2.len() as u32
    }
}

#[derive(Default)]
struct Timestamps {
    preflight: Option<UtcTime>,
    flight: Option<UtcTime>,
    landed: Option<UtcTime>,
    recovered: Option<UtcTime>,
}
impl Timestamps {
    pub fn from_bytes(bytes: &[u8;20] ) -> Self {
        let preflight = UtcTime::try_from_bytes(bytes[0..5].try_into().unwrap());
        let flight    = UtcTime::try_from_bytes(bytes[5..10].try_into().unwrap());
        let landed    = UtcTime::try_from_bytes(bytes[10..15].try_into().unwrap());
        let recovered = UtcTime::try_from_bytes(bytes[15..20].try_into().unwrap());

        Timestamps { preflight, flight, landed, recovered } 
    }
    pub fn as_bytes(&self) -> [u8; 20] {
        let mut arr = [0xFFu8; 20];
        for (i, opt) in [&self.preflight, &self.flight, &self.landed, &self.recovered].iter().enumerate() {
            if let Some(time) = opt {
                arr[5*i..5*i+5].copy_from_slice(&time.as_bytes());
            }
        }
        arr
    }
}

#[derive(PartialEq, Eq, PartialOrd, Ord, Copy, Clone, uDebug)]
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

fn read_sensors(data: &mut SensorData, system: &mut McuBoard, time: &UtcTime, ref_pressure: &Pressure) {
    const IMU_POLL_FREQ_HZ:    u16 = MAIN_LOOP_FREQ_HZ * 1; // poll every loop
    const BARO_POLL_FREQ_HZ:   u16 = MAIN_LOOP_FREQ_HZ * 1;
    const BATT_POLL_FREQ_HZ:   u16 = MAIN_LOOP_FREQ_HZ / 10; // poll once every 10 loops

    const IMU_POLL_PERIOD_MS:  u16 = 1000 / IMU_POLL_FREQ_HZ;
    const BARO_POLL_PERIOD_MS: u16 = 1000 / BARO_POLL_FREQ_HZ;
    const BATT_POLL_PERIOD_MS: u16 = 1000 / BATT_POLL_FREQ_HZ;

    if time.millis.is_multiple_of(IMU_POLL_PERIOD_MS) {
        data.imu = system.imu.measure_raw().ok();
        // if let Some((I16x3 { x, y, z }, I16x3 { x: x2, y: y2, z: z2 }, temp)) = data.imu {
        //     println!("IMU: {:?},{:?},{:?} {:?},{:?},{:?}, {:?}", x,y,z, x2,y2,z2, temp);
        // }
        
    }
    if time.millis.is_multiple_of(BARO_POLL_PERIOD_MS) {
        if let Ok((temperature, pressure)) = system.barometer.temperature_pressure() {
            let altitude = fast_altitude(pressure, *ref_pressure);
            data.baro = Some((pressure.get::<pascal>(), temperature.get::<degree_celsius>(), altitude));
        }
    }
    if time.millis.is_multiple_of(BATT_POLL_PERIOD_MS) {
        data.battery = Some(system.battery_voltage_mv());
    }
}

pub fn fast_altitude(pressure: Pressure, reference_pressure: Pressure) -> f32 {
    8434.0 * fast_ln((reference_pressure / pressure).value)
}

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

    let ln_m = 2.0 * (y);

    ln_m + (exp as f32) * core::f32::consts::LN_2
}

fn update_state(state: State, system: &mut McuBoard, timestamps: &mut Timestamps, data: &mut SensorData, current_time: &UtcTime, calibrated: bool) -> State {
    use State::*;

    // Short circuit to Recovered if disarmed
    if state != Landed && state != Recovered && is_disarmed(system) {
        data.transition = Some((state, Recovered));
        return Recovered;
    }

    match state {
        Idle => {
            if is_armed(system) {
                data.transition = Some((Idle, Calibration));
                Calibration
            } else {
                Idle
            }
        },
        Calibration => {
            if calibrated {
                timestamps.preflight = Some(current_time.clone());
                data.transition = Some((Calibration, Preflight));
                Preflight
            } else {
                Calibration
            }
        },
        Preflight => {
            let above_20m = data.baro.unwrap_or_default().2 > 20.0;
            let above_2gs = data.imu.unwrap_or_default().0.z > 2_000;

            if above_20m || above_2gs {
                timestamps.flight = Some(current_time.clone());
                data.transition = Some((Preflight, Flight));
                Flight
            } else {
                Preflight
            }
        },
        Flight => {
            let below_20m = if let Some(data) = data.baro {data.2 < 20.0} else { false };

            // TODO: Less than 1 deg/sec in all directions
            let not_rotating = if let Some(imu) = data.imu { imu.1.x < 50 && imu.1.y < 50 && imu.1.z < 50 } else { false }; 
            
            // TODO: Calibrate IMU values so we actually have milli-gees 
            // Check if sqrt(x^2 + y^2 + z^2) = 9.8 +- 0.1g
            let not_accelerating = if let Some((acc, ..)) = data.imu { 
                let x = (acc.x as i32) * 1000 / 2_048;
                let y = (acc.y as i32) * 1000 / 2_048;
                let z = (acc.z as i32) * 1000 / 2_048;
                (x*x + y*y + z*z).abs_diff(9_800*9_800) < 100 
            } else { false }; 

            const TIMEOUT_DURATION_SEC: i32 = 60*10;
            let timeout = if let Some(flight_start) = &timestamps.flight {
                current_time.seconds_since(flight_start) > TIMEOUT_DURATION_SEC
            } else { false };

            if below_20m && not_rotating || timeout {
                timestamps.landed = Some(current_time.clone());
                data.transition = Some((Flight, Landed));
                Landed
            } else {
                Flight
            }
        },
        Landed => {
            if is_disarmed(system) {
                timestamps.recovered = Some(current_time.clone());
                data.transition = Some((Landed, Recovered));
                Recovered
            } else {
                Landed
            }
        },
        Recovered => Recovered,
    }
}

fn is_armed(system: &mut McuBoard) -> bool {
    system.gpio.arm_pin.is_low().unwrap() // Infallible unwrap
}

fn is_disarmed(system: &mut McuBoard) -> bool {
    system.gpio.disarm_pin.is_high().unwrap() // Infallible unwrap
}

#[derive(Default)]
struct SensorData {
    pub time: UtcTime,
    pub imu: Option<(I16x3, I16x3, i16)>, // 14B
    pub baro: Option<(f32, f32, f32)>, // 12B
    pub battery: Option<u16>, // 2B
    pub gps: Option<(Degrees, Degrees, Altitude)>, // 16B
    pub reset: bool, // 0B (bitflag byte only)
    pub transition: Option<(State, State)>, // 1B
}
impl SensorData {
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
    pub fn encode(&self) -> ArrayVec<u8, 51> {
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

        if let Some((I16x3 { x: acc_x, y: acc_y, z: acc_z }, I16x3 { x: gyro_x, y: gyro_y, z: gyro_z }, temp)) = &self.imu {
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
        if let Some((latitude, longitude, altitude)) = &self.gps {
            vec.extend(latitude.degrees.to_le_bytes());
            vec.extend(latitude.degrees_millionths.to_le_bytes());
            vec.extend(longitude.degrees.to_le_bytes());
            vec.extend(longitude.degrees_millionths.to_le_bytes());
            vec.extend(altitude.decimetres.to_le_bytes());
        }
        if let Some((state1, state2)) = self.transition {
            vec.push((state1 as u8) << 4 | (state2 as u8) << 0);
        }

        vec
    }
}

#[interrupt(wake_cpu)] // `wake_cpu` returns the CPU to active mode after servicing interrupt 
fn RTC() {
    println!("RTC int");
    unsafe { msp430fr2355::Peripherals::steal().RTC.rtciv.read(); } // Clear interrupt flag
}
