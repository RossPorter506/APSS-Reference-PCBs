#![no_main]
#![no_std]

// External imports
use msp430_rt::entry;
use ufmt::{uDisplay, uwrite};
use embedded_lora_rfm95::rfm95::RFM95_FIFO_SIZE;

// Internal modules
mod board;
mod serial;
mod panic_handler;
mod lora;
mod gps;
#[path ="pin_mappings_v1.rs"]
mod pin_mappings; // Import 'pin_mappings_base_station' as 'pin_mappings'

const TEAM_ID: u8 = 31;

#[entry]
fn main() -> ! {
    let mut board = board::configure();

    let mut radio_buffer: [u8; RFM95_FIFO_SIZE] = [0; 255];

    board.radio.recieve_start(None);
    loop {
        match nb::block!(board.radio.recieve_is_complete(&mut radio_buffer)) {
            Err(_)    => (),
            Ok(bytes) => print_bytes(bytes),
        };
        board.radio.recieve_start(None)
    }
}

fn print_bytes(bytes: &[u8]) {
    let maybe_arr = <[u8; 14]>::try_from(bytes);
    match maybe_arr {
        Ok(arr) => { // 14 byte strings are GPS data. In future we should probably have a packet type field at the start.
            let(team_id, data) = decode_binary(arr);
            let GpsData { time, latitude, longitude, fix_type, altitude, num_sats } = data;
            println!("G{};{};{};{};{};{};{}", team_id, time, latitude, longitude, fix_type, altitude, num_sats);
        },
        Err(_) => { // Some other string of bytes
            println!("?{};{}", bytes.len(), SliceAsHex(bytes));
        },
    }
}

/// Adapter that allows printing a slice of bytes as semicolon-separated hex characters. 
struct SliceAsHex<'a>(&'a [u8]);
impl uDisplay for SliceAsHex<'_> { // "FF;A2;C3;03"
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where W: ufmt::uWrite + ?Sized {
        if let Some((first, others)) = self.0.split_first() {
            uwrite!(f, "{:02X}", *first)?;
            for byte in others {
                uwrite!(f, ";{:02X}", *byte)?;
            }
        }
        Ok(())
    }
}

struct UtcTime {
    hours: u8,
    minutes: u8,
    seconds: u8,
}
impl uDisplay for UtcTime { // "XX:XX:XX UTC"
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where W: ufmt::uWrite + ?Sized {
        for (i, &val) in [self.hours, self.minutes, self.seconds].iter().enumerate() {
            if val < 10 {
                uwrite!(f, "0{}", val)?;
            } else {
                uwrite!(f, "{}", val)?; 
            }
            if i != 2 {
                uwrite!(f, ":")?;
            }
        }
        uwrite!(f, " UTC")?;
        Ok(())
    }
}

struct DecimalDegrees {
    degrees: i16,
    millionths: u32,
}
impl uDisplay for DecimalDegrees { // "52.001234"
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where W: ufmt::uWrite + ?Sized {
        uwrite!(f, "{}.", self.degrees)?;

        let num_leading_zeroes = self.millionths.checked_ilog10().unwrap_or(6) as u8;
        for _ in 0..num_leading_zeroes {
            uwrite!(f, "0")?;
        }

        uwrite!(f, "{}", self.millionths)?;
        Ok(())
    }
}

enum GpsFixType {
    NoFix = 0,
    Gps = 1,
    Dgps = 2
}
impl uDisplay for GpsFixType { // 'n', 'g', or 'd'
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where W: ufmt::uWrite + ?Sized {
        let c = match self {
            GpsFixType::NoFix => 'n',
            GpsFixType::Gps   => 'g',
            GpsFixType::Dgps  => 'd',
        };
        uwrite!(f, "{}", c)?;
        Ok(())
    }
}

struct GpsData {
    time: UtcTime,
    latitude: DecimalDegrees,
    longitude: DecimalDegrees,
    fix_type: GpsFixType,
    altitude: i32,
    num_sats: u8,
}

/// Decodes GPS packet data from a bitpacked binary representation. Used by the base station. 
/// Returns the ID of the team mentioned in the packet.
fn decode_binary(bytes: [u8; 14]) -> (u8, GpsData) {
    let team_id = bytes[0] >> 3;

    let hours = ((bytes[0] & 0b111) << 2) | (bytes[1] >> 6);
    let minutes = bytes[1] & 0b00111111;
    let seconds = bytes[2] >> 2;
    let time = UtcTime {hours, minutes, seconds};
    
    let fix_ok = (bytes[2] & 0b10) > 0;
    let dif_fix = (bytes[2] & 0b1) > 0;
    let fix_type = match (fix_ok, dif_fix) {
        (false, _)    => GpsFixType::NoFix,
        (true, false) => GpsFixType::Gps,
        (true, true)  => GpsFixType::Dgps,
    };
    
    let num_sats: u8 = bytes[3] >> 4;
    
    let altitude_upper_bits: i32 = if (bytes[3] & 0b1000) > 0 {0xFFF00000_u32 as i32} else {0}; // Sign-extend altitude if necessary
    let altitude: i32 = altitude_upper_bits | (bytes[3] as i32 & 0x0F) << 16 | (bytes[4] as i32) << 8 | bytes[5] as i32;
    
    let latitude  = decode_degrees_1e7([bytes[6], bytes[7], bytes[8], bytes[9]]);
    let longitude = decode_degrees_1e7([bytes[10], bytes[11], bytes[12], bytes[13]]);
    
    let data = GpsData {time, latitude, longitude, fix_type, altitude, num_sats};
    (team_id, data)
}

/// Takes an integer value representing a latitude/longitude multiplied by 1e7 and returns a DecimalDegrees.
fn decode_degrees_1e7(bytes: [u8;4]) -> DecimalDegrees {
    let degrees_1e7: i32 = (bytes[0] as i32) << 24 | (bytes[1] as i32) << 16 | (bytes[2] as i32) << 8 | bytes[3] as i32;
    
    let degrees: i16 = (degrees_1e7 / 10_000_000) as i16;
    let millionths: u32 = ((degrees_1e7 - (degrees as i32 * 10_000_000)) / 10).unsigned_abs();

    DecimalDegrees{ degrees, millionths }
}
