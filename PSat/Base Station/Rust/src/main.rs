#![no_main]
#![no_std]

use arrayvec::ArrayString;
// External imports
use msp430_rt::entry;
use ufmt::{uDisplay, uwrite};
use embedded_lora_rfm95::rfm95::RFM95_FIFO_SIZE;
use nb::Error::{Other, WouldBlock};
use crate::gps::{DecimalDegrees, Decimetres, GgaMessage, GgaParseError, GpsFixType, UtcTime};

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
    let mut gps_buffer = ArrayString::new();

    board.radio.recieve_start(None);
    loop {
        // Depending on how much LoRa traffic we receive, we may get starved for GPS messages:
        // Once per second, the GPS sends a byte every ~80us until it's sent ~82 bytes (~7ms total), then nothing for the rest of the second.
        // Printing the radio messages takes longer than 80us, so if we're halfway through a GPS message at the time, then the serial buffer will overrun while we're printing.
        // I doubt this will be an issue in practise, but if it is a problem then receiving the GPS message could instead be done through an interrupt.

        match board.gps.get_gga_message(&mut gps_buffer) {
            Ok(msg) => {
                let GgaMessage { utc_time, latitude, longitude, fix_type, altitude_msl, num_satellites } = msg;
                println!("$B{};{};{};{};{};{}", fix_type, utc_time, latitude, longitude, altitude_msl, num_satellites);
            },
            Err(WouldBlock | Other(GgaParseError::NoFix)) => (),
            Err(_) => gps_buffer.clear(), // Clear buffer and try again
        }
        match board.radio.recieve_is_complete(&mut radio_buffer) {
            Ok(bytes)       => print_bytes(bytes),
            Err(WouldBlock) => continue,
            Err(Other(_))   => (),
        };
        board.radio.recieve_start(None)
    }
}

fn print_bytes(bytes: &[u8]) {
    let maybe_arr = <[u8; 14]>::try_from(bytes);
    match maybe_arr {
        Ok(arr) => { // Treat 14 byte strings as GPS data. In future we should probably have a packet type field at the start.
            let(team_id, data) = decode_binary(arr);
            let GgaMessage { utc_time, latitude, longitude, fix_type, altitude_msl, num_satellites } = data;
            println!("$G{};{};{};{};{};{};{}", team_id, fix_type, utc_time, latitude, longitude, altitude_msl, num_satellites);
        },
        Err(_) => { // Some other string of bytes
            println!("$?{};{}", bytes.len(), SliceAsHex(bytes));
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

/// Decodes GPS packet data from a bitpacked binary representation. Used by the base station. 
/// Returns the ID of the team mentioned in the packet.
fn decode_binary(bytes: [u8; 14]) -> (u8, GgaMessage) {
    let team_id = bytes[0] >> 3;

    let hours = ((bytes[0] & 0b111) << 2) | (bytes[1] >> 6);
    let minutes = bytes[1] & 0b00111111;
    let seconds = bytes[2] >> 2;
    let utc_time = UtcTime {hours, minutes, seconds};
    
    let fix_ok = (bytes[2] & 0b10) > 0;
    let dif_fix = (bytes[2] & 0b1) > 0;
    let fix_type = match (fix_ok, dif_fix) {
        (false, _)    => GpsFixType::NoFix,
        (true, false) => GpsFixType::Gps,
        (true, true)  => GpsFixType::DifferentialGps,
    };
    
    let num_satellites: u8 = bytes[3] >> 4;
    
    let altitude_upper_bits: i32 = if (bytes[3] & 0b1000) > 0 {0xFFF00000_u32 as i32} else {0}; // Sign-extend altitude if necessary
    let altitude: i32 = altitude_upper_bits | (bytes[3] as i32 & 0x0F) << 16 | (bytes[4] as i32) << 8 | bytes[5] as i32;
    
    let latitude  = decode_degrees_1e7([bytes[6], bytes[7], bytes[8], bytes[9]]);
    let longitude = decode_degrees_1e7([bytes[10], bytes[11], bytes[12], bytes[13]]);
    
    let data = GgaMessage {utc_time, latitude, longitude, fix_type, altitude_msl: Decimetres::new(altitude), num_satellites};
    (team_id, data)
}

/// Takes an integer value representing a latitude/longitude multiplied by 1e7 and returns a DecimalDegrees.
fn decode_degrees_1e7(bytes: [u8;4]) -> DecimalDegrees {
    let degrees_1e7: i32 = (bytes[0] as i32) << 24 | (bytes[1] as i32) << 16 | (bytes[2] as i32) << 8 | bytes[3] as i32;
    
    let degrees: i16 = (degrees_1e7 / 10_000_000) as i16;
    let millionths: u32 = ((degrees_1e7 - (degrees as i32 * 10_000_000)) / 10).unsigned_abs();

    DecimalDegrees{ degrees, millionths }
}
