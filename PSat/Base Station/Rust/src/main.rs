#![no_main]
#![no_std]

// External imports
use arrayvec::ArrayString;
use embedded_hal::pwm::SetDutyCycle;
use msp430_rt::entry;

// Internal modules
mod board;
// mod serial;
mod panic_handler;
mod lora;
mod gps;
#[path ="pin_mappings_base_station.rs"]
mod pin_mappings; // Import 'pin_mappings_v2_1' as 'pin_mappings'

// Internal imports
use crate::{
    board::{Board, Mode}, 
    gps::{GgaMessage, GgaParseError, NMEA_MESSAGE_MAX_LEN}
};

/// Identifier unique to each team. A value between 0..=31.
pub const TEAM_ID: u8 = 0;

#[entry]
fn main() -> ! {
    let mut board = board::configure();

    let mut data_recieved: [u8; 255] = [0; 255]; 
    let mut gps_msg_buf = ArrayString::new();
    let mut gps_data: Option<GgaMessage> = None;
    
    loop {
        // board.radio.recieve_start(None);
        // let _ = nb::block!(board.radio.recieve_is_complete(&mut data_recieved));

        // let received_team_id = data_recieved[0] >> 3;
        // match received_team_id {
        //     0x01 => {

        //     }

        //     0x02 => {

        //     }

        //     0x03 => {

        //     }

        //     0x04 => {
                
        //     }

        //     0x05 => {
                
        //     }

        //     0x06 => {
                
        //     }

        //     _ => {}
        // }
        match board.gps.get_gga_message(&mut gps_msg_buf) {
            Ok(msg) => {
                gps_data.replace(msg);
            },
            Err(nb::Error::WouldBlock) => {},
            Err(nb::Error::Other(GgaParseError::NoFix)) => {},
            Err(nb::Error::Other(GgaParseError::SerialError(_))) => {
                gps_msg_buf.clear();
            },
            Err(nb::Error::Other(_)) => {
                todo!()
            }
        }

        }
    }
