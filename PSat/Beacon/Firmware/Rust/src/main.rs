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
#[path ="pin_mappings_v2_1.rs"]
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

    // Buffer used to store partial GPS messages
    let mut gps_msg_buf = ArrayString::new();
    // GPS data ready to transmit
    let mut gps_data: Option<GgaMessage> = None;

    let mut prev_mode = board.check_mode();

    state_transition(&mut board, Mode::default());
    loop {
        let mode = board.check_mode();

        if mode != prev_mode {
            state_transition(&mut board, mode);
        }

        state_actions(&mut board, mode, &mut gps_msg_buf, &mut gps_data);
        prev_mode = mode;

        // TODO: Enter LPM until either:
        // BCtrl GPIO changes
        // UART packet arrives (active mode only)
        // Audio pulse timer hits toggle point or max value
    }
}

fn state_transition(board: &mut Board, mode: Mode) {
    if mode.is_idle() {
        board.gps.disable();
    } else {
        board.gps.enable();
    }
    
    if mode.is_auto() {
        board.drive_shared_bus();
    } else {
        board.yield_shared_bus();
    }

    if mode.beeps() {
        board.audio_pulse_timer.resume();
    } else {
        board.audio_pwm.set_duty_cycle_fully_off();
        board.audio_pulse_timer.pause();
    }
}

fn state_actions(board: &mut Board, mode: Mode, gps_msg_buf: &mut ArrayString<NMEA_MESSAGE_MAX_LEN>, gps_data: &mut Option<GgaMessage>) {
    if mode.beeps() {
        board.manage_buzzer();
    }

    if mode.transmits() {
        // Check for a character. If we have a full message, store it
        match board.gps.get_gga_message(gps_msg_buf) {
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

        // Transmit iff we would respect the transmit duty cycle AND we have data.
        if board.radio_delay_timer.wait().is_ok() {
            if let Some(data) = gps_data.take() {
                // let str = data.encode_string();
                // board.radio.transmit_start(str.as_bytes()).unwrap();
                let bytes = data.encode_binary();
                board.radio.transmit_start(bytes.as_slice()).unwrap();
            }
        }
        if board.radio.transmit_is_complete().is_ok() {
            board.radio_delay_timer.start(board::RADIO_DELAY_MAX);
        }
    }
}
