#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

// External imports
use arrayvec::ArrayString;
use embedded_hal::{digital::InputPin, pwm::SetDutyCycle};
use msp430_rt::entry;
use msp430fr2355::interrupt;

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

        if mode.is_idle() {
            msp430fr2x5x_hal::lpm::request_lpm4();
        } else {
            msp430fr2x5x_hal::lpm::enter_lpm0();
        }
        
        // Awoken from LPM by any of:
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
        board.audio_pulse_timer.enable_interrupts();
        board.audio_pulse_subtimer.enable_interrupts();
    } else {
        board.audio_pwm.set_duty_cycle_fully_off();
        board.audio_pulse_timer.pause();
        board.audio_pulse_timer.disable_interrupts();
        board.audio_pulse_subtimer.disable_interrupts();
    }

    if mode.transmits() {
        board.gps.enable_rx_interrupt();
    } else {
        board.gps.disable_rx_interrupt();
    }

    match board.bctrl0.is_high().unwrap() {
        true  => board.bctrl0.select_falling_edge_trigger(),
        false => board.bctrl0.select_rising_edge_trigger(),
    };
    match board.bctrl1.is_high().unwrap() {
        true  => board.bctrl1.select_falling_edge_trigger(),
        false => board.bctrl1.select_rising_edge_trigger(),
    };
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

// Lazily coded interrupts. I should make proper static vars for the interrupt vectors and such, but we're just
// using interrupts to wake the CPU from LPM, so we just clear interrupt flags and do everything in main.
#[interrupt(wake_cpu)]
fn PORT4() {
    unsafe {
        msp430fr2355::Peripherals::steal().P4.p4iv.read();
    }
}

// TB0 CCR0 interrupt
#[interrupt(wake_cpu)]
fn TIMER0_B0() {
    // Flag automatically reset
}

// TB0 CCRx interrupts, x > 0
#[interrupt(wake_cpu)]
fn TIMER0_B1() {
    unsafe {
        msp430fr2355::Peripherals::steal().TB0.tb0iv.read();
    }
}

// GPS (Rx) interrupts
#[interrupt(wake_cpu)]
fn EUSCI_A0() {
    unsafe {
        msp430fr2355::Peripherals::steal().E_USCI_A0.uca0iv().read();
    }
}
