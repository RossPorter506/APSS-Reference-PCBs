#![no_main]
#![no_std]

use arrayvec::ArrayString;
use gps::GgaParseError;
// External imports
use msp430_rt::entry;
use msp430fr2x5x_hal::hal::blocking::delay::DelayMs;

// Internal modules
mod pin_mappings { include!("pin_mappings_v2_0.rs"); } // Import 'pin_mappings_v2_0' as 'pin_mappings'
mod board;
mod serial;
mod panic_handler;
mod lora;
mod gps;

// Internal imports
use board::Board;

#[entry]
fn main() -> ! {
    let mut board = board::configure(); // Collect board elements, configure printing, etc.

    // Printing can be expensive in terms of executable size. We only have 32kB on the MSP430, use it sparingly.
    // Prints over eUSCI A0. See board::configure() for details.
    println!("Hello world!");

    let mut buf = ArrayString::new();
    loop {

        match nb::block!(board.gps.get_gga_message(&mut buf)) {
            Ok(results) => {
                println!("Time: {}, Lat: {}, Long: {}, Fix type: {:?}, Num sats: {}, Altitude: {}", 
                    results.utc_time, results.latitude, results.longitude, results.fix_type, results.num_satellites, results.altitude_msl
                );
                board.radio.transmit_start(&[results.num_satellites]).unwrap();
                let _ = nb::block!(board.radio.transmit_is_complete());
            },
            Err(GgaParseError::NoFix) => (),
            Err(GgaParseError::SerialError(_)) => (),
            Err(e) => panic!("{:?}", e)
        }
    }

    idle_loop(board);
}

fn idle_loop(mut board: Board) -> ! {
    loop {
        // Snake the LEDs through the rainbow
        const LED_DELAY_MS: u16 = 50; // ms

        board.gpio.red_led.turn_on();
        board.delay.delay_ms(LED_DELAY_MS);

        board.gpio.green_led.turn_on();
        board.delay.delay_ms(LED_DELAY_MS);

        board.gpio.blue_led.turn_on();
        board.delay.delay_ms(LED_DELAY_MS);

        board.gpio.red_led.turn_off();
        board.delay.delay_ms(LED_DELAY_MS);

        board.gpio.green_led.turn_off();
        board.delay.delay_ms(LED_DELAY_MS);

        board.gpio.blue_led.turn_off();
        board.delay.delay_ms(LED_DELAY_MS);
    }
}
