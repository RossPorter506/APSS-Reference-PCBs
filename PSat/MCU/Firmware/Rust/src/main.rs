#![no_main]
#![no_std]

use arrayvec::ArrayString;
use embedded_hal::delay::DelayNs;
use gps::GgaParseError;
// External imports
use msp430_rt::entry;
use msp430fr2x5x_hal::delay::SysDelay;

// Internal modules
mod pin_mappings { include!("pin_mappings_v2_1.rs"); } // Import 'pin_mappings_v2_1' as 'pin_mappings'
mod board;
mod serial;
mod panic_handler;
mod lora;
mod gps;
mod icm42670;

// Internal imports
use crate::board::Gpio;

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

    idle_loop(system.gpio, system.delay);
}

fn idle_loop(mut gpio: Gpio, mut delay: SysDelay) -> ! {
    loop {
        // Snake the LEDs through the rainbow
        const LED_DELAY_MS: u32 = 50; // ms

        gpio.red_led.turn_on();
        delay.delay_ms(LED_DELAY_MS);

        gpio.green_led.turn_on();
        delay.delay_ms(LED_DELAY_MS);

        gpio.blue_led.turn_on();
        delay.delay_ms(LED_DELAY_MS);

        gpio.red_led.turn_off();
        delay.delay_ms(LED_DELAY_MS);

        gpio.green_led.turn_off();
        delay.delay_ms(LED_DELAY_MS);

        gpio.blue_led.turn_off();
        delay.delay_ms(LED_DELAY_MS);
    }
}
