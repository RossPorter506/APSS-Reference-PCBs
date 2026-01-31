use msp430fr2x5x_hal::{clock::Smclk, serial::{BitCount, BitOrder, Loopback, Parity, StopBits, Tx}};

/// Configure the debug UART for use with println!().
pub fn configure_debug_serial(pin: DebugTxPin, smclk: &Smclk, debug_eusci: DebugEusci) {
    pub const DEBUG_SERIAL_BAUD: u32 = 115200;
    let debug_uart = msp430fr2x5x_hal::serial::SerialConfig::new(debug_eusci, 
        BitOrder::LsbFirst, 
        BitCount::EightBits, 
        StopBits::OneStopBit, 
        Parity::NoParity, 
        Loopback::NoLoop, 
        DEBUG_SERIAL_BAUD)
        .use_smclk(smclk)
        .tx_only(pin);

    let debug_uart = crate::serial::PrintableSerial(debug_uart);

    // Move the UART into a global so it can be called anywhere, including in panics.
    msp430::critical_section::with(|cs| {
        crate::serial::SERIAL.replace(cs, Some(debug_uart))
    });
}

// A little bit of magic to get println working.
// Tx can only print bytes by default, but by implementing this we can print arbitrary (ASCII) strings.
// Format strings are automatically implemented for implementers of core::fmt::Write, so
// with the custom println!() macro below this means we get full println! behaviour.
pub struct PrintableSerial( pub Tx<DebugEusci> );
impl core::fmt::Write for PrintableSerial {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for char in s.chars() {
            use embedded_hal_nb::serial::Write;
            nb::block!( self.0.write(char as u8) ); // The cast to u8 assumes ASCII-only characters
        }
        Ok(())
    }
}

// Store our serial handle globally after it's been configured, so we don't have to carry it around with us everywhere.
use msp430::interrupt::Mutex;
use core::cell::RefCell;

use crate::pin_mappings::{DebugEusci, DebugTxPin};
/// Used by println macros to print over UART.
pub static SERIAL: Mutex<RefCell<Option< PrintableSerial >>> = Mutex::new(RefCell::new(None));

// Make a macro equivalent to the regular println!() macro.
/// Prints over `eUSCI_A0` serial. Panics if `board::configure()` hasn't been called yet.
#[macro_export]
macro_rules! print {
    ($first:tt $(, $( $rest:tt )* )?) => {
        {
            msp430::critical_section::with(|cs| {
                use ufmt::uwrite;
                match *$crate::serial::SERIAL.borrow_ref_mut(cs) {
                    Some(ref mut serial) => uwrite!(ufmt_utils::WriteAdapter(serial), $first,  $( $($rest)* )*).ok(),
                    None => panic!("Printing without serial configuration"),
                }
            });
        }
    };
}

/// Prints over `eUSCI_A0` serial. Fails to print if `board::configure()` hasn't been called yet.
#[macro_export]
macro_rules! println {
    ($first:tt $(, $( $rest:tt )* )?) => {
        {
            $crate::print!($first,  $( $($rest)* )*);
            $crate::print!("\n");
        }
    };
}

// Make a macro equivalent to the regular println!() macro.
/// Prints over `eUSCI_A0` serial. Panics if `board::configure()` hasn't been called yet.
/// 
/// This macro uses the Rust core library, which can print more things but bloats the executable. Avoid using this if you can.
#[macro_export]
macro_rules! stdlib_print {
    ($first:tt $(, $( $rest:tt )* )?) => {
        {
            msp430::critical_section::with(|cs| {
                use core::fmt::Write;
                match *$crate::serial::SERIAL.borrow_ref_mut(cs) {
                    Some(ref mut serial) => write!(serial, $first,  $( $($rest)* )*).ok(),
                    None => panic!("Printing without serial configuration"),
                };
            });
        }
    };
}

/// Prints over `eUSCI_A0` serial. Fails to print if `board::configure()` hasn't been called yet.
/// 
/// This macro uses the Rust core library, which can print more things but bloats the executable. Avoid using this if you can.
#[macro_export]
macro_rules! stdlib_println {
    ($first:tt $(, $( $rest:tt )* )?) => {
        {
            $crate::stdlib_print!($first,  $( $($rest)* )*);
            $crate::stdlib_print!("\n");
        }
    };
}