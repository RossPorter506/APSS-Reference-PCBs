use msp430fr2x5x_hal::{clock::Smclk, serial::{BitCount, BitOrder, Loopback, Parity, StopBits, Tx}};
use msp430::interrupt::Mutex;
use core::cell::RefCell;

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

    // Move the UART into a global so it can be called anywhere, including in panics.
    msp430::critical_section::with(|cs| {
        crate::serial::SERIAL.replace(cs, Some(debug_uart))
    });
}

// Store our serial handle globally after it's been configured, so we don't have to carry it around with us everywhere.
use crate::pin_mappings::{DebugEusci, DebugTxPin};
/// Used by println macros to print over UART.
pub static SERIAL: Mutex<RefCell<Option< Tx<DebugEusci> >>> = Mutex::new(RefCell::new(None));
