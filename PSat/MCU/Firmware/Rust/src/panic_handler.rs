use msp430fr2x5x_hal::gpio::*;
use static_assertions::assert_type_eq_all;
use crate::{uprint, uprintln};

// Our panic handler. Currently we print strings here for maximum debuggability. String printing is quite expensive in terms of executable size,
// so if you're running out of space consider commenting out some of these print statements (or uncommenting `strip = true` in cargo.toml!).
use crate::{pin_mappings::{RedLedPin, GreenLedPin, BlueLedPin}};
use core::panic::PanicInfo;
#[panic_handler]
fn panic_handler(panic_info: &PanicInfo) -> ! {
    msp430::interrupt::disable();

    // Turn on red LED, turn off others
    // The following code relies on these assertions. If these pins are changed you will also have to change the code below.
    assert_type_eq_all!(RedLedPin, Pin<P2, Pin0, Output>);
    assert_type_eq_all!(BlueLedPin, Pin<P2, Pin1, Output>);
    assert_type_eq_all!(GreenLedPin, Pin<P2, Pin2, Output>);
    
    unsafe { 
        let p2 = msp430fr2355::Peripherals::steal().P2;
        p2.p2dir.set_bits(|w| w.bits(Pin0::SET_MASK | Pin1::SET_MASK | Pin2::SET_MASK));
        p2.p2out.set_bits(|w| w.bits(Pin1::SET_MASK | Pin2::SET_MASK));
        p2.p2out.clear_bits(|w| w.bits(Pin0::CLR_MASK));
    }

    loop { 
        msp430::asm::barrier();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[unsafe(no_mangle)]
extern "C" fn abort() -> ! {
    panic!();
}
