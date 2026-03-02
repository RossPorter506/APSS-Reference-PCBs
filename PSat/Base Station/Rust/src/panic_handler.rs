// Our panic handler. Rapidly blinks the red LED if something's gone wrong.
use core::panic::PanicInfo;

use msp430::asm::nop;
#[panic_handler]
fn panic_handler(_panic_info: &PanicInfo) -> ! {
    msp430::interrupt::disable();

    let regs = unsafe { msp430fr2355::Peripherals::steal() };

    unsafe { regs.P2.p2dir.set_bits(|w| w.bits(1 << 2 | 1 << 1 | 1 << 0)) }; // Set LEDs (P2.2, 2.1, 2.0) to output
    unsafe{ regs.P2.p2out.set_bits(|w| w.bits(1 << 2 | 1 << 1)) }; // Set green, blue off
    
    loop { 
        for _ in 0..50_000 {
            nop();
        }
        unsafe{ regs.P2.p2out.toggle_bits(|w| w.bits(1)) }; // Toggle red
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
