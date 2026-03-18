
use core::{cell::UnsafeCell, sync::atomic::Ordering};

use defmt::{Logger, Encoder, global_logger};
use embedded_io::Write;
use msp430::interrupt::{enable as enable_interrupts, disable as disable_interrupts};
use portable_atomic::AtomicBool;

#[global_logger]
struct UartLogger;

static LOGGER_STATE: LoggerState = LoggerState::new();

unsafe impl Sync for LoggerState {}

struct LoggerState {
    taken: AtomicBool,
    encoder: UnsafeCell<Encoder>,
}
impl LoggerState {
    pub const fn new() -> Self {
        Self { encoder: UnsafeCell::new(Encoder::new()), taken: AtomicBool::new(false) }
    }
}

unsafe impl Logger for UartLogger {
    fn acquire() {
        disable_interrupts();
        if LOGGER_STATE.taken.swap(true, Ordering::Acquire) {
            panic!("Logger taken re-entrantly");
        }
        unsafe { 
            (*LOGGER_STATE.encoder.get()).start_frame(write_bytes);
        };
    }

    unsafe fn release() {
        unsafe { 
            if !LOGGER_STATE.taken.load(Ordering::Relaxed) {
                panic!("Logger release out of context");
            }
            (*LOGGER_STATE.encoder.get()).end_frame(write_bytes);
            LOGGER_STATE.taken.store(false, Ordering::Release);
            enable_interrupts();
        };
    }

    unsafe fn write(bytes: &[u8]) {
        if !LOGGER_STATE.taken.load(Ordering::Relaxed) {
            panic!("Logger write out of context");
        }
        unsafe{ (*LOGGER_STATE.encoder.get()).write(bytes, write_bytes) };
    }

    unsafe fn flush() {}
}

fn write_bytes(bytes: &[u8]) {
    msp430::critical_section::with(|cs| {
        if let Some(ref mut tx) = *crate::serial::SERIAL.borrow_ref_mut(cs) {
            tx.write_all(bytes);
            tx.flush();
        }
    });
}
