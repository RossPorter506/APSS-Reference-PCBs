#![allow(dead_code)]
use core::time::Duration;

use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_lora_rfm95::{error::{IoError, RxCompleteError, TxStartError}, lora::types::{Bandwidth, CodingRate, CrcMode, HeaderMode, Polarity, PreambleLength, SpreadingFactor, SyncWord}, rfm95::{self, Rfm95Driver}};
use msp430fr2x5x_hal::{delay::SysDelay, spi::Spi};
use nb::Error::{WouldBlock, Other};
use crate::pin_mappings::{RadioCsPin, RadioEusci, RadioResetPin};

const LORA_FREQ_HZ: u32 = 915_000_000;
pub use rfm95::RFM95_FIFO_SIZE;

pub fn new(spi: Spi<RadioEusci>, cs_pin: RadioCsPin, reset_pin: RadioResetPin, delay: SysDelay) -> Radio {
    let spi_device = ExclusiveDevice::new(spi, cs_pin, delay).unwrap();
    let mut rfm95 = match Rfm95Driver::new(spi_device, reset_pin, delay) {
        Ok(rfm) => rfm,
        Err(_e) => panic!("Radio reports invalid silicon revision. Is the beacon connected?"),
    };
    
    // 125kHz bandwidth, 4/5 coding rate, SF8 gives a bitrate of about 3000bps.
    let lora_config = embedded_lora_rfm95::lora::config::Builder::builder()
        .set_bandwidth(Bandwidth::B125) // lower bandwidth == longer range, but very low bandwidths can suffer from clock source tolerance issues
        .set_coding_rate(CodingRate::C4_5) // Error correction lowers bitrate. Consider how electronically noisy the area might be.
        .set_crc_mode(CrcMode::Enabled)
        .set_frequency(LORA_FREQ_HZ.into())
        .set_header_mode(HeaderMode::Explicit)
        .set_polarity(Polarity::Normal)
        .set_preamble_length(PreambleLength::L8)
        .set_spreading_factor(SpreadingFactor::S8) // High SF == Best range
        .set_sync_word(SyncWord::PRIVATE);
    rfm95.set_config(&lora_config).unwrap();

    Radio{driver: rfm95}
}

type SPIDevice = ExclusiveDevice<Spi<RadioEusci>, RadioCsPin, SysDelay>;
type RFM95 = Rfm95Driver<SPIDevice>;
/// Top-level interface for the radio module.
pub struct Radio {
    pub driver: RFM95,
}
impl Radio {
    /// Begin transmission and return immediately. Check whether the transmission is complete by calling `transmit_is_complete()`.
    pub fn transmit_start(&mut self, data: &[u8]) -> Result<(), TxError>{
        match self.driver.start_tx(data) {
            Ok(()) => Ok(()), 
            Err(TxStartError::InvalidArgumentError(_)) => Err(TxError::InvalidBufferSize),
            Err(TxStartError::IoError(_)) => Err(TxError::IoError), 
        }
    }

    /// Check whether the radio has finished sending.
    pub fn transmit_is_complete(&mut self) -> nb::Result<(), IoError> {
        match self.driver.complete_tx(){
            Ok(None) => Err(WouldBlock),    // Still sending
            Ok(_) => Ok(()),                // Sending complete
            Err(e) => Err(Other(e)),
        }
    }
    /// Tell the radio to listen for a packet and return immediately. Check whether anything was recieved by calling `recieve_is_complete()`.
    /// 
    /// A timeout value is optional, if none is provided the maximum timeout is used. You should prepare to deal with timeouts.
    pub fn recieve_start(&mut self, timeout: Option<Duration>) {
        let timeout = match timeout {
            Some(t) => t,
            None => self.driver.rx_timeout_max().unwrap(),
        };
        self.driver.start_rx(timeout).unwrap();
    }

    /// Check whether the radio has recieved a packet. If so, returns a reference to the slice of buf that contains the message.
    /// 
    /// If not, returns either `StillRecieving` or `RxTimeout`. In the timeout case you should call `recieve_start()` again.
    pub fn recieve_is_complete<'a>(&mut self, buf: &'a mut [u8; rfm95::RFM95_FIFO_SIZE]) -> nb::Result<&'a [u8], RxCompleteError> {
        match self.driver.complete_rx(buf.as_mut_slice()) {
            Ok(Some(n)) => Ok(&buf[0..n]),
            Ok(None) => Err(WouldBlock),
            Err(e) => Err(Other(e)),
        }
    }
}

#[derive(Debug)]
pub enum RxError {
    CrcFailure,
    Timeout,
    IoError,
}

#[derive(Debug)]
pub enum TxError {
    InvalidBufferSize,
    IoError,
}

pub mod tests {
    use embedded_lora_rfm95::error::RxCompleteError;
    use ufmt::uwrite;

    pub fn range_test_tx(mut board: crate::board::Board) -> ! {
        let mut current_time = Time::default();
        board.radio_delay_timer.start(msp430fr2x5x_hal::clock::REFOCLK); // 1 second timer
        board.radio.transmit_start(&time_to_bytes(&current_time)).unwrap();
        loop {
            // Sends at most one message per second.
            if board.radio_delay_timer.wait().is_ok() {
                current_time.increment();
                
                if board.radio.transmit_is_complete().is_ok() {
                    board.radio.transmit_start(&time_to_bytes(&current_time)).unwrap();
                }
            }
        }
    }

    fn time_to_bytes(time: &Time) -> [u8;8] {
        [
            time.hours / 10 + b'0', 
            time.hours % 10 + b'0', 
            b':', 
            time.minutes / 10 + b'0', 
            time.minutes % 10 + b'0', 
            b':', 
            time.seconds / 10 + b'0', 
            time.seconds % 10 + b'0'
        ]
    }

    pub fn range_test_rx(mut board: crate::board::Board) -> ! {
        let mut buf = [0u8; super::RFM95_FIFO_SIZE];
        let mut current_time = Time::default();
        board.radio_delay_timer.start(msp430fr2x5x_hal::clock::REFOCLK); // 1 second timer
        board.radio.recieve_start(None);
        loop {
            match board.radio.recieve_is_complete(&mut buf) {
                Err(nb::Error::Other(RxCompleteError::TimeoutError(_))) => board.radio.recieve_start(None),
                Err(_e) => (),
                Ok(msg) => {
                    let Ok(signal_strength) = board.radio.driver.get_packet_strength() else {continue};
                    let Ok(rssi) = board.radio.driver.get_packet_rssi() else {continue};
                    let Ok(snr) = board.radio.driver.get_packet_snr() else {continue};
                    // crate::println!("[{}] '{}', Strength: {}, RSSI: {}, SNR: {}", current_time, core::str::from_utf8(msg).unwrap(), signal_strength, rssi, snr);
                    board.radio.recieve_start(None);
                },
            }
            if board.radio_delay_timer.wait().is_ok() {
                current_time.increment();
            }
        }
    }
    #[derive(Default)]
    struct Time {
        seconds: u8,
        minutes: u8,
        hours: u8,
    }
    impl Time {
        /// Add one second to the time.
        pub fn increment(&mut self) {
            if self.seconds < 59 {
                self.seconds += 1;
                return;
            }

            self.seconds = 0;
            if self.minutes < 59 {
                self.minutes += 1;
                return;
            }

            self.minutes = 0;
            self.hours += 1;
        }
    }
    impl ufmt::uDisplay for Time {
        fn fmt<W: ufmt::uWrite + ?Sized>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error> {
            for (i, &val) in [self.hours, self.minutes, self.seconds].iter().enumerate() {
                if val < 10 {
                    ufmt::uwrite!(f, "0{}", val)?;
                }
                else {
                    ufmt::uwrite!(f, "{}", val)?; 
                }
                if i != 2 {
                    ufmt::uwrite!(f, ":")?;
                }
            }
            Ok(())
        }
    }
}