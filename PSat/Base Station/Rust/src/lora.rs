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
