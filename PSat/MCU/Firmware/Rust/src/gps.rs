#![allow(dead_code)]

use core::{fmt::Debug, num::ParseIntError};

use arrayvec::{ArrayString, ArrayVec};
use msp430fr2x5x_hal::{
    clock::Smclk, 
    serial::{BitCount, BitOrder, Loopback, Parity, RecvError, SerialConfig, StopBits}};
use embedded_hal_nb::serial::Read;
use ufmt::{derive::uDebug, uDisplay, uwrite};
use crate::pin_mappings::{GpsEusci, GpsRx, GpsRxPin, GpsTx, GpsTxPin};

const NMEA_MESSAGE_MAX_LEN: usize = 82;

pub struct Gps {
    tx: GpsTx,
    rx: GpsRx,
    rx_started: bool,
}
impl Gps {
    pub fn new(eusci_reg: GpsEusci, smclk: &Smclk, tx_pin: GpsTxPin, rx_pin: GpsRxPin) -> Self {
        // Configure UART peripheral
        let (tx, rx) = SerialConfig::new(eusci_reg, 
            BitOrder::LsbFirst, 
            BitCount::EightBits, 
            StopBits::OneStopBit, 
            Parity::NoParity, 
            Loopback::NoLoop, 
            9600)
            .use_smclk(smclk)
            .split(tx_pin, rx_pin);
        Self {tx, rx, rx_started: false}
    } 

    /// Slowly builds up a message byte by byte by checking the serial buffer. Call this function repeatedly until it returns `Ok`.
    /// 
    /// This function must be called sufficiently frequently to ensure that the serial buffer does not overrun.
    /// 
    /// After this function returns `Ok(())`, calling it again will clear the buffer to prepare for the next message.
    pub fn get_nmea_message_string(&mut self, buf: &mut ArrayString::<NMEA_MESSAGE_MAX_LEN>) -> nb::Result<(), RecvError> {
        if !self.rx_started {
            buf.clear();
            self.rx_started = true;
        }
        let chr = self.rx.read()?;
        
        if buf.is_empty() { // Wait until new message starts before recording
            if chr == b'$' { 
                buf.push('$');
            }
            return Err(nb::Error::WouldBlock);
        }
        if chr == b'\n' { // Message has finished
            buf.push('\n');
            self.rx_started = false;
            return Ok(());
        }
        buf.push(chr as char);
        Err(nb::Error::WouldBlock)
    }

    /// Get a GPS GGA packet as an ArrayString. Useful if you're just sending over the radio or logging to an SD card.
    /// 
    /// Slowly builds up a GGA message byte by byte by checking the serial buffer. Call this function repeatedly until it returns `Ok`.
    /// 
    /// This function must be called sufficiently frequently to ensure that the serial buffer does not overrun.
    /// 
    /// After this function returns `Ok(())`, calling it again will clear the buffer to prepare for the next message.
    pub fn get_gga_message_string(&mut self, buf: &mut ArrayString::<NMEA_MESSAGE_MAX_LEN>) -> nb::Result<(), RecvError> {
        self.get_nmea_message_string(buf)?;

        if &buf[3..6] == "GGA" { Ok(()) } 
        else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Get a GPS GGA packet as a struct. Useful for on-device computation.
    /// 
    /// Slowly builds up a GGA message byte by byte by checking the serial buffer. Call this function repeatedly until it returns `Ok`.
    /// 
    /// This function must be called sufficiently frequently to ensure that the serial buffer does not overrun.
    /// 
    /// After this function returns `Ok(())`, calling it again will clear the buffer to prepare for the next message.
    pub fn get_gga_message(&mut self, buf: &mut ArrayString::<NMEA_MESSAGE_MAX_LEN>) -> nb::Result<GgaMessage, GgaParseError> {
        match self.get_gga_message_string(buf) {
            Ok(_) => Ok( GgaMessage::try_from(&*buf)? ),
            Err(nb::Error::WouldBlock) => Err(nb::Error::WouldBlock),
            Err(nb::Error::Other(e)) => Err(nb::Error::Other(GgaParseError::SerialError(e))),
        }
    }
}

// A GGA packet in struct form. Useful for interpreting the results on-device.
pub struct GgaMessage {
    pub utc_time: UtcTime,
    pub latitude: Degrees,
    pub longitude: Degrees,
    pub fix_type: GpsFixType,
    pub num_satellites: u8,
    pub altitude_msl: Altitude,
}
impl TryFrom<&ArrayString<NMEA_MESSAGE_MAX_LEN>> for GgaMessage {
    type Error = GgaParseError;

    fn try_from(msg: &ArrayString<NMEA_MESSAGE_MAX_LEN>) -> Result<Self, Self::Error> {
        let sections: ArrayVec<&str, 15> = msg.split(',').take(15).collect();
        if sections.len() != 15 { return Err(GgaParseError::WrongSectionCount) }

        let fix_type = GpsFixType::try_from(sections[6]).map_err(|_| GgaParseError::InvalidGpsFixType)?;
        if fix_type == GpsFixType::None { return Err(GgaParseError::NoFix) }

        Ok( GgaMessage { 
            utc_time: UtcTime::try_from(sections[1])                .unwrap(),//.map_err(GgaParseError::UtcParseError)?, 
            latitude:  Degrees::try_from((sections[2], sections[3])).unwrap(),//.map_err(GgaParseError::LatLongParseError)?, 
            longitude: Degrees::try_from((sections[4], sections[5])).unwrap(),//.map_err(GgaParseError::LatLongParseError)?, 
            num_satellites: sections[7].parse()                     .unwrap(),//.map_err(GgaParseError::InvalidSatelliteNumber)?, 
            altitude_msl: Altitude::try_from(sections[9])           .unwrap(),//.map_err(GgaParseError::AltitudeParseError)?, 
            fix_type,
        })
    }
}

pub enum GgaParseError {
    NoFix,
    SerialError(RecvError),
    WrongSectionCount,
    LatLongParseError(LatLongParseError),
    InvalidGpsFixType,
    InvalidSatelliteNumber(ParseIntError),
    UtcParseError(UtcError),
    AltitudeParseError(ParseIntError),
}
impl Debug for GgaParseError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::SerialError(arg0) => {
                let str = match arg0 {
                    RecvError::Framing => "Framing",
                    RecvError::Parity => "Parity",
                    RecvError::Overrun(_) => "Overrun",
                };
                f.debug_tuple("SerialError").field(&str).finish()
            }
            e => write!(f, "{:?}", e),
        }
    }
}

/// A UTC timestamp
pub struct UtcTime {
    pub hours: u8,
    pub minutes: u8,
    pub seconds: u8,
    pub millis: u16, 
}
impl uDisplay for UtcTime {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized {
        for time in [self.hours, self.minutes] {
            match time {
                0..10 => uwrite!(f, "0{}:", time)?,
                10.. =>  uwrite!(f,  "{}:", time)?,
            };
        }

        match self.seconds {
            0..10 =>  uwrite!(f, "0{}.", self.seconds)?,
            10..  =>  uwrite!(f,  "{}.", self.seconds)?,
        };

        match self.millis {
            0..10   => uwrite!(f, "00{} UTC", self.millis)?,
            10..100 => uwrite!(f,  "0{} UTC", self.millis)?,
            100..   => uwrite!(f,   "{} UTC", self.millis)?,
        };

        Ok(())
    }
}
impl TryFrom<&str> for UtcTime {
    type Error = UtcError;

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        if value.len() < 6 {
            return Err(UtcError::StrTooShort)
        }
        Ok(UtcTime { 
            hours: value[0..2].parse().map_err(UtcError::ParseError)?, 
            minutes: value[2..4].parse().map_err(UtcError::ParseError)?, 
            seconds: value[4..6].parse().map_err(UtcError::ParseError)?, 
            millis: value.get(7..).unwrap_or("0").parse().map_err(UtcError::ParseError)? })
    }
}
#[derive(Debug)]
pub enum UtcError {
    StrTooShort,
    ParseError(ParseIntError),
}

/// A degrees value, stored as a decimal fraction.
pub struct Degrees {
    degrees: i16,
    degrees_millionths: u32,
}
impl uDisplay for Degrees {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized {
        let mut leading_zeroes = ArrayString::<6>::new(); 
        for _ in 0..5-self.degrees_millionths.checked_ilog10().unwrap_or(0) {
            leading_zeroes.push('0');
        }

        uwrite!(f, "{}.{}{} deg", self.degrees, leading_zeroes.as_str(), self.degrees_millionths)
    }
}
impl TryFrom<(&str, &str)> for Degrees {
    type Error = LatLongParseError;

    fn try_from(value: (&str, &str)) -> Result<Self, Self::Error> {
        let (degrees_str, compass_direction) = value;
        if degrees_str.is_empty() || compass_direction.is_empty() {
            return Err(LatLongParseError::NoData);
        }
        let degrees: i16; 
        let minutes_str: &str;
        let minutes_frac_str: &str;
        let (first_half, _) = degrees_str.split_once('.').unwrap();
    
        if first_half.len() == 4 { // ddmm
            degrees          =  degrees_str[0..2].parse().unwrap();
            minutes_str      = &degrees_str[2..4];
            minutes_frac_str = &degrees_str[5..9];

        } else { // dddmm
            degrees          =  degrees_str[0..3].parse().unwrap();
            minutes_str      = &degrees_str[3..5];
            minutes_frac_str = &degrees_str[6..10];
        }

        // 24.3761 -> 243761
        let mut minutes_times_10000 = ArrayString::<6>::from(minutes_str).unwrap();
        minutes_times_10000.push_str(minutes_frac_str);

        let degrees_millionths: u32 = minutes_times_10000.parse::<u32>().unwrap() * 100 / 60;
    
        match compass_direction {
            "N" | "E" => Ok(Degrees{degrees,            degrees_millionths}),
            "S" | "W" => Ok(Degrees{degrees: -degrees,  degrees_millionths}),
            _ => Err(LatLongParseError::InvalidCompassDirection)
        }
    }
}
#[derive(Debug)]
pub enum LatLongParseError {
    NoData,
    InvalidCompassDirection,
}

#[derive(Debug, uDebug, PartialEq, Eq)]
pub enum GpsFixType {
    None = 0,
    Gps = 1,
    DifferentialGps = 2,
}
impl TryFrom<&str> for GpsFixType{
    type Error = ();

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        Ok(match value {
            "0" => GpsFixType::None,
            "1" => GpsFixType::Gps,
            "2" => GpsFixType::DifferentialGps,
            _ => return Err(()), // should be unreachable
        })
    }
}

pub struct Altitude{
    decimetres: i32,
}
impl TryFrom<&str> for Altitude {
    type Error = ParseIntError;

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        let (whole, frac) = value.split_once(".").unwrap();
        Ok(Altitude{ decimetres: whole.parse::<i32>()?*10 + frac[..1].parse::<i32>()?})
    }
}
impl uDisplay for Altitude {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where W: ufmt::uWrite + ?Sized { 
        let metres = self.decimetres / 10;

        uwrite!(f, "{}.{}m", metres, self.decimetres - metres*10 )
    }
}