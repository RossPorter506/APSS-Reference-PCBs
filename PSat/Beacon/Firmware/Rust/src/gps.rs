#![allow(dead_code)]

use core::{fmt::Debug, num::ParseIntError};

use arrayvec::{ArrayString, ArrayVec};
use embedded_hal::digital::OutputPin;
use msp430fr2x5x_hal::{
    clock::Smclk, 
    serial::{BitCount, BitOrder, Loopback, Parity, RecvError, SerialConfig, StopBits}};
use ufmt::{derive::uDebug, uDisplay, uwrite};
use crate::pin_mappings::{GpsEnPin, GpsEusci, GpsRx, GpsRxPin, GpsTx, GpsTxPin};
use embedded_hal_nb::serial::Read;

pub const NMEA_MESSAGE_MAX_LEN: usize = 82;
const GPS_BAUDRATE: u32 = 115200;

pub struct Gps {
    tx: GpsTx,
    rx: GpsRx,
    rx_started: bool,
    gps_en: GpsEnPin,
}
impl Gps {
    pub fn new(eusci_reg: GpsEusci, smclk: &Smclk, tx_pin: GpsTxPin, rx_pin: GpsRxPin, gps_en: GpsEnPin) -> Self {
        // Configure UART peripheral
        let (tx, rx) = SerialConfig::new(eusci_reg, 
            BitOrder::LsbFirst, 
            BitCount::EightBits, 
            StopBits::OneStopBit, 
            Parity::NoParity, 
            Loopback::NoLoop, 
            GPS_BAUDRATE)
            .use_smclk(smclk)
            .split(tx_pin, rx_pin);
        Self {tx, rx, gps_en, rx_started: false}
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

    pub fn enable(&mut self) {
        self.gps_en.set_low();
    }

    pub fn disable(&mut self) {
        self.gps_en.set_high();
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
impl GgaMessage {
    /// Produce a string that represents the GPS data.
    pub fn encode_string(&self) -> ArrayString<60> {
        let num_sats = ArrayString::<3>::new();
        match self.num_satellites {
            0..=9 => ufmt::uwrite!(ufmt_utils::WriteAdapter(num_sats),"0{}", self.num_satellites).unwrap(),
            10..  => ufmt::uwrite!(ufmt_utils::WriteAdapter(num_sats),"{}", self.num_satellites).unwrap()
        };
        let fix_type = match self.fix_type {
            GpsFixType::None => 'N',
            GpsFixType::Gps => 'G',
            GpsFixType::DifferentialGps => 'D',
        };
        let (time, lat, long, alt) = (self.utc_time.clone(), self.latitude.clone(), self.longitude.clone(), self.altitude_msl);

        let data_str = ArrayString::new();
        ufmt::uwrite!(ufmt_utils::WriteAdapter(data_str), "#{} {}; {}; {}; {},{}; {}\n", crate::TEAM_ID + b'A', time, fix_type, num_sats.as_str(), lat, long, alt).unwrap();
        data_str
    }
    /// Produce a binary-encoded byte array that represents the GPS data.
    pub fn encode_binary(&self) -> [u8; 14] {
        let mut bytes = [0;14];
        const { assert!(crate::TEAM_ID < 32); } // 5 bits
        let hours: u8 = self.utc_time.hours.clamp(0, 23); // 5 bits
        let minutes: u8 = self.utc_time.minutes.clamp(0, 59); // 6 bits
        let seconds: u8 = self.utc_time.seconds.clamp(0, 59); // 6 bits
        let num_sats: u8 = self.num_satellites.clamp(0, 15); // 4 bits
        let altitude: i32 = self.altitude_msl.decimetres.clamp(-0x7FFFF, 0x7FFFF); // 20 bits
        let latitude: i32 = self.latitude.degrees as i32 * 10_000_000 + self.latitude.degrees_millionths as i32 * 100;
        let longitude: i32 = self.longitude.degrees as i32 * 10_000_000 + self.longitude.degrees_millionths as i32 * 100;
        let fix_ok: bool = self.fix_type != GpsFixType::None;
        let is_dgps: bool = self.fix_type == GpsFixType::DifferentialGps;

        // team_id | hours[5..2]
        bytes[0]  = (crate::TEAM_ID << 3) | (self.utc_time.hours >> 2);

        // hours[1..0] | minutes
        bytes[1]  = ((hours & 0b11) << 6) | minutes;

        // seconds | fix_ok | is_dgps
        bytes[2]  = (seconds << 2) | ((fix_ok as u8) << 1) | (is_dgps as u8);

        // num_sats | altitude[20..17]
        bytes[3]  = (num_sats << 4) | (((altitude >> 16) & 0xF) as u8);

        // altitude[16..0]
        bytes[4..=5].copy_from_slice(&altitude.to_be_bytes()[2..=3]);

        // latitude[32..0]
        bytes[6..=9].copy_from_slice(&latitude.to_be_bytes());
        
        // longitude[32..0]
        bytes[10..=13].copy_from_slice(&longitude.to_be_bytes());

        bytes
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

#[derive(Debug, Clone)]
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
            0..10 =>  uwrite!(f, "0{} UTC", self.seconds)?,
            10..  =>  uwrite!(f,  "{} UTC", self.seconds)?,
        };

        // match self.millis {
        //     0..10   => uwrite!(f, "00{} UTC", self.millis)?,
        //     10..100 => uwrite!(f,  "0{} UTC", self.millis)?,
        //     100..   => uwrite!(f,   "{} UTC", self.millis)?,
        // };

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

#[derive(Debug, Clone)]
/// A degrees value, stored as a decimal fraction.
pub struct Degrees {
    pub degrees: i16,
    pub degrees_millionths: u32,
}
impl uDisplay for Degrees {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized {
        let mut leading_zeroes = ArrayString::<6>::new(); 
        for _ in 0..5-self.degrees_millionths.checked_ilog10().unwrap_or(0) {
            leading_zeroes.push('0');
        }

        uwrite!(f, "{}.{}{}", self.degrees, leading_zeroes.as_str(), self.degrees_millionths)
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

#[derive(Debug, Copy, Clone)]
pub struct Altitude{
    pub decimetres: i32,
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