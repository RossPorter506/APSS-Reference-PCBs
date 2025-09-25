use core::{marker::PhantomData, ops::{Deref, DerefMut}};

use embedded_hal::i2c::I2c;
use icm42670::{accelerometer::vector::{I16x3, I32x3}, AccelRange, Address, GyroRange, Icm42670};
use icm42670::prelude::*;
use icm42670::accelerometer::Error as AccelerometerError;
use icm42670::Error;

// A simple wrapper to add some integer-based methods.
// Because we implement `Deref` you can transparently access the methods defined on `Icm42670` as if they were defined on `Imu`.

/// Intertial Management Unit - includes accelerometer, gyroscope and temperature sensor.
pub struct Imu<I2C: I2c> {
    imu: Icm42670<I2C>,
    _i2c: PhantomData<I2C>,
}
impl<I:I2c> Deref for Imu<I> {
    type Target = Icm42670<I>;

    fn deref(&self) -> &Self::Target {
        &self.imu
    }
}
impl<I:I2c> DerefMut for Imu<I> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.imu
    }
}

impl<E:core::fmt::Debug, I: I2c<Error=E>> Imu<I> {
    pub fn new(i2c: I, address: Address) -> Result<Self, Error<E>> {
        let imu = Icm42670::new(i2c, address)?;
        Ok( Self{imu, _i2c: PhantomData} )
    }

    #[allow(dead_code)]
    /// Returns acceleration data as integers in milli-g's, e.g. 2_000 = 2g's
    pub fn accel_int_millis(&mut self) -> Result<I16x3, AccelerometerError<Error<E>>> {
        let scale = match self.imu.accel_range()? {
            AccelRange::G2  => 16_384, // Values from datasheet
            AccelRange::G4  =>  8_192,
            AccelRange::G8  =>  4_096,
            AccelRange::G16 =>  2_048,
        };

        // Scale the raw Accelerometer data using the appropriate factor based on the configured range.
        let raw = self.imu.accel_raw()?;
        let x = ((raw.x as i32 * 1000) / scale) as i16;
        let y = ((raw.y as i32 * 1000) / scale) as i16;
        let z = ((raw.z as i32 * 1000) / scale) as i16;

        Ok(I16x3{x, y, z})
    }

    #[allow(dead_code)]
    /// Return the gyro data for each of the three axes as integers in milli-degrees, e.g. 2_000 = 2 degrees
    pub fn gyro_int_millis(&mut self) -> Result<I32x3, Error<E>> {
        let scale_10x = match self.gyro_range()? {
            GyroRange::Deg250  => 1310, // Values from datasheet
            GyroRange::Deg500  =>  655,
            GyroRange::Deg1000 =>  328,
            GyroRange::Deg2000 =>  164,
        };

        // Scale the raw Gyroscope data using the appropriate factor based on the configured range.
        let raw = self.gyro_raw()?;
        let x = ((raw.x as i32 * 1000) / scale_10x) * 10;
        let y = ((raw.y as i32 * 1000) / scale_10x) * 10;
        let z = ((raw.z as i32 * 1000) / scale_10x) * 10;

        Ok(I32x3::new(x, y, z))
    }

    #[allow(dead_code)]
    /// Read the built-in temperature sensor and return the value in degrees centigrade
    pub fn temperature_int(&mut self) -> Result<i16, Error<E>> {
        let raw = self.temperature_raw()?;
        let deg = (raw / 128) + 25; // Eqn from datasheet

        Ok(deg)
    }
}
