#![allow(dead_code)]

// Drivers for LEDs.

use core::convert::Infallible;

use embedded_hal::digital::StatefulOutputPin;

/// Common interface implemented by both active high and active low LEDs
pub trait Led {
    /// Turn the LED on
    fn on(&mut self);
    /// Turn the LED off
    fn off(&mut self);
    /// Toggle the LED
    fn toggle(&mut self);
}

/// An active low LED
pub struct LedActiveLow<PIN: StatefulOutputPin<Error=Infallible>> {
    pin: PIN,
}
impl<PIN: StatefulOutputPin<Error=Infallible>> LedActiveLow<PIN> {
    pub fn new(pin: PIN) -> Self {
        let mut s = Self { pin };
        s.off();
        s
    }
}
impl<PIN: StatefulOutputPin<Error=Infallible>> Led for LedActiveLow<PIN> {
    #[inline(always)]
    fn on(&mut self) {
        self.pin.set_low();
    }
    #[inline(always)]
    fn off(&mut self) {
        self.pin.set_high();
    }
    #[inline(always)]
    fn toggle(&mut self) {
        self.pin.toggle();
    }
}

/// An active high LED
pub struct LedActiveHigh<PIN: StatefulOutputPin<Error=Infallible>> {
    pin: PIN,
}
impl<PIN: StatefulOutputPin<Error=Infallible>> LedActiveHigh<PIN> {
    pub fn new(pin: PIN) -> Self {
        let mut s = Self { pin };
        s.off();
        s
    }
}
impl<PIN: StatefulOutputPin<Error=Infallible>> Led for LedActiveHigh<PIN> {
    #[inline(always)]
    fn on(&mut self) {
        self.pin.set_high();
    }
    #[inline(always)]
    fn off(&mut self) {
        self.pin.set_low();
    }
    #[inline(always)]
    fn toggle(&mut self) {
        self.pin.toggle();
    }
}

/// An RGB LED
pub struct RgbLed<RedLed: Led, GreenLed: Led, BlueLed: Led> {
    pub red: RedLed,
    pub green: GreenLed,
    pub blue: BlueLed,
}
impl<RedLed: Led, GreenLed: Led, BlueLed: Led> RgbLed<RedLed, GreenLed, BlueLed> {
    #[inline(always)]
    pub fn new(red_led: RedLed, green_led: GreenLed, blue_led: BlueLed) -> Self {
        RgbLed { red: red_led, green: green_led, blue: blue_led }
    }
}

impl<RedLed: Led, GreenLed: Led, BlueLed: Led> RgbLed<RedLed, GreenLed, BlueLed> {
    /// Turn the RGB LED off
    pub fn off(&mut self) {
        self.red.off();
        self.green.off();
        self.blue.off();
    }
    /// Make the RGB LED red
    pub fn red(&mut self) {
        self.red.on();
        self.green.off();
        self.blue.off();
    }
    /// Make the RGB LED green
    pub fn green(&mut self) {
        self.red.off();
        self.green.on();
        self.blue.off();
    }
    /// Make the RGB LED blue
    pub fn blue(&mut self) {
        self.red.off();
        self.green.off();
        self.blue.on();
    }
    /// Make the RGB LED cyan
    pub fn cyan(&mut self) {
        self.red.off();
        self.green.on();
        self.blue.on();
    }
    /// Make the RGB LED yellow
    pub fn yellow(&mut self) {
        self.red.on();
        self.green.on();
        self.blue.off();
    }
    /// Make the RGB LED magenta
    pub fn magenta(&mut self) {
        self.red.on();
        self.green.off();
        self.blue.on();
    }
    /// Make the RGB LED white
    pub fn white(&mut self) {
        self.red.on();
        self.green.on();
        self.blue.on();
    }
}
