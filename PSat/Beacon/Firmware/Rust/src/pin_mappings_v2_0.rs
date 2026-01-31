use msp430fr2x5x_hal::{gpio::{Alternate1, Alternate3, Floating, Input, Output, Pin, Pin0, Pin1, Pin2, Pin3, Pin4, Pin5, Pin6, Pin7, Pullup},
pac::{E_USCI_A0, E_USCI_A1, E_USCI_B1, P1, P2, P3, P4, P5, P6}, serial::{Rx, Tx}, spi::SpiBus};

pub type GpsEusci           = E_USCI_A1;
pub type GpsTx              = Tx<E_USCI_A1>;
pub type GpsRx              = Rx<E_USCI_A1>;

pub type GpsTxPin           = Pin<P4, Pin3, Alternate1<Input<Floating>>>;
pub type GpsRxPin           = Pin<P4, Pin2, Alternate1<Input<Floating>>>;

pub type RadioEusci         = E_USCI_B1;
pub type RadioSpi           = SpiBus<E_USCI_B1>;
pub type RadioCsPin         = Pin<P4, Pin4, Output>;
pub type RadioResetPin      = Pin<P5, Pin2, Output>;

pub type RedLedPin          = Pin<P2, Pin0, Output>;
pub type BlueLedPin         = Pin<P2, Pin1, Output>;
pub type GreenLedPin        = Pin<P2, Pin2, Output>;

pub type SpiMisoPin         = Pin<P4, Pin7, Alternate1<Input<Floating>>>;
pub type SpiMosiPin         = Pin<P4, Pin6, Alternate1<Input<Floating>>>;
pub type SpiSclkPin         = Pin<P4, Pin5, Alternate1<Input<Floating>>>;
pub type LoraResetPin       = Pin<P5, Pin2, Output>;
pub type LoraCsPin          = Pin<P4, Pin4, Output>;

pub type DebugEusci         = E_USCI_A0;
pub type DebugTxPin:        = Pin<P1, Pin7, Alternate1<Input<Floating>>>;

pub type I2cSdaPin:         = Pin<P1, Pin2, Alternate1<Input<Floating>>>;
pub type I2cSclPin:         = Pin<P1, Pin3, Alternate1<Input<Floating>>>;

pub type LoraIrqPin:        = Pin<P5, Pin3, Input<Floating>>;
pub type GpsEnPin:          = Pin<P4, Pin1, Output>;

pub type HalfVbatPin:       = Pin<P5, Pin0, Alternate3<Input<Floating>>>;

pub type PowerGood1v8Pin    = Pin<P3, Pin0, Input<Pullup>>;
pub type PowerGood3v3Pin    = Pin<P3, Pin1, Input<Pullup>>;
pub type Enable1v8Pin       = Pin<P3, Pin2, Output>;
pub type Enable5vPin        = Pin<P3, Pin3, Output>;