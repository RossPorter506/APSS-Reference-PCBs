use msp430fr2x5x_hal::{gpio::{Alternate1, Floating, Input, Output, Pin, Pin0, Pin1, Pin2, Pin3, Pin4, Pin5, Pin6, Pin7, Pulldown, Pullup},
pac::{EUsciA0, EUsciA1, P1, P2, P3, P4, P5}, serial::{Rx, Tx}, spi::Spi};

pub type BCtrl0Pin          = Pin<P4, Pin4, Input<Pulldown>>;
pub type BCtrl1Pin          = Pin<P4, Pin5, Input<Pullup>>;

pub type TristateEnPin      = Pin<P5, Pin1, Output>;

pub type AudioPwmPin        = Pin<P5, Pin0, Alternate1<Output>>;

pub type GpsEusci           = EUsciA0;
pub type GpsTx              = Tx<EUsciA0>;
pub type GpsRx              = Rx<EUsciA0>;

// Tx TO the GPS
pub type GpsTxPin           = Pin<P1, Pin7, Alternate1<Input<Floating>>>;
// Rx FROM the GPS
pub type GpsRxPin           = Pin<P1, Pin6, Alternate1<Input<Floating>>>;
pub type GpsResetPin        = Pin<P3, Pin3, Output>;
pub type GpsEnPin           = Pin<P3, Pin2, Output>;

pub type RadioEusci         = EUsciA1;
pub type RadioSpi           = Spi<EUsciA1>;
pub type RadioCsPin         = Pin<P4, Pin0, Output>;
pub type RadioResetPin      = Pin<P2, Pin3, Output>;

pub type SpiMisoPin         = Pin<P4, Pin2, Alternate1<Input<Floating>>>;
pub type SpiMosiPin         = Pin<P4, Pin3, Alternate1<Input<Floating>>>;
pub type SpiSclkPin         = Pin<P4, Pin1, Alternate1<Input<Floating>>>;
