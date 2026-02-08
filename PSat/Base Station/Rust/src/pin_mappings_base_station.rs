use msp430fr2x5x_hal::{gpio::{Alternate1, Floating, Input, Output, Pin, Pin0, Pin1, Pin2, Pin3, Pin4, Pin5, Pin6, Pin7, Pulldown, Pullup},
pac::{E_USCI_A0, E_USCI_A1, E_USCI_B0, P1, P2, P3, P4, P5, P6}, serial::{Rx, Tx}, spi::Spi};

//pub type BCtrl0Pin          = Pin<P4, Pin4, Input<Pulldown>>;
//pub type BCtrl1Pin          = Pin<P4, Pin5, Input<Pullup>>;

//pub type TristateEnPin      = Pin<P5, Pin1, Output>;

//pub type AudioPwmPin        = Pin<P5, Pin0, Alternate1<Output>>;

pub type GpsEusci           = E_USCI_A1;
pub type GpsTx              = Tx<E_USCI_A1>;
pub type GpsRx              = Rx<E_USCI_A1>;

// Tx TO the GPS
pub type GpsTxPin           = Pin<P4, Pin2, Alternate1<Input<Floating>>>;
// Rx FROM the GPS
pub type GpsRxPin           = Pin<P4, Pin3, Alternate1<Input<Floating>>>; 

pub type GpsResetPin        = Pin<P6, Pin2, Output>;
// GPS Enable Pin (Inverted)
pub type GpsEnPin           = Pin<P4, Pin0, Output>;

pub type RadioEusci         = E_USCI_B0;
pub type RadioSpi           = Spi<E_USCI_B0>;
// Radio CS Pin (Inverted)
pub type RadioCsPin         = Pin<P4, Pin4, Output>;
pub type RadioResetPin      = Pin<P6, Pin3, Output>;

pub type SpiMisoPin         = Pin<P1, Pin3, Alternate1<Input<Floating>>>;
pub type SpiMosiPin         = Pin<P1, Pin2, Alternate1<Input<Floating>>>;
pub type SpiSclkPin         = Pin<P1, Pin1, Alternate1<Input<Floating>>>;

// Display pins
pub type DispSdaPin        = Pin<P4, Pin6, Alternate1<Input<Floating>>>;
pub type DispSclPin        = Pin<P4, Pin7, Alternate1<Input<Floating>>>;