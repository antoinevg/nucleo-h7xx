pub use stm32h7xx_hal as hal;
use hal::gpio;

use crate::ethernet;


#[allow(non_snake_case)]
pub struct Pins {
    // "Arduino" pins
    pub D51: gpio::gpiod::PD7<gpio::Analog>,
    pub D52: gpio::gpiod::PD6<gpio::Analog>,

    pub ethernet: ethernet::Pins,

    // TODO
}
