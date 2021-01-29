pub use stm32h7xx_hal as hal;
use hal::gpio;


#[allow(non_snake_case)]
pub struct Pins {
    pub D0:  gpio::gpiob::PB7<gpio::Analog>,  // PIN_16,
    pub D16: gpio::gpioc::PC6<gpio::Analog>,  // PIN_01,
    // TODO
}
