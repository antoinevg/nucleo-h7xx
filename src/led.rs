#[cfg(not(feature = "audio_hal"))]
pub use stm32h7xx_hal as hal;
#[cfg(feature = "audio_hal")]
pub use stm32h7xx_hal_dma as hal;

use hal::hal as embedded_hal;

use hal::gpio;
use embedded_hal::digital::v2::OutputPin;


// - traits -------------------------------------------------------------------

/// Generic LED
pub trait Led {
    /// Turns the LED off
    fn off(&mut self);

    /// Turns the LED on
    fn on(&mut self);
}


// - types --------------------------------------------------------------------

pub struct Pins {
    pub ld_1: gpio::gpiob::PB0<gpio::Output<gpio::PushPull>>,   // SB65=off, SB54=on    - TODO feature
    //pub ld_1: gpio::gpioa::PA5<gpio::Output<gpio::PushPull>>,   // SB65=on,  SB54=off
    pub ld_2: gpio::gpioe::PE1<gpio::Output<gpio::PushPull>>,
    pub ld_3: gpio::gpiob::PB14<gpio::Output<gpio::PushPull>>,
    // TODO
}

#[allow(non_snake_case)]
pub struct UserLeds {
    pub LD1: UserLed<gpio::gpiob::PB0<gpio::Output<gpio::PushPull>>>,   // green - TODO feature
    //pub LD1: UserLed<gpio::gpioa::PA5<gpio::Output<gpio::PushPull>>>,   // green
    pub LD2: UserLed<gpio::gpioe::PE1<gpio::Output<gpio::PushPull>>>,   // yellow
    pub LD3: UserLed<gpio::gpiob::PB14<gpio::Output<gpio::PushPull>>>,  // red
}

impl UserLeds {
    pub fn new(pins: Pins) -> Self {
        Self {
            LD1: UserLed(pins.ld_1),
            LD2: UserLed(pins.ld_2),
            LD3: UserLed(pins.ld_3),
        }
    }
}

pub struct UserLed<P>(P);

impl<P> Led for UserLed<P>
where P: OutputPin +  {
    fn on(&mut self) {
        match self.0.set_high() {
            Ok(()) => (),
            Err(_) => (),
        }
    }

    fn off(&mut self) {
        match self.0.set_low() {
            Ok(()) => (),
            Err(_) => (),
        }
    }
}
