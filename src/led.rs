pub use stm32h7xx_hal as hal;

use embedded_hal::digital::v2::OutputPin;
use hal::hal as embedded_hal;

use crate::pins::user_leds;

// - traits -------------------------------------------------------------------

/// Generic LED
pub trait Led {
    /// Turns the LED off
    fn off(&mut self);

    /// Turns the LED on
    fn on(&mut self);
}

// - UserLed ------------------------------------------------------------------

pub struct UserLed<PIN>(PIN);

impl<PIN> Led for UserLed<PIN>
where
    PIN: OutputPin,
{
    fn on(&mut self) {
        if let Ok(()) = self.0.set_high() {}
    }

    fn off(&mut self) {
        if let Ok(()) = self.0.set_low() {}
    }
}

// - UserLeds -----------------------------------------------------------------

pub struct UserLeds {
    pub ld1: UserLed<user_leds::Ld1>,
    pub ld2: UserLed<user_leds::Ld2>,
    pub ld3: UserLed<user_leds::Ld3>,
}

impl UserLeds {
    pub fn new(pins: user_leds::Pins) -> Self {
        Self {
            ld1: UserLed(pins.ld1.into_push_pull_output()),
            ld2: UserLed(pins.ld2.into_push_pull_output()),
            ld3: UserLed(pins.ld3.into_push_pull_output()),
        }
    }
}

// - UserLedsGeneric ----------------------------------------------------------

pub struct UserLedsGeneric<LD1, LD2, LD3> {
    pub ld1: UserLed<LD1>,
    pub ld2: UserLed<LD2>,
    pub ld3: UserLed<LD3>,
}

impl<LD1, LD2, LD3> UserLedsGeneric<LD1, LD2, LD3>
where
    LD1: OutputPin,
    LD2: OutputPin,
    LD3: OutputPin,
{
    fn new(pin1: LD1, pin2: LD2, pin3: LD3) -> Self {
        Self {
            ld1: UserLed(pin1),
            ld2: UserLed(pin2),
            ld3: UserLed(pin3),
        }
    }

    pub fn new2(pins: user_leds::Pins) -> user_leds::Type {
        UserLedsGeneric::new(
            pins.ld1.into_push_pull_output(),
            pins.ld2.into_push_pull_output(),
            pins.ld3.into_push_pull_output(),
        )
    }
}

pub fn new(pins: user_leds::Pins) -> user_leds::Type {
    UserLedsGeneric::new(
        pins.ld1.into_push_pull_output(),
        pins.ld2.into_push_pull_output(),
        pins.ld3.into_push_pull_output(),
    )
}
