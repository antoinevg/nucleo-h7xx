use stm32h7xx_hal as hal;

use crate::clocks;
use crate::led;
use crate::pins;


// - global static state ------------------------------------------------------

// `no_mangle` is used here to prevent linking different minor
// versions of this crate as that would let you `take` the core
// peripherals more than once (one per minor version)
#[no_mangle]
static NUCLEO_H745_BOARD: () = ();

/// Set to `true` when `take` was called to make `Board` a singleton.
static mut TAKEN: bool = false;


// - Board --------------------------------------------------------------------

pub struct Board;

impl Board {
    #[inline]
    pub fn take() -> Option<Self> {
        cortex_m::interrupt::free(|_| {
            if unsafe { TAKEN } {
                None
            } else {
                Some(unsafe { Board::steal() })
            }
        })
    }

    #[inline]
    pub unsafe fn steal() -> Self {
        Board
    }

    pub fn freeze_clocks(&self,
                         pwr: hal::pwr::Pwr,
                         rcc: hal::rcc::Rcc,
                         syscfg: &hal::device::SYSCFG) -> hal::rcc::Ccdr {
        clocks::configure(pwr, rcc, syscfg)
    }

    /// Takes the board's GPIO peripherals and split them into ZST's
    /// representing the individual GPIO pins used by the board.
    pub fn split_gpios(&self,
                       gpioa: hal::gpio::gpioa::Parts,
                       gpiob: hal::gpio::gpiob::Parts,
                       gpioc: hal::gpio::gpioc::Parts,
                       gpiod: hal::gpio::gpiod::Parts,
                       gpioe: hal::gpio::gpioe::Parts,
                       gpiof: hal::gpio::gpiof::Parts,
                       gpiog: hal::gpio::gpiog::Parts,
                       gpioh: hal::gpio::gpioh::Parts,
                       gpioi: hal::gpio::gpioi::Parts,
                       gpioj: hal::gpio::gpioj::Parts,
                       gpiok: hal::gpio::gpiok::Parts) -> pins::Pins {
        pins::Pins::new(gpioa, gpiob, gpioc, gpiod, gpioe, gpiof, gpiog, gpioh, gpioi, gpioj, gpiok)
    }

    pub fn split_led_user(&self, pins: pins::user_leds::Pins) -> led::UserLedsAlt {
        led::UserLedsAlt::new(pins)
    }
}
