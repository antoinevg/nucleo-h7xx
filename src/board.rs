use stm32h7xx_hal as hal;

use crate::clocks;
use crate::pins;


// - global static state ------------------------------------------------------

// `no_mangle` is used here to prevent linking different minor
// versions of this crate as that would let you `take` the core
// peripherals more than once (one per minor version)
#[no_mangle]
static NUCLEO_BOARD: () = ();

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
                       _gpiof: hal::gpio::gpiof::Parts,
                       gpiog: hal::gpio::gpiog::Parts,
                       _gpioh: hal::gpio::gpioh::Parts,
                       _gpioi: hal::gpio::gpioi::Parts,
                       _gpioj: hal::gpio::gpioj::Parts,
                       _gpiok: hal::gpio::gpiok::Parts) -> pins::Pins {

        pins::Pins {
            d51: gpiod.pd7,
            d52: gpiod.pd6,

            ethernet: pins::ethernet::Pins {
                ref_clk: gpioa.pa1,
                md_io:   gpioa.pa2,
                md_clk:  gpioc.pc1,
                crs:     gpioa.pa7,
                rx_d0:   gpioc.pc4,
                rx_d1:   gpioc.pc5,
                tx_en:   gpiog.pg11,
                tx_d0:   gpiog.pg13,
                tx_d1:   gpiob.pb13,
            },

            user_leds: pins::user_leds::Pins {
                #[cfg(not(feature = "led-1-pa5"))] ld1: gpiob.pb0,
                #[cfg(any(feature = "led-1-pa5"))] ld1: gpioa.pa5,
                ld2: gpioe.pe1,
                ld3: gpiob.pb14
            },

        }
    }

    /*pub fn split_led_user(&self, pin: LedUserPin) -> led::LedUser {
        led::LedUser::new(pin)
    }*/
}
