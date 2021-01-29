//! Board support crate for nucleo-h745zi hardware
//!
//! # Usage - see examples/
//! ```

//#![deny(warnings)]
#![allow(unused_variables)]

#![no_std]

pub use stm32h7xx_hal as hal;
pub use hal::hal as embedded_hal;
pub use hal::pac;

use hal::prelude::*;
use hal::gpio;


// - modules ------------------------------------------------------------------

pub mod clocks;
pub use clocks::configure as configure_clocks;
pub mod led;
pub mod pin;
pub mod usart;

// TODO proper logging with compile-time feature selection of: semihosting/itm/rtt
//pub mod itm;
//#[macro_use]
//pub mod itm_macros;
use panic_semihosting as _;
//use cortex_m_rt::entry;


// - Board --------------------------------------------------------------------

#[allow(non_snake_case)]
pub struct Board<'a> {
    pub clocks: hal::rcc::CoreClocks,
    pub pins: pin::Pins,
    pub leds: led::Leds,

    pub USART1: usart::Interface<'a>,

    _marker: core::marker::PhantomData<&'a *const ()>,
}


// NOTE `no_mangle` is used here to prevent linking different minor versions of this crate as that
// would let you `take` the core peripherals more than once (one per minor version)
#[no_mangle]
static NUCLEO_BOARD: () = ();

/// Set to `true` when `take` or `steal` was called to make `Board` a singleton.
static mut TAKEN: bool = false;


impl<'a> Board<'a> {
    /// Returns the nucleo board *once*
    #[inline]
    pub fn take()  -> Option<Self> {
        cortex_m::interrupt::free(|_| {
            if unsafe { TAKEN } {
                None
            } else {
                unsafe { TAKEN = true; }
                Some(Self::new(pac::CorePeripherals::take()?,
                               pac::Peripherals::take()?))
            }
        })
    }

    /// Unchecked version of `Board::take`
    #[inline]
    pub unsafe fn steal() -> Self {
        Self::new(pac::CorePeripherals::steal(),
                  pac::Peripherals::steal())
    }

    fn new(_cp: pac::CorePeripherals, dp: pac::Peripherals) -> Board<'a> {
        let ccdr: hal::rcc::Ccdr = clocks::configure(dp.PWR.constrain(),
                                                     dp.RCC.constrain(),
                                                     &dp.SYSCFG);

        let gpioa: gpio::gpioa::Parts = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob: gpio::gpiob::Parts = dp.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioc: gpio::gpioc::Parts = dp.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiod: gpio::gpiod::Parts = dp.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioe: gpio::gpioe::Parts = dp.GPIOE.split(ccdr.peripheral.GPIOE);
        let gpiog: gpio::gpiog::Parts = dp.GPIOG.split(ccdr.peripheral.GPIOG);

        let usart1_pins = (
            gpiob.pb14.into_alternate_af4(),    // USART1 TX - GPIO29 - Pin 36 <= GPIOB 14
            gpiob.pb15.into_alternate_af4(),    // USART1 RX - GPIO30 - Pin 37 => GPIOB 15
        );
        let usart1_interface = usart::Interface::init(&ccdr.clocks,
                                                      ccdr.peripheral.USART1,
                                                      usart1_pins).unwrap();

        Self {
            clocks: ccdr.clocks,
            leds: led::Leds {
                USER: led::LedUser::new(gpioc.pc7)
            },
            pins: pin::Pins {
                D0: gpiob.pb7,
                D16: gpioc.pc6,
                // TODO
            },
            USART1: usart1_interface,

            _marker: core::marker::PhantomData
        }
    }
}
