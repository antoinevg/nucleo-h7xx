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
use hal::gpio::Speed::VeryHigh;


// - modules ------------------------------------------------------------------

pub mod clocks;
pub use clocks::configure as configure_clocks;
pub mod led;
pub mod ethernet;
pub mod pin;
pub mod usart;

// TODO proper logging with compile-time feature selection of: semihosting/itm/rtt
//pub mod itm;
//#[macro_use]
//pub mod itm_macros;
use panic_semihosting as _;


// - global static state ------------------------------------------------------

// `no_mangle` is used here to prevent linking different minor
// versions of this crate as that would let you `take` the core
// peripherals more than once (one per minor version)
#[no_mangle]
static NUCLEO_BOARD: () = ();

/// Global static to hold our Board singleton so it's possible to steal it
static mut BOARD: Option<Board> = None;


// - Board --------------------------------------------------------------------

#[allow(non_snake_case)]
pub struct Board<'a> {
    pub clocks: hal::rcc::CoreClocks,
    pub peripheral: hal::rcc::rec::PeripheralREC,
    pub user_leds: led::UserLeds,
    pub pins: pin::Pins,
    pub USART1: usart::Interface<'a>,

    _marker: core::marker::PhantomData<&'a ()>,
}

impl<'a> Board<'a> {
    /// Returns the nucleo board *once*
    #[inline]
    pub fn take() -> Option<&'static mut Self> {
        cortex_m::interrupt::free(|_| {
            if unsafe { BOARD.is_none() } {
                let board = Self::new(
                    pac::CorePeripherals::take()?,
                    pac::Peripherals::take()?
                );
                unsafe { BOARD = Some(board) };
                Some(unsafe { Board::steal() })

            } else {
                None
            }
        })
    }

    /// Unchecked version of `Board::take`
    pub unsafe fn steal() -> &'static mut Self {
        BOARD.as_mut().expect("Board has not been initialized")
    }

    fn new(_cp: pac::CorePeripherals, dp: pac::Peripherals) -> Board<'a> {
        let rcc = dp.RCC.constrain();
        let ccdr_peripheral = unsafe { rcc.steal_peripheral_rec() };

        let ccdr: hal::rcc::Ccdr = clocks::configure(dp.PWR.constrain(),
                                                     rcc,
                                                     &dp.SYSCFG);

        let gpioa: gpio::gpioa::Parts = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob: gpio::gpiob::Parts = dp.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioc: gpio::gpioc::Parts = dp.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiod: gpio::gpiod::Parts = dp.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioe: gpio::gpioe::Parts = dp.GPIOE.split(ccdr.peripheral.GPIOE);
        let gpiog: gpio::gpiog::Parts = dp.GPIOG.split(ccdr.peripheral.GPIOG);

        let usart1_pins = (
            gpiob.pb6.into_alternate_af7(),    // USART1 TX
            gpiob.pb7.into_alternate_af7(),    // USART1 RX
        );
        let usart1_interface = usart::Interface::init(&ccdr.clocks,
                                                      ccdr.peripheral.USART1,
                                                      usart1_pins).unwrap();

        Self {
            clocks: ccdr.clocks,
            peripheral: ccdr_peripheral,
            user_leds: led::UserLeds::new(led::Pins {
                ld_1: gpiob.pb0.into_push_pull_output(), // TODO feature
                // ld_1: gpioa.pa5.into_push_pull_output(), // TODO feature
                ld_2: gpioe.pe1.into_push_pull_output(),
                ld_3: gpiob.pb14.into_push_pull_output(),
            }),
            pins: pin::Pins {
                //D0: gpiob.pb7,
                D16: gpioc.pc6,
                ethernet: ethernet::Pins {
                    ref_clk: gpioa.pa1.into_alternate_af11().set_speed(VeryHigh),
                    md_io:   gpioa.pa2.into_alternate_af11().set_speed(VeryHigh),
                    md_clk:  gpioc.pc1.into_alternate_af11().set_speed(VeryHigh),
                    crs:     gpioa.pa7.into_alternate_af11().set_speed(VeryHigh),
                    rx_d0:   gpioc.pc4.into_alternate_af11().set_speed(VeryHigh),
                    rx_d1:   gpioc.pc5.into_alternate_af11().set_speed(VeryHigh),
                    tx_en:   gpiog.pg11.into_alternate_af11().set_speed(VeryHigh),
                    tx_d0:   gpiog.pg13.into_alternate_af11().set_speed(VeryHigh),
                    tx_d1:   gpiob.pb13.into_alternate_af11().set_speed(VeryHigh),
                },
                // TODO
            },
            USART1: usart1_interface,

            _marker: core::marker::PhantomData
        }
    }
}
