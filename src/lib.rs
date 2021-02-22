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
pub mod timer;
pub mod usart;

// TODO proper logging with compile-time feature selection of: semihosting/itm/rtt
pub mod itm;


// - global static state ------------------------------------------------------

// `no_mangle` is used here to prevent linking different minor
// versions of this crate as that would let you `take` the core
// peripherals more than once (one per minor version)
#[no_mangle]
static NUCLEO_BOARD: () = ();

/// Global static to hold our Board singleton so it's possible to steal it
static mut BOARD: Option<Board> = None;


// - types --------------------------------------------------------------------

#[allow(non_snake_case)]
pub struct DevicePeripherals {
    pub TIM1: hal::device::TIM1,
    pub TIM2: hal::device::TIM2,
    pub TIM4: hal::device::TIM4,
    pub TIM6: hal::device::TIM6,
    pub TIM12: hal::device::TIM12,
    pub TIM13: hal::device::TIM13,
    pub TIM14: hal::device::TIM14,
    pub TIM15: hal::device::TIM15,
    pub TIM16: hal::device::TIM16,
    pub TIM17: hal::device::TIM17,
    // TODO
}


// - Board --------------------------------------------------------------------

#[allow(non_snake_case)]
pub struct Board<'a> {
    pub clocks: hal::rcc::CoreClocks,
    pub ccdr_peripheral: Option<hal::rcc::rec::PeripheralREC>,
    pub user_leds: led::UserLeds,
    pub pins: Option<pin::Pins>,
    pub device_peripherals: Option<DevicePeripherals>,
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

    fn new(mut cp: pac::CorePeripherals, dp: pac::Peripherals) -> Board<'a> {
        // link SRAM3 power state to CPU1
        dp.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

        let rcc = dp.RCC.constrain();
        let ccdr_peripheral = unsafe { rcc.steal_peripheral_rec() };

        // configure clocks
        let ccdr: hal::rcc::Ccdr = clocks::configure(dp.PWR.constrain(),
                                                     rcc,
                                                     &dp.SYSCFG);

        // TODO feature-gate to enable itm

        // configure cpu
        cp.SCB.invalidate_icache();
        cp.SCB.enable_icache();
        cp.DWT.enable_cycle_counter();

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
            ccdr_peripheral: Some(ccdr_peripheral),
            user_leds: led::UserLeds::new(led::Pins {
                ld_1: gpiob.pb0.into_push_pull_output(), // TODO feature
                // ld_1: gpioa.pa5.into_push_pull_output(), // TODO feature
                ld_2: gpioe.pe1.into_push_pull_output(),
                ld_3: gpiob.pb14.into_push_pull_output(),
            }),
            pins: Some(pin::Pins {
                D51: gpiod.pd7,
                D52: gpiod.pd6,
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
            }),
            USART1: usart1_interface,

            device_peripherals: Some(DevicePeripherals {
                TIM1: dp.TIM1,
                TIM2: dp.TIM2,
                TIM4: dp.TIM4,
                TIM6: dp.TIM6,
                TIM12: dp.TIM12,
                TIM13: dp.TIM13,
                TIM14: dp.TIM14,
                TIM15: dp.TIM15,
                TIM16: dp.TIM16,
                TIM17: dp.TIM17,
            }),

            _marker: core::marker::PhantomData
        }
    }

    pub unsafe fn take_pins(&mut self) -> pin::Pins {
        let p = core::ptr::replace(&mut self.pins, None);
        p.unwrap()
    }

    pub unsafe fn take_ccdr_peripheral(&mut self) -> hal::rcc::rec::PeripheralREC {
        let p = core::ptr::replace(&mut self.ccdr_peripheral, None);
        p.unwrap()
    }

    pub unsafe fn take_device_peripherals(&mut self) -> DevicePeripherals {
        let p = core::ptr::replace(&mut self.device_peripherals, None);
        p.unwrap()
    }
}
