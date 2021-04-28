#![deny(warnings)]

//! Board support crate for STMicroElectronics STM32H7 Nucleo-144 boards
//!
//! # Usage - see examples/
//! ```

#![no_std]


// - modules ------------------------------------------------------------------

pub mod board;
pub mod clocks;
pub mod led;
#[cfg(any(feature = "ethernet"))]
pub mod ethernet;
pub mod pins;
pub mod timer;

#[cfg(any(feature = "log-itm"))]
pub mod itm;


// - log macros ---------------------------------------------------------------

#[cfg(any(feature = "log-itm"))]
#[macro_export]
macro_rules! loggit {
    ($($arg:tt)*) => (
        let itm = unsafe { &mut *cortex_m::peripheral::ITM::ptr() };
        cortex_m::iprintln!(&mut itm.stim[0], $($arg)*);
    )
}

#[cfg(not(feature = "log-itm"))]
#[macro_export]
macro_rules! loggit {
    ($($arg:tt)*) => (
        cortex_m_semihosting::hprintln!($($arg)*).unwrap();
    )
}


// - exports ------------------------------------------------------------------

pub use stm32h7xx_hal as hal;
pub use hal::hal as embedded_hal;
pub use hal::pac;

pub use board::Board;
