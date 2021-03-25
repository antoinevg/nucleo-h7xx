#![deny(warnings)]

//! Board support crate for nucleo-h745zi hardware
//!
//! # Usage - see examples/
//! ```

#![no_std]


// - modules ------------------------------------------------------------------

pub mod board;
pub mod clocks;
pub mod led;
pub mod ethernet;
pub mod pins;
pub mod timer;

#[cfg(any(feature = "log-itm"))]
pub mod itm;

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
