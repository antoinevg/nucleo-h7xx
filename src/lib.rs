#![deny(warnings)]

//! Board support crate for STMicroElectronics STM32H7 Nucleo-144 boards
//!
//! # Usage - see examples/
//! ```

#![no_std]

// - modules ------------------------------------------------------------------

pub mod board;
pub mod clocks;
#[cfg(any(feature = "ethernet"))]
pub mod ethernet;
pub mod led;
pub mod pins;
pub mod timer;

#[cfg(any(feature = "log-itm"))]
pub mod itm;

// - log macros ---------------------------------------------------------------

#[macro_export]
macro_rules! loggit {
    ($($arg:tt)*) => (
        #[cfg(feature = "log-itm")]
        {
            let itm = unsafe { &mut *cortex_m::peripheral::ITM::PTR };
            cortex_m::iprintln!(&mut itm.stim[0], $($arg)*);
        }
        #[cfg(feature = "log-semihosting")]
        cortex_m_semihosting::hprintln!($($arg)*);
    )
}

// - exports ------------------------------------------------------------------

pub use hal::hal as embedded_hal;
pub use hal::pac;
pub use stm32h7xx_hal as hal;

pub use board::Board;
