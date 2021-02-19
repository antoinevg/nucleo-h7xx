//! The stm32h7xx-hal implementation of the CountDown trait uses Hertz
//! for the Time type with the result that it's duration is capped to
//! 1000ms.
//!
//! This alternate implementation uses Duration instead, allowing for
//! much longer periods.

use stm32h7xx_hal as hal;
use hal::hal as embedded_hal;
use hal::pac::{
    TIM1, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17, TIM2, TIM3, TIM4, TIM5,
    TIM6, TIM7, TIM8,
};


// - CountDownTimer -----------------------------------------------------------

pub struct CountDownTimer<TIMX> {
    timer: hal::timer::Timer::<TIMX>,
}

impl<TIMX> CountDownTimer<TIMX> {
    pub fn new(timer: hal::timer::Timer::<TIMX>) -> Self {
        Self {
            timer: timer
        }
    }
}


// - embedded_hal::timer::CountDown -------------------------------------------

macro_rules! impl_countdown {
    ($($TIMX:ident: ($cntType:ty),)+) => {
        $(
            impl embedded_hal::timer::Periodic for CountDownTimer<$TIMX> {}

            impl embedded_hal::timer::CountDown for CountDownTimer<$TIMX> {
                type Time = core::time::Duration;

                // TODO check for invalid values by using `cntType`
                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<core::time::Duration>,
                {
                    // Pause
                    self.timer.pause();

                    // Reset counter
                    self.timer.reset_counter();

                    // UEV event occours on next overflow
                    self.timer.urs_counter_only();
                    self.timer.clear_irq();

                    // Set PSC and ARR
                    self.timer.set_timeout(timeout);

                    // Generate an update event to force an update of the ARR register. This ensures
                    // the first timer cycle is of the specified duration.
                    self.timer.apply_freq();

                    // Start counter
                    self.timer.resume()
                }

                fn wait(&mut self) -> nb::Result<(), void::Void> {
                    if self.timer.is_irq_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.timer.clear_irq();
                        Ok(())
                    }
                }
            }
        )+
    }
}

impl_countdown! {
    // Advanced-control
    TIM1: (u16),
    TIM8: (u16),

    // General-purpose
    TIM2: (u32),
    TIM3: (u16),
    TIM4: (u16),
    TIM5: (u32),

    // Basic
    TIM6: (u16),
    TIM7: (u16),

    // General-purpose
    TIM12: (u16),
    TIM13: (u16),
    TIM14: (u16),

    // General-purpose
    TIM15: (u16),
    TIM16: (u16),
    TIM17: (u16),
}
