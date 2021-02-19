#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]

#![no_main]
#![no_std]


use panic_semihosting as _;
use cortex_m_semihosting::hprintln;

use cortex_m_rt::entry;
use cortex_m;

use stm32h7xx_hal as hal;
use hal::prelude::*;
use hal::gpio::Speed::*;
use hal::hal::digital::v2::OutputPin;
use hal::hal::digital::v2::ToggleableOutputPin;
use hal::rcc::CoreClocks;

use hal::pac;
use pac::interrupt;

use embedded_timeout_macros::{
    block_timeout,
    repeat_timeout,
    TimeoutError,
};


// - types --------------------------------------------------------------------

#[derive(Debug)]
pub enum Error {
    Unknown
}


// - entry point --------------------------------------------------------------

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = pac::CorePeripherals::take().unwrap();


    // - power & clocks -------------------------------------------------------

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.smps().vos0(&dp.SYSCFG).freeze();

    // link SRAM3 power state to CPU1
    dp.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .pll1_strategy(hal::rcc::PllConfigStrategy::Iterative)  // pll1 drives system clock
        .sys_ck(480.mhz())
        .freeze(pwrcfg, &dp.SYSCFG);


    // - timers ---------------------------------------------------------------

    //let mut timer = dp.TIM2.timer(u32::hz(10_000), ccdr.peripheral.TIM2, &ccdr.clocks);
    let mut timer = hal::timer::Timer::tim2(dp.TIM2, ccdr.peripheral.TIM2, &ccdr.clocks);


    // - test timeout  --------------------------------------------------------

    static mut COUNTER: usize = 0;

    fn test() -> bool {
        unsafe { COUNTER += 1; }
        if unsafe { COUNTER } <= 5 {
            return false;
        }
        return true;
    }

    hprintln!("start").unwrap();
    timer.start(1000.ms());
    let mut timed_out = true;
    let result = repeat_timeout!(
        &mut timer,
        {
            Ok(test())
        },
        (result) {
            let result: bool = result;
            if result {
                hprintln!("up").unwrap();
                timed_out = false;
                break;
            } else {
                hprintln!("down").unwrap();
                continue;
            }
        };
        (error) {
            let error: Error = error;
            hprintln!("Error: {:?}", error).unwrap();
        };
    );
    hprintln!("timed_out: {}", timed_out).unwrap();

    unsafe { COUNTER = 0; }
    timer.start(1000.ms());
    let result: Result<(), TimeoutError<()>> = block_timeout!(
        &mut timer,
        {
            let is_up = test();
            if is_up {
                hprintln!("up").unwrap();
                Ok(())
            } else {
                hprintln!("down").unwrap();
                Err(nb::Error::WouldBlock)
            }
        }
    );
    hprintln!("result: {:?}", result).unwrap();

    // - main loop ------------------------------------------------------------

    loop {
    }
}
