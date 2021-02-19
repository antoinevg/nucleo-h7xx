#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_semihosting::hprintln;

use cortex_m_rt::entry;

use stm32h7xx_hal as hal;
use hal::prelude::*;
use hal::pac;

use nucleo_h745zi as nucleo;

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

    let timer = dp.TIM2.timer(u32::hz(100), ccdr.peripheral.TIM2, &ccdr.clocks);
    let mut timer = nucleo::timer::CountDownTimer::new(timer);


    // - test timeout  --------------------------------------------------------

    fn test(timeout: usize) -> bool {
        static mut COUNTER: usize = 0;
        if timeout == 0 {
            unsafe { COUNTER = 0 };
            return true;
        }
        unsafe { COUNTER += 1; }
        if unsafe { COUNTER } <= timeout {
            return false;
        }
        return true;
    }

    test(0);
    timer.start(1500.ms());
    let mut timed_out = true;
    repeat_timeout!(
        &mut timer,
        {
            Ok(test(5))
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
    assert_eq!(timed_out, false, "Should not have timed out");

    test(0);
    timer.start(1500.ms());
    let mut timed_out = true;
    repeat_timeout!(
        &mut timer,
        {
            Ok(test(15))
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
    assert_eq!(timed_out, true, "Should have timed out");

    test(0);
    timer.start(1500.ms());
    let result: Result<(), TimeoutError<()>> = block_timeout!(
        &mut timer,
        {
            let is_up = test(5);
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
    match result {
        Err(TimeoutError::Timeout) => panic!("Should not have timed out"),
        _ => (),
    }

    test(0);
    timer.start(1500.ms());
    let result: Result<(), TimeoutError<()>> = block_timeout!(
        &mut timer,
        {
            let is_up = test(15);
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
    match result {
        Err(TimeoutError::Timeout) => (),
        _ => panic!("Should have timed out")
    }


    // - main loop ------------------------------------------------------------

    loop {
    }
}
