#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;

use nucleo_h7xx::hal::pac;


#[entry]
fn main() -> ! {
    // - configuration --------------------------------------------------------

    let dp = pac::Peripherals::take().unwrap();

    // enable gpioe peripheral clock - pac
    let rcc = &dp.RCC;
    rcc.ahb4enr.modify(|_, w| w.gpioeen().set_bit());

    // configure user led pin
    let gpioe = &dp.GPIOE;
    gpioe.moder.modify(|_, w| w.moder1().output());
    gpioe.otyper.modify(|_, w| w.ot1().push_pull());
    gpioe.pupdr.modify(|_, w| w.pupdr1().pull_up());
    gpioe.ospeedr.modify(|_, w| w.ospeedr1().high_speed());


    // - main loop ------------------------------------------------------------

    loop {
        gpioe.odr.modify(|_, w| w.odr1().high());
        cortex_m::asm::delay(32_000_000);

        gpioe.odr.modify(|_, w| w.odr1().low());
        cortex_m::asm::delay(32_000_000);
    }
}
