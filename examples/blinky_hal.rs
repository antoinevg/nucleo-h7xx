#![no_main]
#![no_std]

use panic_semihosting as _;

use nucleo_h745zi::hal;
use hal::{pac, prelude::*};
use hal::rcc::PllConfigStrategy;

use hal::hal as embedded_hal;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};


#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // - power & clocks -------------------------------------------------------

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.smps().vos0(&dp.SYSCFG).freeze();
    let ccdr = dp.RCC.constrain()
        .pll1_strategy(PllConfigStrategy::Iterative)  // pll1 drives system clock
        .sys_ck(480.mhz())                            // system clock @ 480 MHz
        .freeze(pwrcfg, &dp.SYSCFG);

    // - pins -----------------------------------------------------------------

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let mut led_user = gpiob.pb14.into_push_pull_output();
    led_user.set_low().unwrap();

    // - main loop ------------------------------------------------------------

    loop {
        loop {
            led_user.toggle().unwrap();
            cortex_m::asm::delay(480_000_000);
        }
    }
}
