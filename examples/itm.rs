#![no_main]
#![no_std]

use cortex_m::iprintln;
use cortex_m_rt::entry;

use nucleo_h7xx as nucleo;

use hal::prelude::*;
use nucleo::hal;

use nucleo::led::Led;
use nucleo::loggit;

use log::{debug, error, info};
mod utilities;

#[entry]
fn main() -> ! {
    // - board setup ----------------------------------------------------------

    let board = nucleo::Board::take().unwrap();

    let dp = nucleo::pac::Peripherals::take().unwrap();

    let ccdr = board.freeze_clocks(dp.PWR.constrain(), dp.RCC.constrain(), &dp.SYSCFG);

    let pins = board.split_gpios(
        dp.GPIOA.split(ccdr.peripheral.GPIOA),
        dp.GPIOB.split(ccdr.peripheral.GPIOB),
        dp.GPIOC.split(ccdr.peripheral.GPIOC),
        dp.GPIOD.split(ccdr.peripheral.GPIOD),
        dp.GPIOE.split(ccdr.peripheral.GPIOE),
        dp.GPIOF.split(ccdr.peripheral.GPIOF),
        dp.GPIOG.split(ccdr.peripheral.GPIOG),
    );

    // - logger setup ---------------------------------------------------------

    utilities::logger::init();

    let itm = unsafe { &mut *cortex_m::peripheral::ITM::PTR };
    iprintln!(&mut itm.stim[0], "Hello itm example!");

    info!("Hello itm example!");
    debug!("Hello itm example!");
    error!("Hello itm example!");

    // - leds -----------------------------------------------------------------

    let mut user_leds = nucleo::led::UserLeds::new(pins.user_leds);

    // - main loop ------------------------------------------------------------

    let one_second = ccdr.clocks.sys_ck().raw();
    let mut counter = 0;

    loop {
        loggit!("{}: one", counter);
        user_leds.ld3.off();
        user_leds.ld1.on();
        cortex_m::asm::delay(one_second);

        loggit!("{}: two", counter);
        user_leds.ld1.off();
        user_leds.ld2.on();
        cortex_m::asm::delay(one_second);

        loggit!("{}: three", counter);
        user_leds.ld2.off();
        user_leds.ld3.on();
        cortex_m::asm::delay(one_second);

        counter += 1;
    }
}
