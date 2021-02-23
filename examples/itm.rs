#![no_main]
#![no_std]

use cortex_m_rt::entry;

use cortex_m::iprintln;

use nucleo_h745zi as nucleo;
use nucleo::loggit;
use nucleo::led::Led;

use log::{debug, error, info};
mod utilities;


#[entry]
fn main() -> ! {
    // - board setup ----------------------------------------------------------

    let board = nucleo::Board::take().unwrap();
    utilities::logger::init();

    // - test iprintln! -------------------------------------------------------

    let itm = unsafe { &mut *cortex_m::peripheral::ITM::ptr() };
    iprintln!(&mut itm.stim[0], "Hello itm example!");

    info!("Hello itm example!");
    debug!("Hello itm example!");
    error!("Hello itm example!");


    // - leds -----------------------------------------------------------------

    let user_leds = &mut board.user_leds;


    // - main loop ------------------------------------------------------------

    let one_second = board.clocks.sys_ck().0;
    let mut counter = 0;

    loop {
        loggit!("{}: one", counter);
        user_leds.LD3.off();
        user_leds.LD1.on();
        cortex_m::asm::delay(one_second);

        loggit!("{}: two", counter);
        user_leds.LD1.off();
        user_leds.LD2.on();
        cortex_m::asm::delay(one_second);

        loggit!("{}: three", counter);
        user_leds.LD2.off();
        user_leds.LD3.on();
        cortex_m::asm::delay(one_second);

        counter += 1;
    }
}
