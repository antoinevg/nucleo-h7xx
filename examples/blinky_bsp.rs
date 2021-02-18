#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;

use nucleo_h745zi as nucleo;
use nucleo::led::Led;


#[entry]
fn main() -> ! {
    // - board setup ----------------------------------------------------------

    let board = nucleo::Board::take().unwrap();
    let user_leds = &mut board.user_leds;


    // - main loop ------------------------------------------------------------

    let one_second = board.clocks.sys_ck().0;

    loop {
        user_leds.LD3.off();
        user_leds.LD1.on();
        cortex_m::asm::delay(one_second);

        user_leds.LD1.off();
        user_leds.LD2.on();
        cortex_m::asm::delay(one_second);

        user_leds.LD2.off();
        user_leds.LD3.on();
        cortex_m::asm::delay(one_second);
    }
}
