#![no_std]

use panic_probe as _;         // panic handler
use defmt_rtt as _;           // global logger
use nucleo_h7xx as nucleo;    // bsp


// - panic handler ------------------------------------------------------------

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}


// - board initialization -----------------------------------------------------

pub fn init<'a>(board: &'a mut nucleo::Board<'a>) -> &'a mut nucleo::Board<'a> {
    defmt::debug!("initializing board");
    board
}
