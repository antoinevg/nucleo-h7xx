#![no_std]

use panic_probe as _;         // panic handler
use defmt_rtt as _;           // global logger
use nucleo_h745zi as nucleo;  // bsp

use nucleo::pac;


// - panic handler ------------------------------------------------------------

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}


// - global initialization ----------------------------------------------------

pub fn init(_dp: pac::Peripherals) -> () {
    defmt::debug!("initializing board");
}
