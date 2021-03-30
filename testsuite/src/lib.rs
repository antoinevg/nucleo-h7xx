#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};

use panic_probe as _;         // panic handler
use defmt_rtt as _;           // global logger
use nucleo_h7xx as nucleo;    // bsp
use nucleo::hal;              // hal

use hal::prelude::*;


// - panic handler ------------------------------------------------------------

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}


// - defmt::timestamp ---------------------------------------------------------

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});


// - shared state -------------------------------------------------------------

pub struct State {
    pub flag:   bool,
    pub board:  nucleo::Board,
    pub clocks: hal::rcc::CoreClocks,
    pub pins:   nucleo::pins::Pins,
}

#[allow(unused_mut)]
impl State {
    pub fn init() -> State {
        let mut board = defmt::unwrap!(nucleo::Board::take());

        let mut board = self::init(board);

        let dp = defmt::unwrap!(nucleo::pac::Peripherals::take());

        let ccdr = board.freeze_clocks(dp.PWR.constrain(),
                                       dp.RCC.constrain(),
                                       &dp.SYSCFG);

        let pins = board.split_gpios(dp.GPIOA.split(ccdr.peripheral.GPIOA),
                                     dp.GPIOB.split(ccdr.peripheral.GPIOB),
                                     dp.GPIOC.split(ccdr.peripheral.GPIOC),
                                     dp.GPIOD.split(ccdr.peripheral.GPIOD),
                                     dp.GPIOE.split(ccdr.peripheral.GPIOE),
                                     dp.GPIOF.split(ccdr.peripheral.GPIOF),
                                     dp.GPIOG.split(ccdr.peripheral.GPIOG));

        State {
            flag: true,
            board: board,
            clocks: ccdr.clocks,
            pins: pins,
        }
    }
}


// - board initialization -----------------------------------------------------

#[allow(unused_mut)]
pub fn init(mut board: nucleo::Board) -> nucleo::Board {
    defmt::debug!("initializing board");
    board
}
