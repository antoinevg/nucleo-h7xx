#![no_main]
#![no_std]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use cortex_m_rt::entry;
use panic_semihosting as _;

use nucleo::led::Led;
use nucleo_h7xx as nucleo;

use hal::gpio::ExtiPin;
use hal::interrupt;
use hal::prelude::*;
use nucleo::hal;

use nucleo::pac;

// - global static state ------------------------------------------------------

static USER_BUTTON: Mutex<RefCell<Option<hal::gpio::gpioc::PC13<hal::gpio::Input>>>> =
    Mutex::new(RefCell::new(None));
static USER_LEDS: Mutex<RefCell<Option<nucleo::led::UserLeds>>> = Mutex::new(RefCell::new(None));

// - entry-point --------------------------------------------------------------

#[entry]
fn main() -> ! {
    // - board setup ----------------------------------------------------------

    let board = nucleo::Board::take().unwrap();

    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = pac::Peripherals::take().unwrap();

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

    let mut user_button = pins.user_button.into_floating_input();
    let user_leds = nucleo::led::UserLeds::new(pins.user_leds);

    // - configure interrupts -------------------------------------------------

    user_button.make_interrupt_source(&mut dp.SYSCFG);
    user_button.trigger_on_edge(&mut dp.EXTI, hal::gpio::Edge::RisingFalling);
    user_button.enable_interrupt(&mut dp.EXTI);

    cortex_m::interrupt::free(|cs| {
        USER_BUTTON.borrow(cs).replace(Some(user_button));
        USER_LEDS.borrow(cs).replace(Some(user_leds));
    });

    unsafe {
        cp.NVIC.set_priority(interrupt::EXTI15_10, 1);
        cortex_m::peripheral::NVIC::unmask(interrupt::EXTI15_10);
    }

    // - main loop ------------------------------------------------------------

    loop {
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn EXTI15_10() {
    cortex_m::interrupt::free(|cs| {
        if let (Some(user_button), Some(user_leds)) = (
            USER_BUTTON.borrow(cs).borrow_mut().as_mut(),
            USER_LEDS.borrow(cs).borrow_mut().as_mut(),
        ) {
            user_button.clear_interrupt_pending_bit();

            if user_button.is_high() {
                user_leds.ld1.on();
                user_leds.ld2.on();
                user_leds.ld3.on();
            } else {
                user_leds.ld1.off();
                user_leds.ld2.off();
                user_leds.ld3.off();
            }
        }
    });
}
