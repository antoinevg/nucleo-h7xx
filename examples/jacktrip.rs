#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_parens)]
#![allow(unused_variables)]

#![no_main]
#![no_std]

use core::sync::atomic::Ordering;

use cortex_m_rt::entry;
use cortex_m;

use stm32h7xx_hal as hal;
use hal::prelude::*;
use hal::gpio::Speed::*;
use hal::hal::digital::v2::OutputPin;
use hal::hal::digital::v2::ToggleableOutputPin;
use hal::rcc::CoreClocks;
use hal::{ethernet, ethernet::PHY};

use hal::pac;
use pac::interrupt;

use nucleo_h745zi as nucleo;
use nucleo::loggit;

use smoltcp;
use smoltcp::iface::{
    EthernetInterface, EthernetInterfaceBuilder, Neighbor, NeighborCache,
    Route, Routes,
};
use smoltcp::socket::{SocketSet, SocketSetItem};
use smoltcp::socket::{UdpSocket, UdpSocketBuffer, UdpPacketMetadata};
use smoltcp::storage::PacketMetadata;
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv6Cidr, IpEndpoint, Ipv4Address};



// - modules ------------------------------------------------------------------

mod driver;
mod dsp;
mod utilities;


// - global constants ---------------------------------------------------------

const MAC_LOCAL: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];
const IP_LOCAL: [u8; 4] = [ 192, 168, 20, 99 ];

const FS: f32 = 48_000.;


// - global static state ------------------------------------------------------

static mut JACKTRIP_INTERFACE: Option<jacktrip::Interface<driver::lan8742a::Socket>> = None;


// - entry point --------------------------------------------------------------

#[entry]
fn main() -> ! {
    match run() {
        Ok(_)  => {},
        Err(e) => {
            loggit!("Fatal error: {:?}", e);
        }
    }
    loop {}
}


fn run() -> Result<(), jacktrip::Error> {
    let board = nucleo::Board::take().unwrap();

    let dp = nucleo::pac::Peripherals::take().unwrap();

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

    utilities::logger::init();

    // - test points ---------------------------------------------------------

    /*let mut tp1 = board_pins.d51.into_push_pull_output().set_speed(VeryHigh);
    let mut tp2 = board_pins.d52.into_push_pull_output().set_speed(VeryHigh);
    tp1.set_low().ok();
    tp2.set_low().ok();*/


    // - ethernet interface ---------------------------------------------------

    let timeout_timer = dp.TIM17.timer(100.hz(), ccdr.peripheral.TIM17, &ccdr.clocks);
    let timeout_timer = nucleo::timer::CountDownTimer::new(timeout_timer);
    let timeout_timer = match nucleo::ethernet::Interface::start(pins.ethernet,
                                                                 &MAC_LOCAL,
                                                                 &IP_LOCAL,
                                                                 ccdr.peripheral.ETH1MAC,
                                                                 &ccdr.clocks,
                                                                 timeout_timer) {
        Ok(tim17) => tim17,
        Err(e) => {
            loggit!("Failed to start ethernet interface: {:?}", e);
            loop {}
        }
    };

    // wait for link to come up
    nucleo::ethernet::Interface::interrupt_free(|ethernet_interface| {
        while !ethernet_interface.poll_link() { }
    });


    // - jacktrip interface ---------------------------------------------------

    //let jacktrip_host = "192.168.20.114"; // chi
    let jacktrip_host = "192.168.20.115"; // zeta
    let jacktrip_port = 4464;
    let mut jacktrip_interface = jacktrip::Interface::<driver::lan8742a::Socket>::new("192.168.20.99", 12345)?;
    match jacktrip_interface.connect(jacktrip_host, jacktrip_port) {
        Ok(()) => (),
        Err(e) => {
            loggit!("Jacktrip interface failed to connect to remote: {}:{}",
                    jacktrip_host, jacktrip_port);
            loop {}
        }
    };
    let num_frames = jacktrip_interface.config.num_frames;
    unsafe {
        JACKTRIP_INTERFACE = Some(jacktrip_interface);
    }
    //loggit!("connected to jacktrip server");


    // - timers ---------------------------------------------------------------

    let dp = unsafe { pac::Peripherals::steal() };
    let mut cp = unsafe { pac::CorePeripherals::steal() };

    nucleo::ethernet::systick_init(&mut cp.SYST, &ccdr.clocks);  // 1ms tick

    // fs / num_frames = 48_000 / 64 = 750 Hz
    let frequency = (FS / num_frames as f32) - 1.;
    //loggit!("Timer frequency: {} / {} = {} Hz", FS, num_frames, frequency);
    let mut timer = dp.TIM2.timer(u32::hz(frequency as u32), ccdr.peripheral.TIM2, &ccdr.clocks);
    timer.listen(hal::timer::Event::TimeOut);
    unsafe {
        cp.NVIC.set_priority(interrupt::TIM2, 1);
        pac::NVIC::unmask(interrupt::TIM2);
    }


    // - main loop ------------------------------------------------------------

    loop {
        let time = nucleo::ethernet::ATOMIC_TIME.load(Ordering::Relaxed);
        nucleo::ethernet::Interface::interrupt_free(|ethernet_interface| {
            /*match ethernet_interface.poll_link() {
                true => led_link.set_high().unwrap(),
                _    => led_link.set_low().unwrap(),
            }*/
            ethernet_interface.poll(time as i64);
        });
        cortex_m::asm::wfi();
        // TODO https://docs.rs/smoltcp/0.7.0/smoltcp/iface/struct.EthernetInterface.html#method.poll_delay
    }
}


// - TIM2 ---------------------------------------------------------------------

#[interrupt]
fn TIM2() {
    static mut COUNT: usize = 0;
    static mut OSC_1: dsp::osc::Wavetable = dsp::osc::Wavetable::new(dsp::osc::Shape::Sin);
    OSC_1.dx = ((1. / FS) * 440.0) as f32;
    const NUM_FRAMES: usize = 128; // TODO

    // hal
    //let mut rc = TIMER.borrow(cs).borrow_mut();
    //let timer = rc.as_mut().unwrap();
    //timer.clear_irq();
    // pac
    let tim2 = unsafe { &mut pac::Peripherals::steal().TIM2 };
    tim2.sr.modify(|_, w| {
        w.uif().clear_bit() // Clears timeout event
    });

    // generate & send audio buffer
    let jacktrip_interface = unsafe { JACKTRIP_INTERFACE.as_mut().unwrap() };

    /*let mut samples: [f32; NUM_FRAMES] = [0.; NUM_FRAMES];
    for sample in samples.iter_mut() {
        *sample = OSC_1.step();
    }
    match jacktrip_interface.send(&samples) } {
        Ok(_bytes_sent) => (),
        Err(e) => loggit!("oops: {:?}", e),
    }*/

    for _ in 0..NUM_FRAMES {
        jacktrip_interface.send(&[
            OSC_1.step() * 0.75,
        ]).unwrap();
    }
}
