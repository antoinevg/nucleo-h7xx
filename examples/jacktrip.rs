#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_parens)]
#![allow(unused_variables)]

#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_semihosting::hprintln;

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


// - global constants ---------------------------------------------------------

const MAC_LOCAL: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];
const IP_LOCAL: [u8; 4] = [ 192, 168, 20, 99 ];

const FS: f32 = 48_000.;


// - global static state ------------------------------------------------------

static mut JACKTRIP_INTERFACE: Option<jacktrip::Interface<driver::lan8742a::Socket>> = None;


// - entry point --------------------------------------------------------------

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut cp = pac::CorePeripherals::take().unwrap();


    // - power & clocks -------------------------------------------------------

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.smps().vos0(&dp.SYSCFG).freeze();
    //let pwrcfg = pwr.smps().freeze();

    // link SRAM3 power state to CPU1
    dp.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .pll1_strategy(hal::rcc::PllConfigStrategy::Iterative)
        .sys_ck(480.mhz())
        .freeze(pwrcfg, &dp.SYSCFG);

    cp.SCB.invalidate_icache();
    cp.SCB.enable_icache();
    cp.DWT.enable_cycle_counter();


    // - gpios ----------------------------------------------------------------

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);


    // - test points ---------------------------------------------------------

    let mut tp1 = gpiod.pd7.into_push_pull_output().set_speed(VeryHigh);
    let mut tp2 = gpiod.pd6.into_push_pull_output().set_speed(VeryHigh);
    tp1.set_low().ok();
    tp2.set_low().ok();


    // - leds ----------------------------------------------------------------

    let mut led_user = gpiob.pb14.into_push_pull_output();  // LED3, red
    let mut led_link = gpioe.pe1.into_push_pull_output();   // LED2, yellow
    led_user.set_low().ok();
    led_link.set_low().ok();


    // - ethernet interface ---------------------------------------------------

    // configure pins for ethernet interface
    let pins = nucleo::ethernet::Pins {
        ref_clk: gpioa.pa1.into_alternate_af11().set_speed(VeryHigh),
        md_io:   gpioa.pa2.into_alternate_af11().set_speed(VeryHigh),
        md_clk:  gpioc.pc1.into_alternate_af11().set_speed(VeryHigh),
        crs:     gpioa.pa7.into_alternate_af11().set_speed(VeryHigh),
        rx_d0:   gpioc.pc4.into_alternate_af11().set_speed(VeryHigh),
        rx_d1:   gpioc.pc5.into_alternate_af11().set_speed(VeryHigh),
        tx_en:   gpiog.pg11.into_alternate_af11().set_speed(VeryHigh),
        tx_d0:   gpiog.pg13.into_alternate_af11().set_speed(VeryHigh),
        tx_d1:   gpiob.pb13.into_alternate_af11().set_speed(VeryHigh),
    };

    // bring up ethernet interface
    match nucleo::ethernet::Interface::start(pins,
                                             &MAC_LOCAL,
                                             &IP_LOCAL,
                                             ccdr.peripheral.ETH1MAC,
                                             &ccdr.clocks) {
        Ok(()) => (),
        Err(e) => {
            hprintln!("Failed to start ethernet interface: {:?}", e).unwrap();
            loop {}
        }
    }

    // wait for link to come up
    nucleo::ethernet::Interface::interrupt_free(|ethernet_interface| {
        while !ethernet_interface.poll_link() { }
    });


    // - jacktrip interface ---------------------------------------------------

    //let jacktrip_host = "192.168.20.114"; // chi
    let jacktrip_host = "192.168.20.115"; // zeta
    let jacktrip_port = 4464;
    let mut jacktrip_interface = jacktrip::Interface::<driver::lan8742a::Socket>::new("192.168.20.99", 12345);
    match jacktrip_interface.connect(jacktrip_host, jacktrip_port) {
        Ok(()) => (),
        Err(e) => {
            hprintln!("Jacktrip interface failed to connect to remote: {}:{}",
                      jacktrip_host, jacktrip_port).unwrap();
            loop {}
        }
    };
    let num_frames = jacktrip_interface.config.num_frames;
    unsafe {
        JACKTRIP_INTERFACE = Some(jacktrip_interface);
    }
    //hprintln!("connected to jacktrip server").unwrap();


    // - timers ---------------------------------------------------------------

    systick_init(&mut cp.SYST, &ccdr.clocks);  // 1ms tick

    // fs / num_frames = 48_000 / 64 = 750 Hz
    let frequency = (FS / num_frames as f32) - 1.;
    //hprintln!("Timer frequency: {} / {} = {} Hz", FS, num_frames, frequency).unwrap();
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


// - systick ------------------------------------------------------------------

// TODO move to nucleo::ethernet

fn systick_init(syst: &mut pac::SYST, clocks: &CoreClocks) {
    let c_ck_mhz = clocks.c_ck().0 / 1_000_000;
    let syst_calib = 0x3E8;
    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload((syst_calib * c_ck_mhz) - 1);
    syst.enable_interrupt();
    syst.enable_counter();
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
        Err(e) => hprintln!("oops: {:?}", e).unwrap(),
    }*/

    for _ in 0..NUM_FRAMES {
        jacktrip_interface.send(&[
            OSC_1.step() * 0.75,
        ]).unwrap();
    }
}
