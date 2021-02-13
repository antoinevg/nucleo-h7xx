#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]

#![no_main]
#![no_std]

/// Simple ethernet example that will respond to icmp pings on
/// `IP_LOCAL` and perioducally send a udp packet to
/// `IP_REMOTE:IP_REMOTE_PORT`
///
/// Also see: https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/ethernet-rtic-stm32h747i-disco.rs
///           https://github.com/adamgreig/stm32f4-smoltcp-demo

use panic_semihosting as _;
use cortex_m_semihosting::hprintln;

use cortex_m_rt::entry;
use cortex_m;

use stm32h7xx_hal as hal;
use hal::prelude::*;
use hal::gpio::Speed::*;
use hal::hal::digital::v2::OutputPin;
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


// - global constants ---------------------------------------------------------

const MAC_LOCAL: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];
const IP_LOCAL: [u8; 4] = [ 192, 168, 20, 99 ];
const IP_REMOTE: [u8; 4] = [ 192, 168, 20, 114 ];
const IP_REMOTE_PORT: u16 = 34254;


// - entry point --------------------------------------------------------------

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut cp = pac::CorePeripherals::take().unwrap();


    // - power & clocks -------------------------------------------------------

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.smps().freeze();

    // link SRAM3 power state to CPU1
    dp.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

    let rcc = dp.RCC.constrain();
    let ccdr_peripheral: hal::rcc::PeripheralREC = unsafe { rcc.steal_peripheral_rec() };
    let ccdr = rcc
        .sys_ck(200.mhz())
        .hclk(200.mhz())
        .pll1_r_ck(100.mhz()) // for TRACECK
        .freeze(pwrcfg, &dp.SYSCFG);

    cp.SCB.invalidate_icache();
    cp.SCB.enable_icache();
    cp.DWT.enable_cycle_counter();


    // - gpios ----------------------------------------------------------------

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);


    // - leds ----------------------------------------------------------------

    let mut led_user = gpiob.pb14.into_push_pull_output();  // LED3, red
    let mut led_link = gpioe.pe1.into_push_pull_output();   // LED2, yellow
    led_user.set_high().ok();
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
    unsafe {
        nucleo::ethernet::ETHERNET_INTERFACE = Some(nucleo::ethernet::Interface::new(pins));
    }
    let ethernet_interface = unsafe { nucleo::ethernet::ETHERNET_INTERFACE.as_mut().unwrap() };
    match ethernet_interface.up(&MAC_LOCAL, &IP_LOCAL,
                                ccdr.peripheral.ETH1MAC,
                                &ccdr.clocks,) {
        Ok(()) => (),
        Err(e) => {
            hprintln!("Failed to bring up ethernet interface: {}", e).unwrap();
            loop {}
        }
    }

    // wait for link to come up
    let ethernet_interface = unsafe { nucleo::ethernet::ETHERNET_INTERFACE.as_mut().unwrap() };
    while !ethernet_interface.poll_link() { }


    // - jacktrip interface ---------------------------------------------------

    let jacktrip_host = "192.168.20.114";
    let jacktrip_port = 34254;
    let mut jacktrip_interface = jacktrip::Interface::<driver::lan8742a::Socket>::new("192.168.20.99", 12345);
    match jacktrip_interface.connect(jacktrip_host, jacktrip_port) {
        Ok(()) => (),
        Err(e) => {
            hprintln!("Jacktrip interface failed to connect to remote: {}:{}",
                      jacktrip_host, jacktrip_port).unwrap();
            loop {}
        }
    };
    hprintln!("connected to jacktrip server").unwrap();


    // - timer ----------------------------------------------------------------

    systick_init(&mut cp.SYST, &ccdr.clocks);  // 1ms tick
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // - main loop ------------------------------------------------------------

    led_user.set_low().unwrap();

    loop {
        match ethernet_interface.poll_link() {
            true => led_link.set_high().unwrap(),
            _    => led_link.set_low().unwrap(),
        }

        for i in 0..jacktrip_interface.config.num_frames {
            let f = (i % 26) as f32 + 65.0;
            match jacktrip_interface.send(&[f]) {
                Ok(_bytes_sent) => (),
                Err(e) => hprintln!("oops: {:?}", e).unwrap(),
            }
        }

        delay.delay_ms(2000_u16);
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
