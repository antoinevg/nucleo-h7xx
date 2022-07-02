#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_parens)]
#![allow(unused_variables)]
#![no_main]
#![no_std]

use cortex_m;
use cortex_m_rt::entry;

use nucleo::loggit;
use nucleo_h7xx as nucleo;

use hal::gpio::Speed::*;
use hal::hal::digital::v2::OutputPin;
use hal::hal::digital::v2::ToggleableOutputPin;
use hal::prelude::*;
use hal::rcc::CoreClocks;
use hal::{ethernet, ethernet::PHY};
use nucleo::hal;

use hal::pac;
use pac::interrupt;

use smoltcp;
use smoltcp::iface::{Interface, InterfaceBuilder, Neighbor, NeighborCache, Route, Routes};
use smoltcp::socket::{UdpPacketMetadata, UdpSocket, UdpSocketBuffer};
use smoltcp::storage::PacketMetadata;
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, IpEndpoint, Ipv4Address, Ipv6Cidr};

use log::{debug, error, info};

/// Simple ethernet example that will respond to icmp pings on
/// `IP_LOCAL` and periodically send a udp packet to
/// `IP_REMOTE:IP_REMOTE_PORT`
///
/// You can start a simple listening server with netcat:
///
///     nc -u -l 34254

const MAC_LOCAL: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];
const IP_LOCAL: [u8; 4] = [192, 168, 20, 99];
const IP_REMOTE: [u8; 4] = [192, 168, 20, 114];
const IP_REMOTE_PORT: u16 = 34254;

mod utilities;

#[entry]
fn main() -> ! {
    // - endpoints ------------------------------------------------------------

    let local_endpoint = IpEndpoint::new(Ipv4Address::from_bytes(&IP_LOCAL).into(), 1234);
    let remote_endpoint =
        IpEndpoint::new(Ipv4Address::from_bytes(&IP_REMOTE).into(), IP_REMOTE_PORT);

    // - board setup ----------------------------------------------------------

    info!("Setting up board");

    let board = nucleo::Board::take().unwrap();

    let dp = pac::Peripherals::take().unwrap();

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

    utilities::logger::init();

    // - ethernet interface ---------------------------------------------------

    info!("Bringing up ethernet interface");

    let timeout_timer = dp
        .TIM17
        .timer(100.Hz(), ccdr.peripheral.TIM17, &ccdr.clocks);
    let timeout_timer = nucleo::timer::CountDownTimer::new(timeout_timer);
    let timeout_timer = match nucleo::ethernet::EthernetInterface::start(
        pins.ethernet,
        &MAC_LOCAL,
        &IP_LOCAL,
        ccdr.peripheral.ETH1MAC,
        &ccdr.clocks,
        timeout_timer,
    ) {
        Ok(tim17) => tim17,
        Err(e) => {
            error!("Failed to start ethernet interface: {:?}", e);
            loop {}
        }
    };

    // wait for link to come up
    info!("Waiting for link to come up");
    nucleo::ethernet::EthernetInterface::interrupt_free(
        |ethernet_interface| {
            while !ethernet_interface.poll_link() {}
        },
    );

    // create and bind socket
    let socket_handle = nucleo::ethernet::EthernetInterface::interrupt_free(|ethernet_interface| {
        let socket_handle = ethernet_interface.new_udp_socket();
        let socket = ethernet_interface
            .interface
            .as_mut()
            .unwrap()
            .get_socket::<UdpSocket>(socket_handle);
        match socket.bind(local_endpoint) {
            Ok(()) => return socket_handle,
            Err(e) => {
                error!("Failed to bind socket to endpoint: {:?}", local_endpoint);
                loop {}
            }
        }
    });

    // - main loop ------------------------------------------------------------

    info!("Entering main loop");

    let mut last = 0;

    loop {
        cortex_m::asm::wfi();

        // poll ethernet interface
        let now = nucleo::ethernet::EthernetInterface::interrupt_free(|ethernet_interface| {
            match ethernet_interface.poll() {
                Ok(result) => {} // packets were processed or emitted
                Err(smoltcp::Error::Exhausted) => (),
                Err(smoltcp::Error::Unrecognized) => (),
                Err(e) => debug!("ethernet::EthernetInterface.poll() -> {:?}", e),
            }
            ethernet_interface.now()
        });

        // check if it has been 5 seconds since we last sent something
        if (now - last) < 5000 {
            continue;
        } else {
            last = now;
        }

        // send something
        nucleo::ethernet::EthernetInterface::interrupt_free(|ethernet_interface| {
            let socket = ethernet_interface
                .interface
                .as_mut()
                .unwrap()
                .get_socket::<UdpSocket>(socket_handle);
            match socket.send_slice("nucleo says hello!\n".as_bytes(), remote_endpoint) {
                Ok(()) => (),
                Err(smoltcp::Error::Exhausted) => (),
                Err(e) => error!("UdpSocket::send error: {:?}", e),
            };
        });
    }
}
