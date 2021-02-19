#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]

use cortex_m_semihosting::hprintln;

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::interrupt::Mutex;
use cortex_m_rt::exception;

use stm32h7xx_hal as hal;
use hal::prelude::*;
use hal::hal::digital::v2::OutputPin;
use hal::gpio::Speed::*;
use hal::{ethernet, ethernet::PHY};

use hal::pac;
use pac::interrupt;

use smoltcp;
use smoltcp::iface::{
    EthernetInterface, EthernetInterfaceBuilder, Neighbor, NeighborCache,
    Route, Routes,
};
use smoltcp::socket::{SocketHandle, SocketSet, SocketSetItem};
use smoltcp::socket::{UdpSocket, UdpSocketBuffer, UdpPacketMetadata};
use smoltcp::storage::PacketMetadata;
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv6Cidr, IpEndpoint, Ipv4Address};

use heapless::Vec;


// - global constants ---------------------------------------------------------

pub const MAX_UDP_PACKET_SIZE: usize = 576;
//pub const MAX_UDP_PACKET_SIZE: usize = 4096;


// - global static state ------------------------------------------------------

#[link_section = ".sram3.eth"]
pub static mut ETHERNET_DESCRIPTOR_RING: ethernet::DesRing = ethernet::DesRing::new();

//#[link_section = ".axisram.eth"]
static mut ETHERNET_MUTEX: Mutex<RefCell<Option<Interface>>> = Mutex::new(RefCell::new(None));
pub static ATOMIC_TIME: AtomicU32 = AtomicU32::new(0);


// - statically allocated storage ---------------------------------------------

pub static mut ETHERNET_STORAGE: Storage = Storage::new();
pub static mut ETHERNET_SOCKETS_STORAGE: Vec<UdpSocketStorage, heapless::consts::U8>
    = Vec(heapless::i::Vec::new());

pub struct Storage<'a> {
    ip_addrs: [IpCidr; 1],
    socket_set_entries: [Option<SocketSetItem<'a>>; 8],
    neighbor_cache_storage: [Option<(IpAddress, Neighbor)>; 8],
    routes_storage: [Option<(IpCidr, Route)>; 1],
}

impl<'a> Storage<'a> {
    const fn new() -> Self {
        Storage {
            ip_addrs: [IpCidr::Ipv6(Ipv6Cidr::SOLICITED_NODE_PREFIX)],
            socket_set_entries: [None, None, None, None, None, None, None, None],
            neighbor_cache_storage: [None; 8],
            routes_storage: [None; 1],
        }
    }
}

#[derive(Debug)]
pub struct UdpSocketStorage <'a> {
    socket_handle: Option<SocketHandle>,
    udp_rx_metadata: [PacketMetadata<IpEndpoint>; 1],
    udp_tx_metadata: [PacketMetadata<IpEndpoint>; 1],
    udp_rx_buffer: [u8; MAX_UDP_PACKET_SIZE],
    udp_tx_buffer: [u8; MAX_UDP_PACKET_SIZE],
    _marker: core::marker::PhantomData<&'a ()>
}

impl<'a> UdpSocketStorage<'a> {
    const fn new() -> Self {
        Self {
            socket_handle: None,
            udp_rx_metadata: [UdpPacketMetadata::EMPTY],
            udp_tx_metadata: [UdpPacketMetadata::EMPTY],
            udp_rx_buffer: [0u8; MAX_UDP_PACKET_SIZE],
            udp_tx_buffer: [0u8; MAX_UDP_PACKET_SIZE],
            _marker: core::marker::PhantomData,
        }
    }
}


// - ethernet::Interface ------------------------------------------------------

pub struct Interface<'a> {
    pins: Pins,

    lan8742a: Option<hal::ethernet::phy::LAN8742A<hal::ethernet::EthernetMAC>>,
    interface: Option<EthernetInterface<'a, ethernet::EthernetDMA<'a>>>,
    pub sockets: Option<SocketSet<'static>>,
    _marker: core::marker::PhantomData<&'a ()>,
}


impl<'a> Interface<'a> {
    fn new(pins: Pins) -> Self {
        Self {
            pins: pins,
            lan8742a: None,
            interface: None,
            sockets: None,
            _marker: core::marker::PhantomData,
        }
    }

    pub fn start(
        pins: Pins,
        mac_address: &[u8; 6],
        ip_address: &[u8; 4],
        eth1mac: hal::rcc::rec::Eth1Mac,
        ccdr_clocks: &hal::rcc::CoreClocks
    ) -> Result<(), ()> {
        let mut interface = Interface::new(pins);
        match interface.up(mac_address,
                           ip_address,
                           eth1mac,
                           ccdr_clocks) {
            Ok(()) => (),
            Err(e) => {
                hprintln!("Failed to bring up ethernet interface: {}", e).unwrap();
                return Err(()); // TODO return a more informative result on failure
            }
        }

        // wrap ethernet interface in mutex
        cortex_m::interrupt::free(|cs| {
            unsafe {
                ETHERNET_MUTEX.borrow(cs).replace(Some(interface));
            }
        });

        Ok(())
    }

    pub fn interrupt_free<F, R>(f: F) -> R where
        F: FnOnce(&mut Interface<'static>) -> R {
        cortex_m::interrupt::free(|cs| {
            if let Some (ethernet_interface) = unsafe { ETHERNET_MUTEX.borrow(cs).borrow_mut().as_mut() } {
                f(ethernet_interface)
            } else {
                hprintln!("Ethernet interface has not been started").unwrap();
                panic!("Ethernet interface has not been started");
            }
        })
    }

    pub fn poll_link(&mut self) -> bool {
        self.lan8742a.as_mut().unwrap().poll_link()
    }

    // poll ethernet interface
    pub fn poll(&mut self, now: i64) {
        let timestamp = Instant::from_millis(now);

        // TODO handle Option properly
        match self.interface.as_mut().unwrap().poll(&mut self.sockets.as_mut().unwrap(), timestamp) {
            Ok(result) => {
                /*let gpioe = unsafe { &mut pac::Peripherals::steal().GPIOE };
                if result { // packets were processed or emitted
                    gpioe.bsrr.write(|w| w.br1().set_bit());
                } else {
                    gpioe.bsrr.write(|w| w.bs1().set_bit());
                }*/
            },
            Err(smoltcp::Error::Exhausted) => (),
            Err(smoltcp::Error::Unrecognized) => (),
            Err(e) => hprintln!("poll {:?}", e).unwrap(),
        };
    }

    pub fn new_udp_socket(&mut self) -> SocketHandle {
        unsafe {
            ETHERNET_SOCKETS_STORAGE.push(UdpSocketStorage::new()).unwrap();  // TODO handle result
        }
        let len = unsafe { ETHERNET_SOCKETS_STORAGE.len() };
        let socket_storage = unsafe { &mut ETHERNET_SOCKETS_STORAGE[len - 1] };

        let udp_socket = UdpSocket::new(
            UdpSocketBuffer::new(&mut socket_storage.udp_rx_metadata[..],
                                 &mut socket_storage.udp_rx_buffer[..]),
            UdpSocketBuffer::new(&mut socket_storage.udp_tx_metadata[..],
                                 &mut socket_storage.udp_tx_buffer[..]),
        );

        let socket_handle = self.sockets.as_mut().unwrap().add(udp_socket);
        socket_handle
    }

    fn up(&mut self,
              mac_address: &[u8; 6],
              ip_address: &[u8; 4],
              eth1mac: hal::rcc::rec::Eth1Mac,
              ccdr_clocks: &hal::rcc::CoreClocks) -> Result<(), u32> {

        assert_eq!(ccdr_clocks.hclk().0,  240_000_000); // HCLK 240MHz
        assert_eq!(ccdr_clocks.pclk1().0, 120_000_000); // PCLK 120MHz
        assert_eq!(ccdr_clocks.pclk2().0, 120_000_000); // PCLK 120MHz
        assert_eq!(ccdr_clocks.pclk4().0, 120_000_000); // PCLK 120MHz

        let dp = unsafe { pac::Peripherals::steal() };
        let ethernet_address = EthernetAddress::from_bytes(mac_address);
        let (eth_dma, eth_mac) = unsafe {
            ethernet::new_unchecked(
                dp.ETHERNET_MAC,
                dp.ETHERNET_MTL,
                dp.ETHERNET_DMA,
                &mut ETHERNET_DESCRIPTOR_RING,
                ethernet_address,
                eth1mac,
                ccdr_clocks,
            )
        };

        // initialise PHY and wait for link to come up
        let mut lan8742a: hal::ethernet::phy::LAN8742A<hal::ethernet::EthernetMAC>
            = ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));
        lan8742a.phy_reset();
        lan8742a.phy_init();
        while !lan8742a.poll_link() { } // TODO expose as method

        // enable ethernet interrupt
        let cp = unsafe { &mut pac::CorePeripherals::steal() };
        unsafe {
            ethernet::enable_interrupt();
            cp.NVIC.set_priority(pac::Interrupt::ETH, 196); // mid prio
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::ETH);
        }

        // --------------------------------------------------------------------

        unsafe {
            ETHERNET_STORAGE.ip_addrs = [IpCidr::new(Ipv4Address::from_bytes(ip_address).into(), 0)];
        }

        let neighbor_cache = NeighborCache::new(unsafe { &mut ETHERNET_STORAGE.neighbor_cache_storage[..] });
        let routes = Routes::new(unsafe { &mut ETHERNET_STORAGE.routes_storage[..] });
        let interface = EthernetInterfaceBuilder::new(eth_dma)
            .ethernet_addr(ethernet_address)
            .neighbor_cache(neighbor_cache)
            .ip_addrs(unsafe { &mut ETHERNET_STORAGE.ip_addrs[..] })
            .routes(routes)
            .finalize();
        let sockets = SocketSet::new(unsafe { &mut ETHERNET_STORAGE.socket_set_entries[..] });

        self.lan8742a = Some(lan8742a);
        self.interface = Some(interface);
        self.sockets = Some(sockets);

        Ok(())
    }
}


// - ethernet::Pins -----------------------------------------------------------

type AlternateFunction11 = hal::gpio::Alternate<hal::gpio::AF11>;

// Also see: https://github.com/stm32-rs/stm32-eth/blob/master/src/setup.rs

use hal::gpio::gpioa;
use hal::gpio::gpiob;
use hal::gpio::gpioc;
use hal::gpio::gpiog;

pub struct Pins {
    pub ref_clk: gpioa::PA1 <AlternateFunction11>, // REFCLK,   // RmiiRefClk
    pub md_io:   gpioa::PA2 <AlternateFunction11>, // IO,       // MDIO
    pub md_clk:  gpioc::PC1 <AlternateFunction11>, // CLK,      // MDC
    pub crs:     gpioa::PA7 <AlternateFunction11>, // CRS,      // RmiiCrsDv
    pub rx_d0:   gpioc::PC4 <AlternateFunction11>, // RXD0,     // RmiiRxD0
    pub rx_d1:   gpioc::PC5 <AlternateFunction11>, // RXD1,     // RmiiRxD0
    pub tx_en:   gpiog::PG11<AlternateFunction11>, // TXEN,     // RmiiTxEN
    pub tx_d0:   gpiog::PG13<AlternateFunction11>, // TXD0,     // RmiiTxD0
    pub tx_d1:   gpiob::PB13<AlternateFunction11>, // TXD1,     // RmiiTxD1
}


// - systick configuration ----------------------------------------------------

pub fn systick_init(syst: &mut pac::SYST, clocks: &hal::rcc::CoreClocks) {
    let c_ck_mhz = clocks.c_ck().0 / 1_000_000;
    let syst_calib = 0x3E8;
    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload((syst_calib * c_ck_mhz) - 1);
    syst.enable_interrupt();
    syst.enable_counter();
}


// - interrupts and exceptions ------------------------------------------------

#[interrupt]
fn ETH() {
    unsafe { ethernet::interrupt_handler() };

    /*let time = ATOMIC_TIME.load(Ordering::Relaxed);
    cortex_m::interrupt::free(|cs| {
        if let Some (ethernet_interface) = unsafe { ETHERNET_MUTEX.borrow(cs).borrow_mut().as_mut() } {
            ethernet_interface.poll(time as i64);
        }
    });*/
}

#[exception]
fn SysTick() {
    ATOMIC_TIME.fetch_add(1, Ordering::Relaxed);
}
