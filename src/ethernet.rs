#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::interrupt::Mutex;
use cortex_m_rt::exception;

use hal::gpio::Speed::*;
use hal::hal::digital::v2::OutputPin;
use hal::prelude::*;
use hal::time::MilliSeconds;
use hal::{ethernet, ethernet::PHY};
use stm32h7xx_hal as hal;

use hal::pac;
use pac::interrupt;

use embedded_timeout_macros::{block_timeout, repeat_timeout, TimeoutError};

use smoltcp;
use smoltcp::iface::{
    Interface, InterfaceBuilder, Neighbor, NeighborCache, Route, Routes, SocketHandle,
    SocketStorage,
};
use smoltcp::socket::{UdpPacketMetadata, UdpSocket, UdpSocketBuffer};
use smoltcp::storage::PacketMetadata;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, IpEndpoint, Ipv4Address, Ipv6Cidr};

use heapless::Vec;

use crate::pins;
use crate::timer::CountDownTimer as Timer;

// - global constants ---------------------------------------------------------

pub const MAX_UDP_PACKET_SIZE: usize = 576;
//pub const MAX_UDP_PACKET_SIZE: usize = 4096;

// - global static state ------------------------------------------------------

#[link_section = ".sram3.eth"]
pub static mut ETHERNET_DESCRIPTOR_RING: ethernet::DesRing<4, 4> = ethernet::DesRing::new();

static mut ETHERNET_MUTEX: Mutex<RefCell<Option<EthernetInterface>>> =
    Mutex::new(RefCell::new(None));
pub static ATOMIC_TIME: AtomicU32 = AtomicU32::new(0);

// - statically allocated storage ---------------------------------------------

static mut ETHERNET_STORAGE: EthernetStorage = EthernetStorage::new();
static mut ETHERNET_SOCKETS_STORAGE: Vec<UdpSocketStorage, heapless::consts::U8> =
    Vec(heapless::i::Vec::new());

pub struct EthernetStorage<'a> {
    ip_addrs: [IpCidr; 1],
    socket_storage: [SocketStorage<'a>; 8],
    neighbor_cache_storage: [Option<(IpAddress, Neighbor)>; 8],
    routes_storage: [Option<(IpCidr, Route)>; 1],
}

impl<'a> EthernetStorage<'a> {
    const fn new() -> Self {
        EthernetStorage {
            ip_addrs: [IpCidr::Ipv6(Ipv6Cidr::SOLICITED_NODE_PREFIX)],
            socket_storage: [SocketStorage::EMPTY; 8],
            neighbor_cache_storage: [None; 8],
            routes_storage: [None; 1],
        }
    }
}

#[derive(Debug)]
pub struct UdpSocketStorage<'a> {
    socket_handle: Option<SocketHandle>,
    udp_rx_metadata: [PacketMetadata<IpEndpoint>; 1],
    udp_tx_metadata: [PacketMetadata<IpEndpoint>; 1],
    udp_rx_buffer: [u8; MAX_UDP_PACKET_SIZE],
    udp_tx_buffer: [u8; MAX_UDP_PACKET_SIZE],
    _marker: core::marker::PhantomData<&'a ()>,
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

// - types --------------------------------------------------------------------

#[derive(Debug)]
pub enum Error {
    LinkTimedOut,
    ToDo,
}

// - ethernet::EthernetInterface ----------------------------------------------

pub struct EthernetInterface<'a> {
    pins: self::Pins,
    lan8742a: Option<hal::ethernet::phy::LAN8742A<hal::ethernet::EthernetMAC>>,
    pub interface: Option<Interface<'a, ethernet::EthernetDMA<'a, 4, 4>>>,
    _marker: core::marker::PhantomData<&'a ()>,
}

impl<'a> EthernetInterface<'a> {
    fn new(pins: self::Pins) -> Self {
        Self {
            pins,
            lan8742a: None,
            interface: None,
            _marker: core::marker::PhantomData,
        }
    }

    pub unsafe fn free(
        mut self,
    ) -> (
        pins::ethernet::Pins,
        hal::ethernet::phy::LAN8742A<hal::ethernet::EthernetMAC>,
    ) {
        // halt interrupts
        let eth_dma = &*pac::ETHERNET_DMA::ptr();
        eth_dma.dmacier.modify(
            |_, w| {
                w.nie()
                    .clear_bit() // normal interrupt summary enable
                    .rie()
                    .clear_bit() // receive interrupt enable
                    .tie()
                    .clear_bit()
            }, // transmit interrupt enable
        );
        cortex_m::peripheral::NVIC::mask(pac::Interrupt::ETH);

        // reclaim the objects used to create this structure
        let owned_resources = (
            pins::ethernet::Pins {
                ref_clk: self.pins.ref_clk.into_analog(),
                md_io: self.pins.md_io.into_analog(),
                md_clk: self.pins.md_clk.into_analog(),
                crs: self.pins.crs.into_analog(),
                rx_d0: self.pins.rx_d0.into_analog(),
                rx_d1: self.pins.rx_d1.into_analog(),
                tx_en: self.pins.tx_en.into_analog(),
                tx_d0: self.pins.tx_d0.into_analog(),
                tx_d1: self.pins.tx_d1.into_analog(),
            },
            core::ptr::replace(&mut self.lan8742a, None).unwrap(),
        );

        // clean out static global singleton
        cortex_m::interrupt::free(|cs| {
            ETHERNET_MUTEX.borrow(cs).replace(None);
        });

        owned_resources
    }

    pub fn start(
        pins: pins::ethernet::Pins,
        mac_address: &[u8; 6],
        ip_address: &[u8; 4],
        eth1mac: hal::rcc::rec::Eth1Mac,
        ccdr_clocks: &hal::rcc::CoreClocks,
        timeout_timer: Timer<pac::TIM17>,
    ) -> Result<Timer<pac::TIM17>, Error> {
        let pins = self::Pins {
            ref_clk: pins.ref_clk.into_alternate(),
            md_io: pins.md_io.into_alternate(),
            md_clk: pins.md_clk.into_alternate(),
            crs: pins.crs.into_alternate(),
            rx_d0: pins.rx_d0.into_alternate(),
            rx_d1: pins.rx_d1.into_alternate(),
            tx_en: pins.tx_en.into_alternate(),
            tx_d0: pins.tx_d0.into_alternate(),
            tx_d1: pins.tx_d1.into_alternate(),
        };

        let mut interface = EthernetInterface::new(pins);
        let timeout_timer = match interface.up(
            unsafe { &mut ETHERNET_STORAGE },
            mac_address,
            ip_address,
            eth1mac,
            ccdr_clocks,
            timeout_timer,
        ) {
            Ok(timeout_timer) => timeout_timer,
            Err(e) => {
                return Err(e);
            }
        };

        // wrap ethernet interface in mutex
        cortex_m::interrupt::free(|cs| unsafe {
            ETHERNET_MUTEX.borrow(cs).replace(Some(interface));
        });

        // configure systick timer to 1ms
        let syst = unsafe { &mut pac::CorePeripherals::steal().SYST };
        let c_ck_mhz = ccdr_clocks.c_ck().to_MHz();
        let syst_calib = 0x3E8;
        syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
        syst.set_reload((syst_calib * c_ck_mhz) - 1);
        syst.enable_interrupt();
        syst.enable_counter();

        Ok(timeout_timer)
    }

    pub fn interrupt_free<F, R>(f: F) -> R
    where
        F: FnOnce(&mut EthernetInterface<'static>) -> R,
    {
        cortex_m::interrupt::free(|cs| {
            if let Some(ethernet_interface) =
                unsafe { ETHERNET_MUTEX.borrow(cs).borrow_mut().as_mut() }
            {
                f(ethernet_interface)
            } else {
                panic!("Ethernet interface has not been started");
            }
        })
    }

    fn up(
        &mut self,
        ethernet_storage: &'static mut EthernetStorage<'a>,
        mac_address: &[u8; 6],
        ip_address: &[u8; 4],
        eth1mac: hal::rcc::rec::Eth1Mac,
        ccdr_clocks: &hal::rcc::CoreClocks,
        mut timeout_timer: Timer<pac::TIM17>,
    ) -> Result<Timer<pac::TIM17>, Error> {
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

        // initialise PHY
        let mut lan8742a: hal::ethernet::phy::LAN8742A<hal::ethernet::EthernetMAC> =
            ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));
        lan8742a.phy_reset();
        lan8742a.phy_init();

        // wait for link to come up
        let ten_seconds = core::time::Duration::new(10, 0);
        timeout_timer.start(ten_seconds);
        let result: Result<(), TimeoutError<()>> = block_timeout!(&mut timeout_timer, {
            if lan8742a.poll_link() {
                Ok(())
            } else {
                Err(nb::Error::WouldBlock)
            }
        });
        match result {
            Ok(()) => (),
            Err(TimeoutError::Timeout) | Err(_) => {
                return Err(Error::LinkTimedOut);
            }
        }

        // enable ethernet interrupt
        let cp = unsafe { &mut pac::CorePeripherals::steal() };
        unsafe {
            ethernet::enable_interrupt();
            cp.NVIC.set_priority(pac::Interrupt::ETH, 196); // mid prio
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::ETH);
        }

        // --------------------------------------------------------------------

        // set local ip address
        ethernet_storage.ip_addrs = [IpCidr::new(Ipv4Address::from_bytes(ip_address).into(), 0)];

        let neighbor_cache = NeighborCache::new(&mut ethernet_storage.neighbor_cache_storage[..]);
        let routes = Routes::new(&mut ethernet_storage.routes_storage[..]);
        let interface = InterfaceBuilder::new(eth_dma, &mut ethernet_storage.socket_storage[..])
            .hardware_addr(ethernet_address.into())
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut ethernet_storage.ip_addrs[..])
            .routes(routes)
            .finalize();
        self.lan8742a = Some(lan8742a);
        self.interface = Some(interface);

        Ok(timeout_timer)
    }

    pub fn poll_link(&mut self) -> bool {
        self.lan8742a.as_mut().unwrap().poll_link()
    }

    // poll ethernet interface
    pub fn poll(&mut self) -> Result<bool, smoltcp::Error> {
        let timestamp = smoltcp::time::Instant::from_millis(self.now());
        self.interface.as_mut().unwrap().poll(timestamp)
    }

    pub fn poll_delay(&mut self) -> Option<smoltcp::time::Duration> {
        let timestamp = smoltcp::time::Instant::from_millis(self.now());
        self.interface.as_mut().unwrap().poll_delay(timestamp)
    }

    /// returns an absolute time value in milliseconds
    pub fn now(&self) -> i64 {
        ATOMIC_TIME.load(Ordering::Relaxed).into()
    }

    pub fn new_udp_socket(&mut self) -> SocketHandle {
        unsafe {
            ETHERNET_SOCKETS_STORAGE
                .push(UdpSocketStorage::new())
                .unwrap(); // TODO handle result
        }
        let len = unsafe { ETHERNET_SOCKETS_STORAGE.len() };
        let socket_storage = unsafe { &mut ETHERNET_SOCKETS_STORAGE[len - 1] };

        let udp_socket = UdpSocket::new(
            UdpSocketBuffer::new(
                &mut socket_storage.udp_rx_metadata[..],
                &mut socket_storage.udp_rx_buffer[..],
            ),
            UdpSocketBuffer::new(
                &mut socket_storage.udp_tx_metadata[..],
                &mut socket_storage.udp_tx_buffer[..],
            ),
        );

        let socket_handle = self.interface.as_mut().unwrap().add_socket(udp_socket);
        socket_handle
    }
}

// - Pins ---------------------------------------------------------------------

type AlternateFunction11 = hal::gpio::Alternate<11>;

// Also see: https://github.com/stm32-rs/stm32-eth/blob/master/src/setup.rs

use hal::gpio::gpioa;
use hal::gpio::gpiob;
use hal::gpio::gpioc;
use hal::gpio::gpiog;

pub struct Pins {
    pub ref_clk: gpioa::PA1<AlternateFunction11>, // REFCLK,   // RmiiRefClk
    pub md_io: gpioa::PA2<AlternateFunction11>,   // IO,       // MDIO
    pub md_clk: gpioc::PC1<AlternateFunction11>,  // CLK,      // MDC
    pub crs: gpioa::PA7<AlternateFunction11>,     // CRS,      // RmiiCrsDv
    pub rx_d0: gpioc::PC4<AlternateFunction11>,   // RXD0,     // RmiiRxD0
    pub rx_d1: gpioc::PC5<AlternateFunction11>,   // RXD1,     // RmiiRxD0
    pub tx_en: gpiog::PG11<AlternateFunction11>,  // TXEN,     // RmiiTxEN
    pub tx_d0: gpiog::PG13<AlternateFunction11>,  // TXD0,     // RmiiTxD0
    pub tx_d1: gpiob::PB13<AlternateFunction11>,  // TXD1,     // RmiiTxD1
}

// - interrupts and exceptions ------------------------------------------------

#[interrupt]
fn ETH() {
    unsafe { ethernet::interrupt_handler() };
}

#[exception]
fn SysTick() {
    ATOMIC_TIME.fetch_add(1, Ordering::Relaxed);
}
