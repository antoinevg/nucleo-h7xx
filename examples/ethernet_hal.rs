#![no_main]
#![no_std]

/// Simple ethernet example that will respond to icmp pings on
/// `IP_LOCAL` and periodically send a udp packet to
/// `IP_REMOTE:IP_REMOTE_PORT`
///
/// Also see: https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/ethernet-rtic-stm32h747i-disco.rs
///           https://github.com/adamgreig/stm32f4-smoltcp-demo

use panic_semihosting as _;
use cortex_m_semihosting::hprintln;

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m_rt::{entry, exception};
use cortex_m;

use stm32h7xx_hal as hal;
use hal::gpio::Speed::*;
use hal::hal::digital::v2::OutputPin;
use hal::rcc::CoreClocks;
use hal::{ethernet, ethernet::PHY};

use hal::{pac, prelude::*};
use pac::interrupt;

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


// - global constants ---------------------------------------------------------

const MAC_LOCAL: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];
const IP_LOCAL: [u8; 4] = [ 192, 168, 20, 99 ];
const IP_REMOTE: [u8; 4] = [ 192, 168, 20, 114 ];
const IP_REMOTE_PORT: u16 = 34254;

const MAX_UDP_PACKET_SIZE: usize = 576;


// - global static state ------------------------------------------------------

static ATOMIC_TIME: AtomicU32 = AtomicU32::new(0);

static mut ETHERNET: Option<Net> = None;
static mut ETHERNET_STATIC_STORAGE: NetStaticStorage = NetStaticStorage::new();

#[link_section = ".sram3.eth"]
static mut ETHERNET_DESCRIPTOR_RING: ethernet::DesRing = ethernet::DesRing::new();


// - entry points -------------------------------------------------------------

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
    let ccdr = rcc
        .sys_ck(200.mhz())
        .hclk(200.mhz())
        .pll1_r_ck(100.mhz()) // for TRACECK
        .freeze(pwrcfg, &dp.SYSCFG);

    cp.SCB.invalidate_icache();
    cp.SCB.enable_icache();
    cp.DWT.enable_cycle_counter();

    // - leds -----------------------------------------------------------------

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    let mut led_user = gpiob.pb14.into_push_pull_output();  // LED3, red
    let mut led_link = gpioe.pe1.into_push_pull_output();   // LED2, yellow
    led_user.set_high().ok();
    led_link.set_low().ok();

    // - ethernet -------------------------------------------------------------

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);

    let _rmii_ref_clk = gpioa.pa1.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_mdio = gpioa.pa2.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_mdc = gpioc.pc1.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_crs_dv = gpioa.pa7.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_rxd0 = gpioc.pc4.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_rxd1 = gpioc.pc5.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_tx_en = gpiog.pg11.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_txd0 = gpiog.pg13.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_txd1 = gpiob.pb13.into_alternate_af11().set_speed(VeryHigh);

    assert_eq!(ccdr.clocks.hclk().0, 200_000_000); // HCLK 200MHz
    assert_eq!(ccdr.clocks.pclk1().0, 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk2().0, 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk4().0, 100_000_000); // PCLK 100MHz

    let mac_addr = EthernetAddress::from_bytes(&MAC_LOCAL);
    let (eth_dma, eth_mac) = unsafe {
        ethernet::new_unchecked(
            dp.ETHERNET_MAC,
            dp.ETHERNET_MTL,
            dp.ETHERNET_DMA,
            &mut ETHERNET_DESCRIPTOR_RING,
            mac_addr.clone(),
            ccdr.peripheral.ETH1MAC,
            &ccdr.clocks,
        )
    };

    // initialise PHY and wait for link to come up
    let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));
    lan8742a.phy_reset();
    lan8742a.phy_init();
    while !lan8742a.poll_link() { }

    // enable ethernet interrupt
    unsafe {
        ethernet::enable_interrupt();
        cp.NVIC.set_priority(pac::Interrupt::ETH, 196); // mid prio
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::ETH);
    }

    // create global static ETHERNET object
    unsafe {
        ETHERNET = Some(Net::new(&mut ETHERNET_STATIC_STORAGE, eth_dma, mac_addr));
    }

    // - udp socket -----------------------------------------------------------

    let store = unsafe { &mut ETHERNET_STATIC_STORAGE };
    let udp_rx_buffer = UdpSocketBuffer::new(&mut store.udp_rx_metadata[..],
                                             &mut store.udp_rx_buffer_storage[..]);
    let udp_tx_buffer = UdpSocketBuffer::new(&mut store.udp_tx_metadata[..],
                                             &mut store.udp_tx_buffer_storage[..]);
    let mut udp_socket = UdpSocket::new(udp_rx_buffer, udp_tx_buffer);

    let endpoint_local = IpEndpoint::new(Ipv4Address::from_bytes(&IP_LOCAL).into(), 12345);
    let endpoint_remote = IpEndpoint::new(Ipv4Address::from_bytes(&IP_REMOTE).into(), IP_REMOTE_PORT);
    if !udp_socket.is_open() {
        udp_socket.bind(endpoint_local).unwrap();
    }

    let udp_socket_handle = unsafe { ETHERNET.as_mut().unwrap().sockets.add(udp_socket) };


    // - timer ----------------------------------------------------------------

    systick_init(&mut cp.SYST, &ccdr.clocks);  // 1ms tick
    let mut delay = cp.SYST.delay(ccdr.clocks);


    // - main loop ------------------------------------------------------------

    led_user.set_low().unwrap();

    loop {
        match lan8742a.poll_link() {
            true => led_link.set_high().unwrap(),
            _    => led_link.set_low().unwrap(),
        }

        // send a packet
        let mut udp_socket =  unsafe { ETHERNET.as_mut().unwrap().sockets.get::<UdpSocket>(udp_socket_handle) };
        match udp_socket.send_slice("hello there\n".as_bytes(), endpoint_remote) {
            Ok(()) => (),
            Err(e) => hprintln!("oops: {:?}", e).unwrap(),
        }

        delay.delay_ms(2000_u16);
    }
}


// - systick ------------------------------------------------------------------

fn systick_init(syst: &mut pac::SYST, clocks: &CoreClocks) {
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

    if let Some(ethernet) = unsafe { ETHERNET.as_mut() } {
        let time = ATOMIC_TIME.load(Ordering::Relaxed);
        ethernet.poll(time as i64);
    }
}

#[exception]
fn SysTick() {
    ATOMIC_TIME.fetch_add(1, Ordering::Relaxed);
}


// - NetStaticStorage ---------------------------------------------------------

pub struct NetStaticStorage<'a> {
    ip_addrs: [IpCidr; 1],
    socket_set_entries: [Option<SocketSetItem<'a>>; 8],
    neighbor_cache_storage: [Option<(IpAddress, Neighbor)>; 8],
    routes_storage: [Option<(IpCidr, Route)>; 1],

    // network buffers
    udp_rx_metadata: [PacketMetadata<IpEndpoint>; 1],
    udp_tx_metadata: [PacketMetadata<IpEndpoint>; 1],
    udp_rx_buffer_storage: [u8; MAX_UDP_PACKET_SIZE],
    udp_tx_buffer_storage: [u8; MAX_UDP_PACKET_SIZE],
}

impl<'a> NetStaticStorage<'a> {
    pub const fn new() -> Self {
        NetStaticStorage {
            ip_addrs: [IpCidr::Ipv6(Ipv6Cidr::SOLICITED_NODE_PREFIX)],
            socket_set_entries: [None, None, None, None, None, None, None, None],
            neighbor_cache_storage: [None; 8],
            routes_storage: [None; 1],

            udp_rx_metadata: [UdpPacketMetadata::EMPTY],
            udp_tx_metadata: [UdpPacketMetadata::EMPTY],
            udp_rx_buffer_storage: [0u8; MAX_UDP_PACKET_SIZE],
            udp_tx_buffer_storage: [0u8; MAX_UDP_PACKET_SIZE],
        }
    }
}


// - Net ----------------------------------------------------------------------

pub struct Net<'a> {
    interface: EthernetInterface<'a, ethernet::EthernetDMA<'a>>,
    sockets: SocketSet<'a>,
}

impl<'a> Net<'a> {
    pub fn new(
        store: &'static mut NetStaticStorage<'a>,
        ethdev: ethernet::EthernetDMA<'a>,
        ethernet_addr: EthernetAddress,
    ) -> Self {
        store.ip_addrs = [IpCidr::new(Ipv4Address::from_bytes(&IP_LOCAL).into(), 0)];

        let neighbor_cache = NeighborCache::new(&mut store.neighbor_cache_storage[..]);
        let routes = Routes::new(&mut store.routes_storage[..]);
        let interface = EthernetInterfaceBuilder::new(ethdev)
            .ethernet_addr(ethernet_addr)
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut store.ip_addrs[..])
            .routes(routes)
            .finalize();
        let sockets = SocketSet::new(&mut store.socket_set_entries[..]);

        Net {
            interface,
            sockets
        }
    }

    // poll ethernet interface
    pub fn poll(&mut self, now: i64) {
        let timestamp = Instant::from_millis(now);

        match self.interface.poll(&mut self.sockets, timestamp) {
            Ok(_) => (),
            Err(smoltcp::Error::Exhausted) => (),
            Err(smoltcp::Error::Unrecognized) => (),
            Err(e) => hprintln!("Error polling: {:?}", e).unwrap(),
        };

    }
}
