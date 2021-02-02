/// Simple ethernet example that will respond to icmp pings on
/// 192.168.20.99

#![no_main]
#![no_std]

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
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv6Cidr};


// - global constants ---------------------------------------------------------

const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];


// - global static state ------------------------------------------------------

static ATOMIC_TIME: AtomicU32 = AtomicU32::new(0);

static mut ETHERNET: Option<Net> = None;
static mut ETHERNET_STATIC_STORAGE: NetStorageStatic = NetStorageStatic::new();

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
    // cp.SCB.enable_dcache(&mut cp.CPUID); // TODO: ETH DMA coherence issues
    cp.DWT.enable_cycle_counter();

    // - ethernet -------------------------------------------------------------

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
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

    let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(&MAC_ADDRESS);
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

    // initialise PHY
    let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));
    lan8742a.phy_reset();
    lan8742a.phy_init();

    // eth_dma should not be used until the PHY reports the link is up
    unsafe {
        ethernet::enable_interrupt();
        cp.NVIC.set_priority(pac::Interrupt::ETH, 196); // mid prio
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::ETH);
    }

    // create global static ETHERNET object
    unsafe {
        ETHERNET = Some(Net::new(&mut ETHERNET_STATIC_STORAGE, eth_dma, mac_addr));
    }

    // leds
    let mut led_user = gpiob.pb14.into_push_pull_output();  // LED3, red
    let mut led_link = gpioe.pe1.into_push_pull_output();   // LED2, yellow
    led_user.set_high().ok();
    led_link.set_low().ok();


    // - start timer ----------------------------------------------------------

    let delay = cp.SYST.delay(ccdr.clocks);
    systick_init(&mut delay.free(), ccdr.clocks);
    unsafe {
        cp.SCB.shpr[15 - 4].write(128); // systick exception priority
    }
    //systick_init(&mut cp.SYST, ccdr.clocks);  // 1ms tick

    // - main loop ------------------------------------------------------------

    loop {
        match lan8742a.poll_link() {
            true => {
                led_link.set_high().unwrap();
                led_user.set_low().unwrap();
            },
            _ => {
                led_link.set_low().unwrap();
                led_user.set_high().unwrap();
            }
        }
    }
}


// - systick ------------------------------------------------------------------

fn systick_init(syst: &mut pac::SYST, clocks: CoreClocks) {
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

#[exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}


// - Net ----------------------------------------------------------------------

pub struct NetStorageStatic<'a> {
    ip_addrs: [IpCidr; 1],
    socket_set_entries: [Option<SocketSetItem<'a>>; 8],
    neighbor_cache_storage: [Option<(IpAddress, Neighbor)>; 8],
    routes_storage: [Option<(IpCidr, Route)>; 1],
}

impl<'a> NetStorageStatic<'a> {
    pub const fn new() -> Self {
        NetStorageStatic {
            ip_addrs: [IpCidr::Ipv6(Ipv6Cidr::SOLICITED_NODE_PREFIX)],
            socket_set_entries: [None, None, None, None, None, None, None, None],
            neighbor_cache_storage: [None; 8],
            routes_storage: [None; 1],
        }
    }
}


pub struct Net<'a> {
    interface: EthernetInterface<'a, ethernet::EthernetDMA<'a>>,
    sockets: SocketSet<'a>,
}

impl<'a> Net<'a> {
    pub fn new(
        store: &'static mut NetStorageStatic<'a>,
        ethdev: ethernet::EthernetDMA<'a>,
        ethernet_addr: EthernetAddress,
    ) -> Self {
        store.ip_addrs = [IpCidr::new(IpAddress::v4(192, 168, 20, 99), 0)];

        let neighbor_cache = NeighborCache::new(&mut store.neighbor_cache_storage[..]);
        let routes = Routes::new(&mut store.routes_storage[..]);

        let interface = EthernetInterfaceBuilder::new(ethdev)
            .ethernet_addr(ethernet_addr)
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut store.ip_addrs[..])
            .routes(routes)
            .finalize();
        let sockets = SocketSet::new(&mut store.socket_set_entries[..]);

        return Net { interface, sockets };
    }

    /// Polls on the ethernet interface. You should refer to the smoltcp
    /// documentation for poll() to understand how to call poll efficiently
    pub fn poll(&mut self, now: i64) {
        let timestamp = Instant::from_millis(now);

        self.interface
            .poll(&mut self.sockets, timestamp)
            .map(|_| ()) // ?
            .unwrap_or_else(|e| {
                hprintln!("Poll: {:?}", e).unwrap()
            });
    }
}
