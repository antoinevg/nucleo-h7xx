pub use stm32h7xx_hal as hal;


// - types --------------------------------------------------------------------

pub type D51 = hal::gpio::gpiod::PD7<hal::gpio::Analog>;
pub type D52 = hal::gpio::gpiod::PD6<hal::gpio::Analog>;


// - Pins ---------------------------------------------------------------------

pub struct Pins {
    // ST Zio "Arduino" pins
    pub d51: D51,
    pub d52: D52,

    // board peripherals
    pub ethernet:  ethernet::Pins,
    pub user_leds: user_leds::Pins,

    // TODO
}

pub mod user_leds {
    use stm32h7xx_hal as hal;
    use hal::gpio::{Output, PushPull};

    #[cfg(not(feature = "led-1-pa5"))]
    pub type Pin1 = hal::gpio::gpiob::PB0<hal::gpio::Analog>; // SB65=off, SB54=on
    #[cfg(any(feature = "led-1-pa5"))]
    pub type Pin1 = hal::gpio::gpioa::PA5<hal::gpio::Analog>; // SB65=on,  SB54=off
    pub type Pin2 = hal::gpio::gpioe::PE1<hal::gpio::Analog>;
    pub type Pin3 = hal::gpio::gpiob::PB14<hal::gpio::Analog>;

    #[cfg(not(feature = "led-1-pa5"))]
    pub type Ld1 = hal::gpio::gpiob::PB0<Output<PushPull>>;
    #[cfg(any(feature = "led-1-pa5"))]
    pub type Ld1 = hal::gpio::gpioa::PA5<Output<PushPull>>;
    pub type Ld2 = hal::gpio::gpioe::PE1<Output<PushPull>>;
    pub type Ld3 = hal::gpio::gpiob::PB14<Output<PushPull>>;

    pub type Type = crate::led::UserLedsGeneric<Ld1, Ld2, Ld3>;

    pub struct Pins {
        pub ld1: Pin1,
        pub ld2: Pin2,
        pub ld3: Pin3,
    }
}

pub mod ethernet {
    use stm32h7xx_hal as hal;

    pub struct Pins {
        pub ref_clk: hal::gpio::gpioa::PA1 <hal::gpio::Analog>, // REFCLK,   // RmiiRefClk
        pub md_io:   hal::gpio::gpioa::PA2 <hal::gpio::Analog>, // IO,       // MDIO
        pub md_clk:  hal::gpio::gpioc::PC1 <hal::gpio::Analog>, // CLK,      // MDC
        pub crs:     hal::gpio::gpioa::PA7 <hal::gpio::Analog>, // CRS,      // RmiiCrsDv
        pub rx_d0:   hal::gpio::gpioc::PC4 <hal::gpio::Analog>, // RXD0,     // RmiiRxD0
        pub rx_d1:   hal::gpio::gpioc::PC5 <hal::gpio::Analog>, // RXD1,     // RmiiRxD0
        pub tx_en:   hal::gpio::gpiog::PG11<hal::gpio::Analog>, // TXEN,     // RmiiTxEN
        pub tx_d0:   hal::gpio::gpiog::PG13<hal::gpio::Analog>, // TXD0,     // RmiiTxD0
        pub tx_d1:   hal::gpio::gpiob::PB13<hal::gpio::Analog>, // TXD1,     // RmiiTxD1
    }
}
