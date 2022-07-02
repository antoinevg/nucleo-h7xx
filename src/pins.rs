pub use stm32h7xx_hal as hal;

// - types --------------------------------------------------------------------

pub type A0 = hal::gpio::gpioa::PA3<hal::gpio::Analog>;
pub type A1 = hal::gpio::gpioc::PC0<hal::gpio::Analog>;
pub type A2 = hal::gpio::gpioc::PC3<hal::gpio::Analog>;
pub type A3 = hal::gpio::gpiob::PB1<hal::gpio::Analog>;
pub type A4 = hal::gpio::gpioc::PC2<hal::gpio::Analog>;
pub type A5 = hal::gpio::gpiof::PF11<hal::gpio::Analog>;
pub type A6 = hal::gpio::gpiof::PF6<hal::gpio::Analog>;
pub type A7 = hal::gpio::gpiof::PF10<hal::gpio::Analog>;
pub type A8 = hal::gpio::gpioa::PA2<hal::gpio::Analog>;

pub type D0 = hal::gpio::gpiob::PB7<hal::gpio::Analog>;
pub type D1 = hal::gpio::gpiob::PB6<hal::gpio::Analog>;
pub type D2 = hal::gpio::gpiog::PG14<hal::gpio::Analog>;
pub type D3 = hal::gpio::gpioe::PE13<hal::gpio::Analog>;
pub type D4 = hal::gpio::gpioe::PE14<hal::gpio::Analog>;
pub type D5 = hal::gpio::gpioe::PE11<hal::gpio::Analog>;
pub type D6 = hal::gpio::gpioa::PA8<hal::gpio::Analog>;
pub type D7 = hal::gpio::gpiog::PG12<hal::gpio::Analog>;
pub type D8 = hal::gpio::gpiog::PG9<hal::gpio::Analog>;
pub type D9 = hal::gpio::gpiod::PD15<hal::gpio::Analog>;

pub type D10 = hal::gpio::gpiod::PD14<hal::gpio::Analog>;
pub type D11 = hal::gpio::gpiob::PB5<hal::gpio::Analog>;
pub type D12 = hal::gpio::gpioa::PA6<hal::gpio::Analog>;
pub type D13 = hal::gpio::gpioa::PA5<hal::gpio::Analog>;
pub type D14 = hal::gpio::gpiob::PB9<hal::gpio::Analog>;
pub type D15 = hal::gpio::gpiob::PB8<hal::gpio::Analog>;
pub type D16 = hal::gpio::gpioc::PC6<hal::gpio::Analog>;
pub type D17 = hal::gpio::gpiob::PB15<hal::gpio::Analog>;
pub type D18 = hal::gpio::gpiob::PB13<hal::gpio::Analog>;
pub type D19 = hal::gpio::gpiob::PB12<hal::gpio::Analog>;

pub type D20 = hal::gpio::gpioa::PA15<hal::gpio::Alternate<0>>;
pub type D21 = hal::gpio::gpioc::PC7<hal::gpio::Analog>;
pub type D22 = hal::gpio::gpiob::PB5<hal::gpio::Analog>;
pub type D23 = hal::gpio::gpiob::PB3<hal::gpio::Alternate<0>>;
pub type D24 = hal::gpio::gpioa::PA4<hal::gpio::Analog>;
pub type D25 = hal::gpio::gpiob::PB4<hal::gpio::Alternate<0>>;
pub type D26 = hal::gpio::gpiog::PG6<hal::gpio::Analog>;
pub type D27 = hal::gpio::gpiob::PB2<hal::gpio::Analog>;
pub type D28 = hal::gpio::gpiod::PD13<hal::gpio::Analog>;
pub type D29 = hal::gpio::gpiod::PD12<hal::gpio::Analog>;

pub type D30 = hal::gpio::gpiod::PD11<hal::gpio::Analog>;
pub type D31 = hal::gpio::gpioe::PE2<hal::gpio::Analog>;
pub type D32 = hal::gpio::gpioa::PA0<hal::gpio::Analog>;
pub type D33 = hal::gpio::gpiob::PB0<hal::gpio::Analog>;
pub type D34 = hal::gpio::gpioe::PE0<hal::gpio::Analog>;
pub type D35 = hal::gpio::gpiob::PB11<hal::gpio::Analog>;
pub type D36 = hal::gpio::gpiob::PB10<hal::gpio::Analog>;
pub type D37 = hal::gpio::gpioe::PE15<hal::gpio::Analog>;
pub type D38 = hal::gpio::gpioe::PE6<hal::gpio::Analog>;
pub type D39 = hal::gpio::gpioe::PE12<hal::gpio::Analog>;

pub type D40 = hal::gpio::gpioe::PE10<hal::gpio::Analog>;
pub type D41 = hal::gpio::gpioe::PE7<hal::gpio::Analog>;
pub type D42 = hal::gpio::gpioe::PE8<hal::gpio::Analog>;
pub type D43 = hal::gpio::gpioc::PC8<hal::gpio::Analog>;
pub type D44 = hal::gpio::gpioc::PC9<hal::gpio::Analog>;
pub type D45 = hal::gpio::gpioc::PC10<hal::gpio::Analog>;
pub type D46 = hal::gpio::gpioc::PC11<hal::gpio::Analog>;
pub type D47 = hal::gpio::gpioc::PC12<hal::gpio::Analog>;
pub type D48 = hal::gpio::gpiod::PD2<hal::gpio::Analog>;
pub type D49 = hal::gpio::gpiog::PG10<hal::gpio::Analog>;

pub type D50 = hal::gpio::gpiog::PG8<hal::gpio::Analog>;
pub type D51 = hal::gpio::gpiod::PD7<hal::gpio::Analog>;
pub type D52 = hal::gpio::gpiod::PD6<hal::gpio::Analog>;
pub type D53 = hal::gpio::gpiod::PD5<hal::gpio::Analog>;
pub type D54 = hal::gpio::gpiod::PD4<hal::gpio::Analog>;
pub type D55 = hal::gpio::gpiod::PD3<hal::gpio::Analog>;
pub type D56 = hal::gpio::gpioe::PE2<hal::gpio::Analog>;
pub type D57 = hal::gpio::gpioe::PE4<hal::gpio::Analog>;
pub type D58 = hal::gpio::gpioe::PE5<hal::gpio::Analog>;
pub type D59 = hal::gpio::gpioe::PE6<hal::gpio::Analog>;

pub type D60 = hal::gpio::gpioe::PE3<hal::gpio::Analog>;
pub type D61 = hal::gpio::gpiof::PF8<hal::gpio::Analog>;
pub type D62 = hal::gpio::gpiof::PF7<hal::gpio::Analog>;
pub type D63 = hal::gpio::gpiof::PF9<hal::gpio::Analog>;
pub type D64 = hal::gpio::gpiod::PD10<hal::gpio::Analog>;
pub type D65 = hal::gpio::gpiob::PB14<hal::gpio::Analog>;
pub type D66 = hal::gpio::gpiod::PD1<hal::gpio::Analog>;
pub type D67 = hal::gpio::gpiod::PD0<hal::gpio::Analog>;
pub type D68 = hal::gpio::gpiof::PF15<hal::gpio::Analog>;
pub type D69 = hal::gpio::gpiof::PF14<hal::gpio::Analog>;

pub type D70 = hal::gpio::gpiob::PB5<hal::gpio::Analog>;
pub type D71 = hal::gpio::gpioe::PE9<hal::gpio::Analog>;
pub type D72 = hal::gpio::gpiob::PB2<hal::gpio::Analog>;

pub mod user_button {
    use stm32h7xx_hal as hal;

    #[cfg(not(feature = "button-1-pa0"))] // SB81=off, SB82=on
    pub type Pin = hal::gpio::gpioc::PC13<hal::gpio::Analog>;
    #[cfg(any(feature = "button-1-pa0"))] // SB81=on,  SB82=off
    pub type Pin = hal::gpio::gpioa::PA0<hal::gpio::Analog>;
}

pub mod user_leds {
    use hal::gpio::{Output, PushPull};
    use stm32h7xx_hal as hal;

    #[cfg(not(feature = "led-1-pa5"))] // SB65=off, SB54=on
    pub type Pin1 = hal::gpio::gpiob::PB0<hal::gpio::Analog>;
    #[cfg(any(feature = "led-1-pa5"))] // SB65=on,  SB54=off
    pub type Pin1 = hal::gpio::gpioa::PA5<hal::gpio::Analog>;
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
        pub ref_clk: hal::gpio::gpioa::PA1<hal::gpio::Analog>, // REFCLK,   // RmiiRefClk
        pub md_io: hal::gpio::gpioa::PA2<hal::gpio::Analog>,   // IO,       // MDIO
        pub md_clk: hal::gpio::gpioc::PC1<hal::gpio::Analog>,  // CLK,      // MDC
        pub crs: hal::gpio::gpioa::PA7<hal::gpio::Analog>,     // CRS,      // RmiiCrsDv
        pub rx_d0: hal::gpio::gpioc::PC4<hal::gpio::Analog>,   // RXD0,     // RmiiRxD0
        pub rx_d1: hal::gpio::gpioc::PC5<hal::gpio::Analog>,   // RXD1,     // RmiiRxD0
        pub tx_en: hal::gpio::gpiog::PG11<hal::gpio::Analog>,  // TXEN,     // RmiiTxEN
        pub tx_d0: hal::gpio::gpiog::PG13<hal::gpio::Analog>,  // TXD0,     // RmiiTxD0
        pub tx_d1: hal::gpio::gpiob::PB13<hal::gpio::Analog>,  // TXD1,     // RmiiTxD1
    }
}

// - Pins ---------------------------------------------------------------------

pub struct Pins {
    // ST Zio "Arduino" pins
    pub a0: A0,
    pub a1: A1,
    pub a2: A2,
    pub a3: A3,
    pub a4: A4,
    pub a5: A5,
    pub a6: A6,
    pub a7: A7,
    //pub a8: A8, // used by ethernet (PA2)
    pub d0: D0,
    pub d1: D1,
    pub d2: D2,
    pub d3: D3,
    pub d4: D4,
    pub d5: D5,
    pub d6: D6,
    pub d7: D7,
    pub d8: D8,
    pub d9: D9,
    pub d10: D10,
    pub d11: D11,
    pub d12: D12,
    #[cfg(not(feature = "led-1-pa5"))]
    pub d13: D13, // used by ld1 alt. (PA5)
    pub d14: D14,
    pub d15: D15,
    pub d16: D16,
    pub d17: D17,
    //pub d18: D18, // used by ethernet (PB13)
    pub d19: D19,
    pub d20: D20,
    pub d21: D21,
    //pub d22: D22, // duplicated by d11 (PB5)
    pub d23: D23,
    pub d24: D24,
    pub d25: D25,
    pub d26: D26,
    pub d27: D27,
    pub d28: D28,
    pub d29: D29,
    pub d30: D30,
    pub d31: D31,
    #[cfg(not(feature = "button-1-pa0"))]
    pub d32: D32, // used by b1 (PA0)
    #[cfg(any(feature = "led-1-pa5"))]
    pub d33: D33, // used by ld1 (PB0)
    pub d34: D34,
    pub d35: D35,
    pub d36: D36,
    pub d37: D37,
    pub d38: D38,
    pub d39: D39,
    pub d40: D40,
    pub d41: D41,
    pub d42: D42,
    pub d43: D43,
    pub d44: D44,
    pub d45: D45,
    pub d46: D46,
    pub d47: D47,
    pub d48: D48,
    pub d49: D49,
    pub d50: D50,
    pub d51: D51,
    pub d52: D52,
    pub d53: D53,
    pub d54: D54,
    pub d55: D55,
    //pub d56: D56, // duplicated by d31 (PE2)
    pub d57: D57,
    pub d58: D58,
    //pub d59: D59, // duplicated by d38 (PE6)
    pub d60: D60,
    pub d61: D61,
    pub d62: D62,
    pub d63: D63,
    pub d64: D64,
    //pub d65: D65, // used by ld3 (PB14)
    pub d66: D66,
    pub d67: D67,
    pub d68: D68,
    pub d69: D69,
    //pub d70: D70, // duplicated by d11 (PB5)
    pub d71: D71,
    //pub d72: D72, // duplicated by d27 (PB2)

    // board peripherals
    pub ethernet: ethernet::Pins,
    pub user_button: user_button::Pin,
    pub user_leds: user_leds::Pins,
    // TODO
}

// - construction -------------------------------------------------------------

impl Pins {
    pub fn new(
        gpioa: hal::gpio::gpioa::Parts,
        gpiob: hal::gpio::gpiob::Parts,
        gpioc: hal::gpio::gpioc::Parts,
        gpiod: hal::gpio::gpiod::Parts,
        gpioe: hal::gpio::gpioe::Parts,
        gpiof: hal::gpio::gpiof::Parts,
        gpiog: hal::gpio::gpiog::Parts,
    ) -> Self {
        Self {
            a0: gpioa.pa3,
            a1: gpioc.pc0,
            a2: gpioc.pc3,
            a3: gpiob.pb1,
            a4: gpioc.pc2,
            a5: gpiof.pf11,
            a6: gpiof.pf6,
            a7: gpiof.pf10,
            //a8: gpioa.pa2,
            d0: gpiob.pb7,
            d1: gpiob.pb6,
            d2: gpiog.pg14,
            d3: gpioe.pe13,
            d4: gpioe.pe14,
            d5: gpioe.pe11,
            d6: gpioa.pa8,
            d7: gpiog.pg12,
            d8: gpiog.pg9,
            d9: gpiod.pd15,

            d10: gpiod.pd14,
            d11: gpiob.pb5,
            d12: gpioa.pa6,
            #[cfg(not(feature = "led-1-pa5"))]
            d13: gpioa.pa5,
            d14: gpiob.pb9,
            d15: gpiob.pb8,
            d16: gpioc.pc6,
            d17: gpiob.pb15,
            //d18: gpiob.pb13,
            d19: gpiob.pb12,

            d20: gpioa.pa15,
            d21: gpioc.pc7,
            //d22: gpiob.pb5,
            d23: gpiob.pb3,
            d24: gpioa.pa4,
            d25: gpiob.pb4,
            d26: gpiog.pg6,
            d27: gpiob.pb2,
            d28: gpiod.pd13,
            d29: gpiod.pd12,

            d30: gpiod.pd11,
            d31: gpioe.pe2,
            #[cfg(not(feature = "button-1-pa0"))]
            d32: gpioa.pa0,
            #[cfg(any(feature = "led-1-pa5"))]
            d33: gpiob.pb0,
            d34: gpioe.pe0,
            d35: gpiob.pb11,
            d36: gpiob.pb10,
            d37: gpioe.pe15,
            d38: gpioe.pe6,
            d39: gpioe.pe12,

            d40: gpioe.pe10,
            d41: gpioe.pe7,
            d42: gpioe.pe8,
            d43: gpioc.pc8,
            d44: gpioc.pc9,
            d45: gpioc.pc10,
            d46: gpioc.pc11,
            d47: gpioc.pc12,
            d48: gpiod.pd2,
            d49: gpiog.pg10,

            d50: gpiog.pg8,
            d51: gpiod.pd7,
            d52: gpiod.pd6,
            d53: gpiod.pd5,
            d54: gpiod.pd4,
            d55: gpiod.pd3,
            //d56: gpioe.pe2,
            d57: gpioe.pe4,
            d58: gpioe.pe5,
            //d59: gpioe.pe6,
            d60: gpioe.pe3,
            d61: gpiof.pf8,
            d62: gpiof.pf7,
            d63: gpiof.pf9,
            d64: gpiod.pd10,
            //d65: gpiob.pb14,
            d66: gpiod.pd1,
            d67: gpiod.pd0,
            d68: gpiof.pf15,
            d69: gpiof.pf14,

            //d70: gpiob.pb5,
            d71: gpioe.pe9,
            //d72: gpiob.pb2,
            ethernet: ethernet::Pins {
                ref_clk: gpioa.pa1,
                md_io: gpioa.pa2,
                md_clk: gpioc.pc1,
                crs: gpioa.pa7,
                rx_d0: gpioc.pc4,
                rx_d1: gpioc.pc5,
                tx_en: gpiog.pg11,
                tx_d0: gpiog.pg13,
                tx_d1: gpiob.pb13,
            },

            #[cfg(not(feature = "button-1-pa0"))]
            user_button: gpioc.pc13,
            #[cfg(any(feature = "button-1-pa0"))]
            user_button: gpioa.pa0,

            user_leds: user_leds::Pins {
                #[cfg(not(feature = "led-1-pa5"))]
                ld1: gpiob.pb0,
                #[cfg(any(feature = "led-1-pa5"))]
                ld1: gpioa.pa5,
                ld2: gpioe.pe1,
                ld3: gpiob.pb14,
            },
        }
    }
}
