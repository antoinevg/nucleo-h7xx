use crate::hal;
use hal::prelude::*;
use hal::pac;
use hal::pwr;
use hal::rcc;
use hal::time::Hertz;
use hal::time::MegaHertz;


// - constants ----------------------------------------------------------------

// SAI clock uses pll3
const PLL3_P: Hertz = Hertz(48_000 * 256);


// - types --------------------------------------------------------------------

pub trait HseCrystal {
    // by default it uses the MCO output of the ST-Link which has a fixed frequency of 8 MHz
    // TODO Add a feature to support the onboard HSE oscillator from X2, a 25 MHz crystal.
    const CRYSTAL_FREQ: MegaHertz = MegaHertz(8);
    fn use_hse_crystal(self) -> Self;
}

impl HseCrystal for rcc::Rcc {
    fn use_hse_crystal(self) -> Self {
        self.use_hse(Self::CRYSTAL_FREQ)
    }
}


// - configure ----------------------------------------------------------------

/// Configures system clocks:
///
///   HSE crystal
///   480 MHz system clock
///   PLL3 for SAI audio
///
/// Usage:
///
/// ```
/// let dp = pac::Peripherals::take().unwrap();
/// let ccdr = configure(dp.PWR.constrain(),
///                      dp.RCC.constrain(),
///                      &dp.SYSCFG);
/// ```
pub fn configure(pwr: pwr::Pwr, rcc: rcc::Rcc, syscfg: &pac::SYSCFG) -> rcc::Ccdr {
    let pwrcfg = pwr.smps().vos0(syscfg).freeze();
    rcc.sys_ck(480.mhz())                                // system clock @ 480 MHz
       .pll1_strategy(rcc::PllConfigStrategy::Iterative) // pll1 drives system clock
       .pll1_r_ck(480.mhz())                             // for TRACECLK - TODO feature gate it for ITM
       .pll3_p_ck(PLL3_P)
       //.use_hse_crystal()  // TODO
       .freeze(pwrcfg, syscfg)
}
