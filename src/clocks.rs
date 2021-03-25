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
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    let dp = unsafe { pac::Peripherals::steal() };

    // link SRAM3 power state to CPU1
    dp.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

    let pwrcfg = pwr.smps().vos0(syscfg).freeze();

    #[cfg(not(feature = "log-itm"))]
    let ccdr = rcc.sys_ck(480.mhz())                      // system clock @ 480 MHz
        .pll1_strategy(rcc::PllConfigStrategy::Iterative) // pll1 drives system clock
        .pll3_p_ck(PLL3_P)                                // sai clock @ 12.288 MHz
        //.use_hse_crystal()                              // TODO hse oscillator @ 25 MHz
        .freeze(pwrcfg, syscfg);

    #[cfg(any(feature = "log-itm"))]
    let ccdr = rcc.sys_ck(480.mhz())                      // system clock @ 480 MHz
        .pll1_strategy(rcc::PllConfigStrategy::Iterative) // pll1 drives system clock
        .pll1_r_ck(480.mhz())                             // TRACECLK
        .pll3_p_ck(PLL3_P)                                // sai clock @ 12.288 MHz
        //.use_hse_crystal()                              // TODO hse oscillator @ 25 MHz
        .freeze(pwrcfg, syscfg);

    // enable itm if the feature is selected
    #[cfg(any(feature = "log-itm"))]
    unsafe {
        let swo_frequency = 2_000_000;
        crate::itm::enable_itm(&mut cp.DCB,
                               &dp.DBGMCU,
                               &mut cp.ITM,
                               ccdr.clocks.c_ck().0,
                               swo_frequency);
    }

    // configure cpu
    cp.SCB.invalidate_icache();
    cp.SCB.enable_icache();
    cp.DWT.enable_cycle_counter();

    ccdr
}


pub fn log_clocks(clocks: &hal::rcc::CoreClocks) {
    use crate::loggit;

    loggit!("AHB1,2,3,4 hclk: {}", clocks.hclk().0);
    loggit!("AXI aclk: {}", clocks.aclk().0);
    loggit!("APB1 pclk1: {}", clocks.pclk1().0);
    loggit!("APB1 ppre1: {}", clocks.ppre1());
    loggit!("APB2 pclk2: {}", clocks.pclk2().0);
    loggit!("APB2 ppre2: {}", clocks.ppre2());
    loggit!("APB3 pclk3: {}", clocks.pclk3().0);
    loggit!("APB3 ppre3: {}", clocks.ppre3());
    loggit!("APB4 pclk4: {}", clocks.pclk4().0);
    loggit!("APB4 ppre4: {}", clocks.ppre4());

    loggit!("csi_ck: {}", clocks.csi_ck().unwrap_or(0.hz()).0);
    loggit!("hsi_ck: {}", clocks.hsi_ck().unwrap_or(0.hz()).0);
    loggit!("hsi48_ck: {}", clocks.hsi48_ck().unwrap_or(0.hz()).0);
    loggit!("per_ck: {}", clocks.per_ck().unwrap_or(0.hz()).0);
    loggit!("hse_ck: {}", clocks.hse_ck().unwrap_or(0.hz()).0);
    loggit!("lsi_ck: {}", clocks.lsi_ck().unwrap_or(0.hz()).0);
    loggit!("mco1_ck: {}", clocks.mco1_ck().unwrap_or(0.hz()).0);
    loggit!("mco2_ck: {}", clocks.mco2_ck().unwrap_or(0.hz()).0);
    loggit!("pll1_p_ck: {}", clocks.pll1_p_ck().unwrap_or(0.hz()).0);
    loggit!("pll1_q_ck: {}", clocks.pll1_q_ck().unwrap_or(0.hz()).0);
    loggit!("pll1_r_ck: {}", clocks.pll1_r_ck().unwrap_or(0.hz()).0);
    loggit!("pll2_p_ck: {}", clocks.pll2_p_ck().unwrap_or(0.hz()).0);
    loggit!("pll2_q_ck: {}", clocks.pll2_q_ck().unwrap_or(0.hz()).0);
    loggit!("pll2_r_ck: {}", clocks.pll2_r_ck().unwrap_or(0.hz()).0);
    loggit!("pll3_p_ck: {}", clocks.pll3_p_ck().unwrap_or(0.hz()).0);
    loggit!("pll3_q_ck: {}", clocks.pll3_q_ck().unwrap_or(0.hz()).0);
    loggit!("pll3_r_ck: {}", clocks.pll3_r_ck().unwrap_or(0.hz()).0);

    loggit!("SCGU sys_ck: {}", clocks.sys_ck().0);
    loggit!("SCGU sysclk: {}", clocks.sysclk().0);
    loggit!("APB1 timx_ker_ck: {}", clocks.timx_ker_ck().0);
    loggit!("APB2 timy_ker_ck: {}", clocks.timy_ker_ck().0);
    loggit!("Core c_ck: {}", clocks.c_ck().0);
}
