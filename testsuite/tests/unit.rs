#![no_std]
#![no_main]


#[defmt_test::tests]
mod tests {
    use defmt::{assert, assert_eq};

    #[init]
    fn init() -> testsuite::State {
        testsuite::State::init()
    }

    #[test]
    fn assert_true() {
        assert!(true)
    }

    #[test]
    fn assert_eq() {
        assert_eq!(true, true, "TODO: write actual tests")
    }

    #[test]
    fn assert_state(state: &mut testsuite::State) {
        assert!(state.flag);
        state.flag = false;
    }

    #[test]
    fn assert_state_changed(state: &mut testsuite::State) {
        assert_eq!(state.flag, false);
    }

    #[test]
    fn assert_board_clocks(state: &mut testsuite::State) {
        let clocks = state.clocks;

        assert_eq!(clocks.hclk().0,  240_000_000, "AHB1,2,3");
        assert_eq!(clocks.aclk().0,  240_000_000, "AXI");
        assert_eq!(clocks.pclk1().0, 120_000_000, "APB1");
        assert_eq!(clocks.ppre1(),   2,           "APB1");
        assert_eq!(clocks.pclk2().0, 120_000_000, "APB2");
        assert_eq!(clocks.ppre2(),   2,           "APB2");
        assert_eq!(clocks.pclk3().0, 120_000_000, "APB3");
        assert_eq!(clocks.ppre3(),   2,           "APB3");
        assert_eq!(clocks.pclk4().0, 120_000_000, "APB4");
        assert_eq!(clocks.ppre4(),   2,           "APB4");

        assert_eq!(defmt::unwrap!(clocks.csi_ck()).0,     4_000_000);
        assert_eq!(defmt::unwrap!(clocks.hsi_ck()).0,    64_000_000);
        assert_eq!(defmt::unwrap!(clocks.hsi48_ck()).0,  48_000_000);
        assert_eq!(defmt::unwrap!(clocks.per_ck()).0,    64_000_000);
        assert!(clocks.hse_ck().is_none());
        assert_eq!(defmt::unwrap!(clocks.lsi_ck()).0,        32_000);
        assert!(clocks.mco1_ck().is_none());
        assert!(clocks.mco2_ck().is_none());
        assert_eq!(defmt::unwrap!(clocks.pll1_p_ck()).0, 480_000_000);
        assert!(clocks.pll1_q_ck().is_none());
        assert_eq!(defmt::unwrap!(clocks.pll1_r_ck()).0, 240_000_000);
        assert!(clocks.pll2_p_ck().is_none());
        assert!(clocks.pll2_q_ck().is_none());
        assert!(clocks.pll2_r_ck().is_none());
        assert_eq!(defmt::unwrap!(clocks.pll3_p_ck()).0,  12_235_294);
        assert!(clocks.pll3_q_ck().is_none());
        assert!(clocks.pll3_r_ck().is_none());

        assert_eq!(clocks.sys_ck().0,      480_000_000, "SCGU");
        assert_eq!(clocks.sysclk().0,      clocks.sys_ck().0); // alias for sys_ck
        assert_eq!(clocks.timx_ker_ck().0, 240_000_000, "APB1");
        assert_eq!(clocks.timy_ker_ck().0, 240_000_000, "APB2");
        assert_eq!(clocks.c_ck().0,        480_000_000, "Core");
    }
}
