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

        assert_eq!(clocks.hclk().raw(),  240_000_000, "AHB1,2,3");
        assert_eq!(clocks.aclk().raw(),  240_000_000, "AXI");
        assert_eq!(clocks.pclk1().raw(), 120_000_000, "APB1");
        assert_eq!(clocks.ppre1(),   2,           "APB1");
        assert_eq!(clocks.pclk2().raw(), 120_000_000, "APB2");
        assert_eq!(clocks.ppre2(),   2,           "APB2");
        assert_eq!(clocks.pclk3().raw(), 120_000_000, "APB3");
        assert_eq!(clocks.ppre3(),   2,           "APB3");
        assert_eq!(clocks.pclk4().raw(), 120_000_000, "APB4");
        assert_eq!(clocks.ppre4(),   2,           "APB4");

        assert_eq!(defmt::unwrap!(clocks.csi_ck()).raw(),     4_000_000);
        assert_eq!(defmt::unwrap!(clocks.hsi_ck()).raw(),    64_000_000);
        assert_eq!(defmt::unwrap!(clocks.hsi48_ck()).raw(),  48_000_000);
        assert_eq!(defmt::unwrap!(clocks.per_ck()).raw(),    64_000_000);
        assert!(clocks.hse_ck().is_none());
        assert_eq!(defmt::unwrap!(clocks.lsi_ck()).raw(),        32_000);
        assert!(clocks.mco1_ck().is_none());
        assert!(clocks.mco2_ck().is_none());
        assert_eq!(defmt::unwrap!(clocks.pll1_p_ck()).raw(), 480_000_000);
        assert!(clocks.pll1_q_ck().is_none());
        assert_eq!(defmt::unwrap!(clocks.pll1_r_ck()).raw(), 240_000_000);
        assert!(clocks.pll2_p_ck().is_none());
        assert!(clocks.pll2_q_ck().is_none());
        assert!(clocks.pll2_r_ck().is_none());
        assert_eq!(defmt::unwrap!(clocks.pll3_p_ck()).raw(),  12_235_294);
        assert!(clocks.pll3_q_ck().is_none());
        assert!(clocks.pll3_r_ck().is_none());

        assert_eq!(clocks.sys_ck().raw(),      480_000_000, "SCGU");
        assert_eq!(clocks.sysclk().raw(),      clocks.sys_ck().raw()); // alias for sys_ck
        assert_eq!(clocks.timx_ker_ck().raw(), 240_000_000, "APB1");
        assert_eq!(clocks.timy_ker_ck().raw(), 240_000_000, "APB2");
        assert_eq!(clocks.c_ck().raw(),        480_000_000, "Core");
    }
}
