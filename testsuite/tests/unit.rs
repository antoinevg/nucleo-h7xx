#![no_std]
#![no_main]

use testsuite as _; // memory layout + panic handler + global logger


#[defmt_test::tests]
mod tests {
    use defmt::{assert, assert_eq};

    #[test]
    fn assert_true() {
        assert!(true)
    }

    #[test]
    fn assert_eq() {
        assert_eq!(24, 42, "TODO: write actual tests")
    }
}
