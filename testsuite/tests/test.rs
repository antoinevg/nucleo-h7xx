#![no_std]
#![no_main]

use nucleo_h745zi as _;
use panic_probe as _;   // panic handler
use defmt_rtt as _;     // global logger

use core::sync::atomic::{AtomicUsize, Ordering};


#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}


static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});


// See https://crates.io/crates/defmt-test/0.1.0 for more documentation (e.g. about the 'state'
// feature)
#[defmt_test::tests]
mod tests {
    use defmt::{assert, assert_eq};

    #[test]
    fn assert_true() {
        assert!(true)
    }

    #[test]
    fn assert_eq() {
        assert_eq!(24, 24, "TODO: write actual tests")
    }
}
