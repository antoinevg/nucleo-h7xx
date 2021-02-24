# We need to pass the .cargo/config.toml info using environment
# variables for sub-projects as cargo configs in parent directories
# turn the local config into garbage.
#
# Possibly related: https://github.com/rust-lang/cargo/issues/7004

TEST_FLAGS = CARGO_TARGET_THUMBV7EM_NONE_EABIHF_RUSTFLAGS="-C link-arg=-Tdefmt.x" \
			 CARGO_TARGET_THUMBV7EM_NONE_EABIHF_RUNNER="probe-run --chip STM32H745ZITx" \
			 CARGO_BUILD_TARGET="thumbv7em-none-eabihf"

test:
	${TEST_FLAGS} cargo test
