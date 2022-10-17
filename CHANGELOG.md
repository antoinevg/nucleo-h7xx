# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!--
## [Unreleased]
-->

## [0.2.1] - 2022-10-17
### Changed
- Update `cortex-m` to `0.7.6`
- Update `stm32h7xx-hal` to `0.13.0`
- Update `heapless` to `0.7.16`
- Update `panic-semihosting` to `0.6.0`

## [0.2.0] - 2022-07-02
### Fixed
- Panic at start due to `cortex-m-rt = 0.7.x` memory layout changes. (Ref: [rust-embedded/cortex-m #426](https://github.com/rust-embedded/cortex-m/issues/426#issuecomment-1092384050))
### Changed
- Update `cortex-m` to `0.7.5`
- Update `cortex-m-rt` to `0.7.1`
- Update `cortex-m-semihosting` to `0.5.0`
- Update `stm32h7xx-hal` to `0.12.2`
- Update `smoltcp` to `0.8.1`
- Update `defmt` to `0.3.2`
- Update `defmt-rtt` to `0.3.2`
- Update `defmt-test` to `0.3.0`
- Update `panic-probe` to `0.3.0`


## [0.1.2] - 2021-12-01
### Added
- New Board method which takes a user-provided function to configure CCDR: `freeze_clocks_with()`


## [0.1.1] - 2021-04-01
### Changed
- Changed license to MIT


## [0.1.0] - 2021-03-30
### Added
- Initial release
- Supported features are:
  * Default clock configuration @ 480 MHz
  * User LED's
  * Ethernet


[Unreleased]: https://github.com/antoinevg/nucleo-h7xx/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/antoinevg/nucleo-h7xx/compare/v0.1.2...v0.2.0
[0.1.2]: https://github.com/antoinevg/nucleo-h7xx/compare/v0.1.1...v0.1.2
[0.1.1]: https://github.com/antoinevg/nucleo-h7xx/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/antoinevg/nucleo-h7xx/releases/tag/v0.1.0
