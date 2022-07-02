# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!--
## [Unreleased]
-->

## [0.2.0] - 2022-07-02
### Changed
- Update `cortex-m` to `0.7.5`
- Update `cortex-m-rt` to `0.7.1`
- Update `cortex-m-semihosting` to `0.5.0`
- Update `stm32h7xx-hal` to `0.9.0`
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


[0.1.0]: https://github.com/antoinevg/nucleo-h7xx/releases/tag/v0.1.0
