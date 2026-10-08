# Changelog

## Unreleased

### Added

- `PestoLink_OneMotor` example: one gamepad axis drives one ESC over
  Bluetooth, with the ESC's telemetry in PestoLink's terminal.

## 1.1 - 2026-10-05

### Added

- `setPushPull()`: drives each frame push-pull, then releases the line for the
  ESC's reply. This lets weak pull-ups (5-10k) work, which ESCs with a series
  resistor on their signal pad need. Off by default.
- `Rotini_V4_Telemetry` example: drives two ESCs from the AlfredoTelemetry web
  page and plots RPM, link loss, temperature, voltage and current. It sends
  3D-mode throttle (-100 to 100 %), so the ESCs need 3D mode on.
- `AM32_ConfiguratorLink`: `ENTER_BOOTLOADER` option for boards that power the
  ESP and the ESC together. It sends 4 s of zero-throttle DShot at boot so a
  running ESC drops into its bootloader. On by default, so the USB port now
  takes about 7 s to appear.
- README: how to size the pull-up when the ESC has a series resistor.

### Changed

- `current()` now reads 1 A steps to match AM32 2.21. On AM32 2.20 and older it
  reads double the real current.
- Wiring diagrams and the README use a 100 ohm series resistor everywhere.

### Fixed

- Telemetry at DShot300 with AM32. AM32 replies at the DShot600 rate there, so
  the decoder now also tries half and double the expected bit time.

## 1.0.0

First release.
