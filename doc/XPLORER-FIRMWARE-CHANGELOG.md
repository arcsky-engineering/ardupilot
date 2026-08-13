# Xplorer Firmware Changelog

Source of truth for what changed in each Xplorer firmware release. The customer
release notes are **generated** from this file by
`Tools/xplorer/gen_release_notes.py`, so an entry missing here is a release note
missing in the field — `Tools/xplorer/build_release.sh` fails the build rather
than let that happen.

Version string lives in
[`libraries/AP_HAL_ChibiOS/hwdef/include/xplorer_version.inc`](../libraries/AP_HAL_ChibiOS/hwdef/include/xplorer_version.inc)
and reaches the boot banner, the `AUTOPILOT_VERSION` MAVLink message and the
dataflash `MSG` line (as `Xplorer v<X.Y.Z> [ArduCopter V4.6.2]`).

Format: one `## vX.Y.Z — YYYY-MM-DD` heading per release, newest first, then an
optional `###` headline banner and the body. Group changes under `### Added`,
`### Fixed`, `### Changed` and `### Known issues`. Keep entries operator-facing —
say what changed for someone flying or supporting the aircraft, not which
function was edited.

Versioning: bump **patch** for fixes, **minor** for new behaviour or new
parameters, **major** when a release needs a deliberate parameter migration or is
not field-upgradable in place.

---

## v1.0.1 — 2026-08-13

### Added
- Engineering DEV firmware target with PSC/ATC/EK3/notch/FFT parameters unlocked for field tuning. Announces itself at WARNING level on boot and must never be published to the release channel.
- Also created versioning and build tools (including exe) that can be used to make the process easier

---

## v1.0.0 — 2026-01-30

### FIRST VERSIONED XPLORER FIRMWARE

### Added
- First versioned Xplorer firmware. Custom version string via
  `AP_CUSTOM_FIRMWARE_STRING`, reported in the boot banner and dataflash logs.
- Remote ID integration for the DB201, including provisioning and periodic
  metadata output.
- Xplorer BMS support over DroneCAN: cell-level data, temperatures and error
  messages.
- Forward obstacle avoidance, and rangefinder-based landing speed control.
- Read-only parameter enforcement plus a hard clamp table, so operators cannot
  set factory-configured or out-of-range values from a GCS.
