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

## v1.0.2 — unreleased

### Fixed
- Gimbal and camera MAVLink traffic could be throttled to roughly 75% of its
  normal rate (Gremsy attitude at 7-8 Hz instead of 10 Hz, camera messages at
  0.7-0.8 Hz instead of 1 Hz), with occasional missed commands such as an
  autofocus tap. Cause was `SR*_PARAMS` sitting at 0, which is ArduPilot's
  upstream default. Parameter downloads are driven by that stream rate, so at 0
  a download can never complete; the ground station retries indefinitely, and
  each failed fetch renews a 100-byte payload reservation
  (`GCS_MAVLink/GCS_Signing.cpp`) that is shared across every channel and steals
  bandwidth from all other traffic, including messages the autopilot is
  forwarding from the gimbal and camera. All `SR*_PARAMS` are now seeded to 10
  and left writable. The rate only produces traffic while a download is pending,
  so it is free at idle.

  Only affected aircraft where a ground station had never explicitly written
  `SR1_PARAMS` — stored values survive a firmware update, so airframes whose
  rates had been set during commissioning were unaffected and looked identical
  in every other respect. Any change to the parameter count can trigger it,
  because a changed count makes a GCS re-download the whole set; in this release
  the trigger was the new BATT SOC and FFT parameters.

- Remote ID pre-arm failures now name the actual problem instead of echoing a raw
  status code. The DB300 module reports its status as a number rather than the
  text the DB201 used, so a missing GPS fix reached the pilot as
  `PreArm: Remote I D: err 8`; it now reads `No Drone Location or GPS fix`.
  Missing BasicID, missing operator location, a missing System message and a
  disabled transmitter are decoded the same way. DB201 modules are unaffected —
  their text codes still translate exactly as before, so one build covers both.

  Not yet confirmed against DB300 hardware; the code mapping is taken from the
  DB300 manual v1.3 and matches the X55 firmware, which uses the same module.

- Arming is blocked again when the ground station has no operator location, or
  has stopped reporting one. Remote ID broadcasts must carry the operator's
  position; when the GCS has no fix it does not omit the field, it sends
  latitude/longitude 0, so the aircraft was transmitting an operator location of
  0,0 and nothing objected. The autopilot had been leaving this check to the
  Remote ID module, which the DB201 did perform and the DB300 is not confirmed
  to. Pre-arm now reports `No Operator Location`, or `Operator Location stale`
  if the ground station link has gone quiet for more than 3 seconds while a
  previously good position is still being rebroadcast.

  Bench testing indoors will now fail this check when neither the controller nor
  the aircraft has a GPS fix. That is the check working — take the aircraft
  outside, or clear the `EnforceArming` bit in `DID_OPTIONS` for bench work.

### Changed
- `SR5_PARAMS` and `SR6_PARAMS` are no longer read-only. A stream rate locked at
  0 means that channel can never be recovered in the field, and the DEV unlock
  list does not cover `SR*`.

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
