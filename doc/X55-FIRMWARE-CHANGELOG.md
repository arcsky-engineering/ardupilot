# X55 firmware changelog

Versions here are Arcsky X55 firmware versions, not ArduPilot versions. The
version string is set in
[`libraries/AP_HAL_ChibiOS/hwdef/include/x55_version.inc`](../libraries/AP_HAL_ChibiOS/hwdef/include/x55_version.inc)
and appears on the boot banner, in `AUTOPILOT_VERSION`, and in the dataflash
`MSG` line as e.g.:

```
X55 v1.0.0 (77e5940f) [ArduCopter V4.4.4]
```

The bracketed value is the upstream ArduPilot baseline the release is built on;
the parenthesised value is the git commit. Every release below has a matching
`x55-fw-v<version>` tag, so a fielded banner resolves to exact source.

Versioning: bump **patch** for fixes, **minor** for new behaviour or new
parameters, **major** when a release needs a deliberate parameter migration or
is not field-upgradable in place.

---

## v1.0.0 — 2026-08-11

First versioned X55 release. Baseline: ArduCopter 4.4.4, branch
`x55/4.4.4-custom`, commit `77e5940f`.

This establishes the version string; the firmware content is the same as the
build that was already in production as of this date. Notable Arcsky content in
this baseline, from the branch history:

- Hybrid Module generator interaction — throttle management, RPM monitoring,
  power mode transitions
- `GEN_BAUD` parameter so this firmware stays compatible with older Hybrid
  Modules; default baud 115200, with the revised buffer read strategy that
  removed the large read latency
- "Hybrid Module Communication Restored" status message on link recovery
- Remote ID / OpenDroneID for DB300, including the `data64` metadata pathway
  alongside the `~`-prefixed variant
- `DID_*` parameters locked read-only in `CubeOrange/defaults.parm` to protect
  Remote ID integrity

Builds: `CubeOrange` (APJ 140), `CubeOrangePlus` (APJ 1063).

### Releases before v1.0.0

Earlier X55 builds were unversioned — they all reported `ArduCopter V4.4.4` and
are only distinguishable by the git hash in the banner, where one was recorded
at all. Do not attempt to retrofit version numbers onto them; treat anything
without an `X55 v…` string as pre-v1.0.0 and identify it by hash.
