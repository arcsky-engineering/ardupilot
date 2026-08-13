# Arcsky ArduPilot Fork

This is Arcsky's fork of [ArduPilot](https://github.com/ArduPilot/ardupilot). It contains custom ArduCopter modifications for Arcsky products, maintained as branches off specific upstream stable releases.

## Branches

### Active Product Branches

| Branch | Product | Base Version | Description |
|---|---|---|---|
| `xplorer/4.6.2-custom` | Xplorer | ArduCopter 4.6.2 stable | Current Xplorer production firmware with Arcsky customizations |
| `x55/4.4.4-custom` | X55 / Hybrid Module | ArduCopter 4.4.4 stable | Current X55 production firmware with Arcsky customizations |

### Legacy Branches

| Branch | Product | Base Version | Description |
|---|---|---|---|
| `4-3-7-custom` | X55 / Hybrid Module | ArduCopter 4.3.7 stable | Older X55 firmware — superseded by `x55/4.4.4-custom` |

### Upstream

| Branch | Description |
|---|---|
| `master` | Tracks upstream ArduPilot master. Do not commit Arcsky changes here. |

## Local Setup

This repo is large. To keep X55 and Xplorer work separate without switching branches, clone it twice into different local folders:

```
C:\GitHub\ardupilot-xplorer\           → checked out on xplorer/4.6.2-custom
C:\GitHub\ardupilot-x55\ardupilot\     → checked out on x55/4.4.4-custom
```

Both point to the same remote (`arcsky-engineering/ardupilot`). Commits on one clone are visible to the other after a `git fetch`.

## Firmware Versioning

Each product carries its own Arcsky version string, independent of the upstream ArduPilot version it is based on. This is set via ArduPilot's `AP_CUSTOM_FIRMWARE_STRING` vendor hook in the board hwdef — **never** by editing `ArduCopter/version.h`, whose `THISFIRMWARE` and `FIRMWARE_VERSION` values must stay at their upstream values so ground stations load the right parameter metadata.

| Product | Reports as | Version defined in |
|---|---|---|
| X55 | `X55 v1.0.0 (<hash>) [ArduCopter V4.4.4]` | `libraries/AP_HAL_ChibiOS/hwdef/include/x55_version.inc` |
| Xplorer | `Xplorer v1.0.1 (<hash>) [ArduCopter V4.6.2]` | `libraries/AP_HAL_ChibiOS/hwdef/CubeOrangePlus/hwdef.dat` |

The bracketed upstream baseline appears in the dataflash `MSG` line, so any flight log identifies both the Arcsky version and the ArduPilot release under it.

Every release gets a tag — `x55-fw-v<X.Y.Z>` / `xplorer-fw-v<X.Y.Z>` — so a version string seen on a fielded unit resolves to exact source. On the X55 branch, `Tools/x55/build_release.sh` enforces this (clean tree, matching tag, changelog entry, version string verified present in the compiled `hwdef.h`) and writes a per-release `MANIFEST.txt` with sha256 sums. See [`Tools/x55/README.md`](Tools/x55/README.md) and [`doc/X55-FIRMWARE-CHANGELOG.md`](doc/X55-FIRMWARE-CHANGELOG.md).

Note that `GIT_VERSION` is a bare short hash with no dirty marker, so a binary built from an uncommitted tree advertises a hash whose code it is not. Never hand a dirty-tree build to a customer.

## Types of Customizations

Arcsky's patches include modifications such as:
- Secondary power supply safety checks and pre-arm modifications
- Battery data handling for the Xplorer BMS (DroneCAN cell-level data, temperatures, error messages)
- Remote ID integration (OpenDroneID message formatting and provisioning for DB201)
- Generator interaction and control (Hybrid Module — throttle management, RPM monitoring, power mode transitions)
- Forward obstacle avoidance implementation
- Custom startup tones and debug message filtering
- Rangefinder-based landing speed control

See the commit history on each product branch for the full list of changes.

## Pulling Upstream Updates

When moving to a newer ArduPilot baseline:

```bash
git fetch upstream
git checkout xplorer/4.6.2-custom
git log --oneline upstream/Copter-4.6.2..HEAD   # Review Arcsky patches
```

Then create a new branch off the target upstream release and cherry-pick or rebase the Arcsky patches onto it. Test thoroughly on SITL before deploying to hardware.

## Build Instructions

**X55: start with [`doc/X55-DEV-WORKFLOW.md`](doc/X55-DEV-WORKFLOW.md)** — one-time
setup, the change/test loop, and the release process end to end. Note in particular
the Cygwin git line-ending fix, which will otherwise show ~4500 files as modified
on your first `git status`.

See the Arcsky Engineering Handbook for:
- Windows build environment setup (waf, ARM GCC toolchain, Anaconda Python considerations)
- SITL testing procedures
- Firmware deployment process

ArduPilot's own build documentation: https://ardupilot.org/dev/docs/building-the-code.html
