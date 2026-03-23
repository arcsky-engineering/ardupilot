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
C:\GitHub\ardupilot-xplorer\    → checked out on xplorer/4.6.2-custom
C:\GitHub\ardupilot-x55\        → checked out on x55/4.4.4-custom
```

Both point to the same remote (`arcsky-engineering/ardupilot`). Commits on one clone are visible to the other after a `git fetch`.

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

See the Arcsky Engineering Handbook for:
- Windows build environment setup (waf, ARM GCC toolchain, Anaconda Python considerations)
- SITL testing procedures
- Firmware deployment process

ArduPilot's own build documentation: https://ardupilot.org/dev/docs/building-the-code.html
