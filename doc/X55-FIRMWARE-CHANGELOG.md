# X55 ArduCopter firmware changelog

**This file is the single source of truth for X55 release notes.** The
customer-facing `X55_ArduCopter_Release_Notes.txt` is *generated* from it by
`Tools/x55/gen_release_notes.py` (which `Tools/x55/build_release.sh` runs
automatically). Never hand-edit that `.txt` — edit this file and rebuild.

Versions here are Arcsky X55 firmware versions, not ArduPilot versions. The
version string is set in
[`libraries/AP_HAL_ChibiOS/hwdef/include/x55_version.inc`](../libraries/AP_HAL_ChibiOS/hwdef/include/x55_version.inc)
and appears on the boot banner, in `AUTOPILOT_VERSION`, and in the dataflash
`MSG` line as:

```
X55 v1.0.0 (ece37646) [ArduCopter V4.4.4]
```

The bracketed value is the upstream ArduPilot baseline; the parenthesised value
is the git commit. Every release from v1.0.0 onward has a matching
`x55-fw-v<version>` tag, so a fielded banner resolves to exact source.

Versioning: bump **patch** for fixes, **minor** for new behaviour or new
parameters, **major** when a release needs a deliberate parameter migration or is
not field-upgradable in place. The historical `Build N` numbers are kept as
aliases so techs who know "Build 13" can still navigate.

**Writing an entry:** one `##` heading per release, an optional `###` line for a
headline banner, then prose. Use `**Changes:**`, `**Compatibility:**` and
`**Files:**` sections. Keep the reader in mind — a tech deciding whether to
flash this, and a support engineer six months from now working out why a unit
behaves the way it does.

Builds before the 4.4.4 port (4.3.5, 4.3.7) are not recorded here; see the
`4-3-7-custom` branch history.

---

## v1.0.0 — Build 15 — 2026-08-11

### REMOTE ID DB300 / CUBE ID SUPPORT + FIRMWARE VERSIONING

**Commit:** `ece37646`

First versioned X55 release. From this build on, the firmware identifies itself
as `X55 v<X.Y.Z>` rather than `ArduCopter V4.4.4`, and every release carries a
git tag. See `Tools/x55/README.md` for the release process.

The flight-code content of this release is Build 14 plus the DB300 Remote ID
work, which landed 2026-06-19 in commit `77e5940f` and was not previously
written up.

**Changes:**

- Remote ID SelfID and OperatorID are forwarded to the module again, over both
  DroneCAN and the static-out MAVLink path. This reverses the Build 9 decision
  to stop sending them: the DB300 and Cube ID modules refuse to operate until
  they have received both.
- ArduPilot now originates generic placeholder values so such modules work out
  of the box, via `set_self_id_operator_id_defaults()`:
    - SelfID description = `"X55"`, type `MAV_ODID_DESC_TYPE_TEXT`
    - OperatorID = `"001"`, type `MAV_ODID_OPERATOR_ID_TYPE_CAA`
  Both are build-overridable (`OPENDRONEID_SELF_ID_DEFAULT`,
  `OPENDRONEID_OPERATOR_ID_DEFAULT`). A GCS can override either at runtime by
  sending `OPEN_DRONE_ID_SELF_ID` / `OPEN_DRONE_ID_OPERATOR_ID`; once a real
  value arrives the non-empty field is left untouched.
- Arm-status error decoding now handles both module families. The DB201 sends
  text tokens (`LOC`, `SYS`, `OP_LOC`, `ID`); the DB300 instead reports a
  numeric bitmask such as `err 8`. The first integer found in the error string
  is decoded per the DB300 manual v1.3 section 1.8:
    - `1`  = transmission disabled       --> "Remote I D Tx Disabled"
    - `2`  = no BasicID received         --> "Basic I D not Configured"
    - `4`  = no OperatorID received      --> "No Operator Location"
    - `8`  = no drone Location / no fix  --> "No Drone Location or GPS fix"
    - `16` = no System msg / no GPS info --> "No System Message or GPS info"
  Codes combine (10 = 2|8). Configuration faults are surfaced ahead of the
  GPS/timing-dependent ones, because they will not clear on their own. Text
  tokens fall through to the existing DB201 path unchanged.
- Pilot-declared emergency latch. A GCS-sent `SELF_ID` with
  `description_type == MAV_ODID_DESC_TYPE_EMERGENCY` latches an emergency; any
  other description type clears it. While latched, `Location.status` is forced
  to `MAV_ODID_STATUS_EMERGENCY` so the emergency reaches modules that key off
  operational status rather than SelfID text.
- The emergency latch auto-clears 30 s after the last `SELF_ID` receipt
  (`PILOT_EMERGENCY_TIMEOUT_MS`), so closing the GCS mid-emergency no longer
  broadcasts EMERGENCY until reboot. Clearing emits
  `"ODID: pilot emergency auto-cleared (GCS link stale)"`.

**Compatibility:**

- **DB201 units are affected by this build, not just DB300 units.** They will
  now receive SelfID and OperatorID messages they previously did not, on both
  the DroneCAN and static-out paths. The DB201 is pre-programmed with these
  values, so confirm on a bench DB201 unit that the incoming messages are
  ignored rather than overwriting what is programmed, before flashing a
  DB201 aircraft.
- **The default OperatorID `"001"` is a placeholder, not a real CAA
  registration.** If no GCS ever sends a real OperatorID, the module will
  broadcast `001` as a CAA-type operator ID. Confirm what the applicable Remote
  ID rules require here, and whether the operator ID should instead be
  provisioned per-unit, before rolling this to the fleet.
- The pilot emergency path requires the GCS to send `OPEN_DRONE_ID_SELF_ID`
  with `description_type = 1`. Arcsky Control needs to support that before the
  feature is usable in the field; without it the latch simply never sets and
  behaviour matches Build 14.
- All Build 13/14 behaviour (GEN_BAUD cutover, META_TX metadata pathways,
  "Hybrid Module Communication Restored") is unchanged.

Known documentation nit: two comments in `AP_OpenDroneID.cpp` still state
that "the SelfID forward path is intentionally disabled", which this build
reversed. Harmless, but worth correcting next time the file is touched.

**Files:** AP_OpenDroneID.cpp, AP_OpenDroneID.h, AP_OpenDroneID_DroneCAN.cpp,
hwdef/include/x55_version.inc, hwdef/CubeOrange/hwdef.dat,
hwdef/CubeOrangePlus/hwdef.dat, hwdef/CubeOrangePlus-Arcsky/hwdef.dat

---

## Build 14 — 2026-04-18

### METADATA TRANSMIT MODE (META_TX)

**Commit:** `f8f8f467d8` (follow-up to Build 13, same release date code)

Adds a configurable transmit mode for the Arcsky telemetry metadata introduced
in Build 13. Previously the ~ARCSKY / ~HM messages streamed out as STATUSTEXT
with a leading "~" on every channel. Some third-party GCS apps do not filter
"~"-prefixed lines and display them as spam in the text panel. META_TX lets
operators route the metadata through a DATA64 MAVLink message instead, which is
invisible to every GCS text UI but still captured in the tlog.

**Changes:**

- New parameter META_TX (bitmask):
    - Bit 0 (value 1) = STATUSTEXT with "~" prefix (logged to tlog AND .bin
      flight log as MSG records)
    - Bit 1 (value 2) = DATA64 MAVLink message (logged to tlog only; invisible
      in GCS text UI)
  Default: 2 (DATA64 only -- quietest for third-party GCS). Set 0 to disable
  metadata, 3 to send both during the transition period.
- DATA64 payload carries the same CSV strings previously sent via STATUSTEXT,
  with a reserved type ID in the u8 type field:
    - type 200 --> "ARCSKY,X55,<serial>,<flight_hours>"
    - type 201 --> "HM,<serial>,HW<v>,FW<v>,<runtime>,<maintenance>"
  Payload is ASCII; use the DATA64.len field for string length.
- Metadata builds the CSV line once per tick and dispatches it through
  either/both pathways based on META_TX bits. 30s alternating cadence between
  the two slots is unchanged.

**Compatibility:**

- Tlog parsers must be updated to read DATA64 when DATA64 mode is in use.
  Filter on DATA64.type in {200, 201} and decode data[0..len] as ASCII.
- With the default (META_TX=2), metadata no longer appears in .bin flight logs
  -- only in the tlog. If .bin-log capture of the serial is required post-flight
  (without tlog), set META_TX=1 or META_TX=3 (STATUSTEXT path writes to .bin via
  MSG records).
- Arcsky Control GCS ingestion needs DATA64 support before rolling the default
  out to the fleet. Until then, techs can set META_TX=1 for STATUSTEXT
  compatibility or META_TX=3 for dual-mode.
- All other Build 13 behavior (GEN_BAUD cutover, MediumLoop parsing, "Hybrid
  Module Communication Restored" message) unchanged.

**Files:** UserCode.cpp, Parameters.cpp, Parameters.h

---

## Build 13 — 2026-04-18

### HYBRID MODULE BAUD CUTOVER + TELEMETRY METADATA

**Commit:** `ae5f20a13c` (tip of x55/4.4.4-custom; spans fafba82aa1,
f0acc5a43c, cf707bd293, ae5f20a13c)

Hybrid Module serial link (rectifier --> FC) moved from 57600 baud to 115200
baud by default, matching the new rectifier firmware (d5809c3, 2026-04-14). A
new GEN_BAUD parameter provides backward compatibility with older Hybrid Module
firmware (<= 64d112d) which still runs at 57600.

**Files:** UserCode.cpp, Parameters.cpp, Parameters.h

---

## Build 12 — 2026-03-30

**Commit:** `7a77fc4cb2`

**Changes:**

- Re-enabled sending BasicID to DB201 over DroneCAN so the module can auto-save
  the UAS ID from the flight controller (supports new workflow where serial
  number is managed via DID parameters, not QGC)
- Rewrote BasicID receive handler: compares incoming BasicID to stored ID,
  updates persistent storage only if the ID actually changed
- Added ECU data handling and logging improvements
- Added serial number and model number sending to GCS and flight logs (for
  post-flight parsing and fleet tracking)

**Files:** AP_OpenDroneID.cpp, AP_OpenDroneID.h, AP_OpenDroneID_DroneCAN.cpp,
GCS_Mavlink.cpp, Log.cpp, UserCode.cpp

---

## Build 11 — 2026-03-23

**Commit:** `e5f62f977c`

**Changes:**

- Refactored EFI/ECU communications strategy in UserCode for new rectifier
  board comms approach (untested at time of commit)

**Files:** UserCode.cpp

---

## Build 10 — 2026-01-29

**Commit:** `4ae11fa03a`

**Changes:**

- Shortened Remote ID pre-arm failure messages for better readability on GCS:
    - "operator location not set in GCS"  --> "No Operator Location"
    - "vehicle location not available"    --> "No Drone Location"
    - "system data not received..."       --> "Remote I D System Message"
    - "BasicID not configured..."         --> "Basic I D not Configured"
- Changed pre-arm prefix from "OpenDroneID:" to "Remote I D:" (spaces are
  intentional for GCS text-to-speech / display compatibility)

**Files:** AP_Arming.cpp, AP_OpenDroneID.cpp

---

## Build 9 — 2026-01-24

### MAJOR REMOTE ID OVERHAUL

**Commit:** `9654a59728`

This build significantly changes how ArduPilot interacts with the Bluemark DB201
Remote ID module. The philosophy shifts from ArduPilot validating individual RID
fields to trusting the DB201's ARM_STATUS as the single source of truth.

**Changes:**

- Stopped sending BasicID, SelfID, and OperatorID to DB201 (both MAVLink and
  DroneCAN). The DB201 is pre-programmed with these values. Only Location and
  System messages are still forwarded.
- Added translate_arm_status_error() to convert cryptic DB201 error codes (e.g.
  "LOC", "SYS", "OP_LOC", "ID") into user-friendly messages
- Tightened ARM_STATUS timeout from 15 seconds back to 3 seconds
- Added 30-second boot grace period to suppress false "lost transmitter"
  warnings during startup
- Changed message text: "ODID: lost transmitter" --> "Remote ID: lost
  connection"
- Cleaned up dead/commented-out code from pre-arm checks
- Added explicit DID_ENABLE check in pre-arm

**Compatibility:**

- This build pairs with the updated QGC (Arcsky Control) that no longer sends
  BasicID/serial number from the ground station. Units with firmware OLDER than
  this build will still forward whatever BasicID they receive (or zeroed data if
  QGC doesn't send it) to the DB201. The DB201 firmware rejects zeroed BasicID
  messages, so no data corruption occurs, but the older ArduPilot may display
  confusing raw error codes from the DB201.

**Files:** AP_OpenDroneID.cpp, AP_OpenDroneID.h, AP_OpenDroneID_DroneCAN.cpp

---

## Build 8 — 2025-04-17

**Commit:** `8f330ea82f`

**Changes:**

- Added board voltage (VCC) monitoring: checks internal voltage and warns the
  operator if voltage is low, indicating possible bad pin connection (prompted
  by CB unit / Everest LiDAR issue)
- Changed generator failsafe severity level from ERROR/WARNING to CRITICAL

A matching 4.3.7 build was also produced with these same changes for units
still running the older firmware base.

**Files:** UserCode.cpp

---

## Build 7 — 2025-01-21

**Commit:** `5df9387319`

**Changes:**

- Added maximum angle limit of 15 degrees in Alt Hold mode only (does not affect
  Loiter, Auto, or other modes). Prompted by incident involving extremely
  fast/high power draw in Alt Hold mode flight.
- Custom startup tone to distinguish X55 firmware from stock ArduCopter

First firmware build done on JS laptop via VS Code + Cygwin (previously built on
JS Desktop via Eclipse). No functional difference expected.

**Files:** mode_althold.cpp, ToneAlarm.cpp

---

## Build 6 — 2024-07-25

**Commit:** `98c62370cf`

**Changes:**

- Updated EFI data logging
- Relaxed gyro consistency arming threshold to account for vibration from the
  hybrid module (V2 units were having arming issues)

**Files:** UserCode.cpp, AP_Arming.cpp

---

## Build 5 — 2024-07-01

**Commit:** `f53413b6ed`

**Changes:**

- Fixed EFI status message display issue

**Files:** UserCode.cpp

---

## Build 4 — 2024-06-30

**Commit:** `6768dabb79`

**Changes:**

- Updated UserCode to handle new EFI data format from rectifier board (matching
  changes from the 4.3.7 branch)

**Files:** UserCode.cpp

---

## Build 3 — 2024-06-17

**Commit:** `5c02b9dc13`

**Changes:**

- Removed generator kill logic from UserCode
- Fixed UART read bug: code now checks if generator protocol is configured on a
  SERIAL port before attempting UART reads (tested on MWH repair unit)

**Files:** UserCode.cpp

---

## Build 2 — 2024-04-05

**Commit:** `da2742e192`

**Changes:**

- Improved OpenDroneID functionality to reduce bugginess on GCS
- Added experimental Arcsky-locked hardware definitions for CubeOrange and
  CubeOrange+ (custom bootloader-locked builds)

**Files:** AP_OpenDroneID.cpp, CubeOrange-Arcsky/*, CubeOrangePlus-Arcsky/*

---

## Build 1 — 2024-01-23

### INITIAL PORT TO 4.4.4

**Commit:** `0030eae3b5`

Ported all X55 customizations from 4.3.7 stable to 4.4.4 stable. Required rework
of custom battery monitor code due to upstream API changes (battery monitor
files could no longer directly access Copter; new member functions added to
battery monitor, called from UserCode).

Auto alt offset adjustment code was NOT included in this build.

Features carried forward from 4.3.7:

- Hybrid Module (generator/rectifier board) serial communication
- Custom battery monitor (AP_BattMonitor_Generic_Fuel) for fuel percentage
- Generator error/warning detection and failsafe actions
- Hybrid Module heartbeat/timeout detection
- Redundant power supply voltage monitoring
- OpenDroneID customizations
- Custom GCS messaging

**Files:** 17 files across ArduCopter/, AP_BattMonitor/, AP_OpenDroneID/,
GCS_MAVLink/
