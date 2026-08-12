# Xplorer param disposition sheet — what to change in v2

Comparison of `xplorer-params-disposition-v2.xlsx` against the firmware as it
stands on `xplorer/4.6.2-custom` @ `cc3aec9c4f`.

A regenerated, firmware-true sheet is at **`xplorer-params-disposition-v3.xlsx`**.
Columns A–H match v2's layout exactly, so it is a drop-in replacement. Columns
I–K are additive: `v2_user_visible (intent)`, `v2_readonly (intent)` and
`changed_vs_v2`. Changed rows are highlighted. Nothing from v2 is discarded.

## Where the firmware keeps each column's truth

| Sheet column | Firmware source |
|---|---|
| `default value`, `readonly` | [defaults.parm](libraries/AP_HAL_ChibiOS/hwdef/CubeOrangePlus-ODID/defaults.parm) — value plus `@READONLY` marker |
| `user_visible` | `AP_PARAM_FLAG_HIDDEN` in the `var_info` tables — [Parameters.cpp](ArduCopter/Parameters.cpp), [RC_Channels_VarInfo.h](libraries/RC_Channel/RC_Channels_VarInfo.h), [SRV_Channels.cpp](libraries/SRV_Channel/SRV_Channels.cpp) |
| `Range min` / `Range max` | [GCS_Param_Clamps.h](libraries/GCS_MAVLink/GCS_Param_Clamps.h) |

`CubeOrangePlus-ODID/defaults.parm` and `CubeOrangePlus/defaults.parm` are byte-identical.

Counts: 1098 params seeded in `defaults.parm`, 728 of them `@READONLY`; 29 clamp
entries; 1223 rows in the sheet. Every param the firmware seeds already has a
row — **nothing needs to be added**.

---

## 1. `user_visible` — 822 rows to flip False → True

This is the big one, and it is not a bookkeeping error: **the sheet records the
hiding plan, and the firmware has only implemented part of it.**

Everything the firmware actually hides is hidden via one of these, and that set
is a strict *subset* of what v2 marks invisible. There is no row where v2 says
visible but the firmware hides it.

Currently hidden — group-level, `ArduCopter/Parameters.cpp`:

`CHUTE_`, `LGR_`, `CIRCLE_`, `SCHED_`, `AVOID_`, `RSSI_`, `RNGFND`, `PLND_`,
`RPM`, `ADSB_`, `AVD_`, `NTF_`, `OSD`, `CC`

Currently hidden — individual params: `RC_SPEED`, `RNGFND_FILT`,
`RC_OVERRIDE_TIME`, `RC_OPTIONS`, `RC_PROTOCOLS`, `SERVO_AUTO_TRIM` (Plane
frame only), `SERVO_RATE`, `SERVO_DSHOT_RATE`, `SERVO_DSHOT_ESC`,
`SERVO_GPIO_MASK`, `SERVO_RC_FS_MSK`, `SERVO_32_ENABLE`, plus the
`SERVO_VOLZ_` / `SERVO_SBUS_` / `SERVO_BLH_` / `SERVO_ROB_` / `SERVO_FTW_`
subgroups and the `SERVO1_`–`SERVO8_` and `SERVO12_`–`SERVO32_` channel
subgroups. **`SERVO9_`, `SERVO10_` and `SERVO11_` are still visible** — worth a
look, since 1–8 and 12+ are hidden and that gap may not be deliberate.

Note also that commit `e4f05f6579` *un-hid* five groups that v2 still marks
invisible: `INS`, `PSC`, `AHRS_`, `BARO` and `RCMAP_`.

Largest groups still exposed to the GCS despite v2 marking them invisible:

| Prefix | Params | | Prefix | Params |
|---|---|---|---|---|
| `EK3` | 83 | | `BATT`/`BATT2`/`BATT3` | 39 |
| `ATC` | 60 | | `MNT1` | 18 |
| `INS` | 41 | | `SCR` | 15 |
| `COMPASS` | 32 | | `GPS`/`GPS1` | 24 |
| `PSC` | 32 | | `SR0`/`SR1`/`SR5`/`SR6` | 40 |
| `CAN` | 27 | | `CAM1` | 12 |
| `BRD` | 25 | | `AHRS` | 10 |
| `MOT` | 23 | | `BARO`+`BARO1..3` | 18 |

**Decide which way to close this gap** — the sheet and the firmware disagree, so
one of them has to move:

- If v2's intent stands, ~14 more group prefixes need `GOBJECT_HIDDEN` /
  `AP_PARAM_FLAG_HIDDEN` and the sheet stays as-is. v3 would then be wrong.
- If the current firmware behaviour is what you want to ship, take v3's column E
  and treat column I as the historical wishlist.

v3 assumes the second reading, because you asked for the sheet to match the
firmware. Column I keeps the intent either way.

## 2. `readonly` — 30 rows to flip True → False

All 30 are compass/mag related and all stem from a **temporary experiment that
is still live in the tree**. 26 carry an explicit
`# TEMP-COMPASS-EXPERIMENT unlocked` comment in `defaults.parm`:

`BRD_BOOT_DELAY`, `COMPASS_AUTODEC`, `COMPASS_AUTO_ROT`, `COMPASS_DEV_ID`,
`COMPASS_DEV_ID2`–`COMPASS_DEV_ID8`, `COMPASS_ENABLE`, `COMPASS_EXTERNAL`,
`COMPASS_EXTERN2`, `COMPASS_EXTERN3`, `COMPASS_MOTCT`, `COMPASS_OPTIONS`,
`COMPASS_ORIENT`, `COMPASS_ORIENT2`, `COMPASS_ORIENT3`, `COMPASS_PRIO1_ID`,
`COMPASS_PRIO2_ID`, `COMPASS_PRIO3_ID`, `COMPASS_USE`, `COMPASS_USE2`,
`COMPASS_USE3`

Four more were unlocked without a marker comment (commit `6f1d594654`), so
they are easy to lose track of — consider adding comments:
`COMPASS_CAL_FIT`, `COMPASS_LEARN`, `COMPASS_OFFS_MAX`, `EK3_MAG_CAL`

> **Flagging for a production build:** `xplorer_compass_mot_boot_align()` is
> commented out at [system.cpp:216](ArduCopter/system.cpp#L216) with a
> "revert before production build" note. The device-ID pinning table in
> [xplorer_compass_mot_pinning.h](ArduCopter/xplorer_compass_mot_pinning.h) is
> therefore inert right now. If the experiment is finished, re-locking these 30
> and re-enabling the call would make v3's column D revert to v2's values.

## 3. `default value` — 2 real corrections

| Param | v2 says | firmware | 
|---|---|---|
| `EK3_MAGB_P_NSE` | 0.0002 | **0.0001** |
| `EK3_MAG_M_NSE` | 0.08 | **0.05** |

## 4. `Range` — 1 correction

| Param | v2 range | clamp table |
|---|---|---|
| `LOIT_SPEED` | 50 … 1800 | 50 … **2000** |

The other 28 clamp entries match v2 exactly.

## 5. 125 rows whose `default value` should be blanked

These params **exist and are writable**, but are deliberately *not* seeded in
`defaults.parm`. v2 carries a captured value from one specific airframe, which
is misleading in a fleet-wide disposition document — a fresh board will not have
these numbers. v3 blanks column C and explains why in column F. They also all
become `readonly = False` (113 of them were marked True), because there is no
`@READONLY` line to enforce it.

| Group | Count | Why unseeded |
|---|---|---|
| Per-unit INS cal (`INS_ACC*`, `INS_GYR*`, `*_CALTEMP`, `*_ID`) | 39 | Stripped in `6f1d594654` so each airframe keeps its own calibration |
| `INS4_*`, `INS5_*` | 36 | Aux IMU slots — `CubeOrangePlus/hwdef.dat:82` sets `INS_AUX_INSTANCES 2`, so the params exist, but no 4th/5th IMU is fitted. **Keep the rows.** |
| Per-unit compass cal (`COMPASS_OFS*`, `DIA*`, `ODI*`, `SCALE*`) | 30 | Same commit |
| `COMPASS_MOT*` | 9 | Handled by the device-ID pinning table — currently disabled, see §2 |
| Runtime-managed | 8 | `FENCE_TOTAL`, `FORMAT_VERSION`, `MIS_TOTAL`, `RALLY_TOTAL`, `STAT_BOOTCNT`, `STAT_FLTTIME`, `STAT_RESET`, `STAT_RUNTIME` — documented as omitted in the `defaults.parm` header so a fresh board does not inherit stale state |
| `AHRS_TRIM_X/Y/Z` | 3 | Per-unit level trim |

v2's own note on the nine `COMPASS_MOT*` rows — *"should make false once we fix
this on a lower level"* — is preserved in v3's notes column. That fix is the
pinning table, which is written but disabled.

---

## Totals

| | Rows |
|---|---|
| `user_visible` False → True | 822 |
| `readonly` True → False | 143 (30 unlocked + 113 unseeded) |
| `default value` changed or blanked | 127 (2 corrections + 125 blanked) |
| `Range` corrected | 1 |
| Rows to add | 0 |
| Rows to delete | 0 |
| Rows with no change at all | 384 |

## One caveat on method

`user_visible` was derived by reading the `AP_PARAM_FLAG_HIDDEN` flags out of the
`var_info` tables statically, not by flashing a board and walking
`PARAM_REQUEST_LIST`. The hidden set is small and explicit, so this should be
exact — but a `param download` from a live unit is the authoritative check if you
want one before publishing the sheet.
