/*
   Xplorer DEV-build parameter unlock list.

   In a DEV build the @READONLY markers in defaults.parm are IGNORED for the
   parameter name prefixes listed below, and the hard clamp table in
   GCS_Param_Clamps.h is bypassed entirely. This lets engineering tune the
   control loops, EKF and filters over MAVLink without a firmware rebuild per
   change.

   Production builds leave XPLORER_DEV_UNLOCK_ENABLED at 0 and behave exactly as
   before: this header then compiles to nothing.

   DEV firmware is a SEPARATE BOARD TARGET (CubeOrangePlus-ODID-DEV) rather than
   a build flag, so it cannot be produced by accident - the board name is in
   every build command. Build it with:

       ./waf configure --board CubeOrangePlus-ODID-DEV
       ./waf copter

   IMPORTANT: it shares APJ_BOARD_ID 10163 with production. Fielded bootloaders
   are built with AP_SIGNED_FIRMWARE and enforce that ID with no ALT_BOARD_ID, so
   a unique ID would make this build impossible to upload rather than safer.
   Separation is therefore PROCEDURAL - the WARNING boot banner and not shipping
   DEV apj files - not hardware-enforced. A signed DEV apj will load onto any
   Xplorer, so handle it accordingly.

   HOW THE UNLOCK WORKS
     AP_Param::parse_param_line() is the single place @READONLY is interpreted,
     and num_read_only is derived from what it returns, so clearing read_only
     there is consistent everywhere downstream (is_read_only(),
     allow_set_via_mavlink(), the GCS param count).

   ADDING A SUBSYSTEM
     Add its prefix below. Matching is by prefix, so "ATC_" covers every
     ATC_RAT_* param. Then regenerate the docs so they describe what this build
     actually enforces:
         python Tools/xplorer/gen_param_docs.py --dev
 */

#pragma once

#include <string.h>

#ifndef XPLORER_DEV_UNLOCK_ENABLED
#define XPLORER_DEV_UNLOCK_ENABLED 0
#endif

#if XPLORER_DEV_UNLOCK_ENABLED

/*
  Parameter name prefixes whose @READONLY marker is ignored in DEV builds.

  CAUTION on "MOT_": this deliberately includes MOT_PWM_TYPE, MOT_SPIN_ARM,
  MOT_SPIN_MIN/MAX and MOT_THST_EXPO. A bad value there is a bench-test hazard
  (unexpected motor output), not merely poor tuning. Props off when changing
  them.

  CAUTION on "EK3_": a bad EKF noise/gate value can produce a healthy-looking
  but wrong state estimate. Prefer changing one parameter at a time and watching
  the EKF status flags.
 */
static const char *const xplorer_dev_unlock[] = {
    // Control loops - the primary reason this build exists
    "PSC_",             // position controller
    "ATC_",             // attitude controller, incl. all ATC_RAT_* rate gains
    "EK3_",             // EKF3 tuning and source selection

    // Filtering / vibration work
    "INS_HNTCH_",       // harmonic notch 1
    "INS_HNTC2_",       // harmonic notch 2
    "FFT_",             // in-flight FFT, incl. the normally-locked structural set
    "FILT",             // FILT1_TYPE .. FILT8_TYPE filter bank

    // Tuning that pairs with the above
    "AUTOTUNE_",
    "MOT_",             // see CAUTION above
    "LOIT_",
    "WPNAV_",
    "PILOT_",
    "ANGLE_MAX",
};

static const uint8_t XPLORER_DEV_UNLOCK_COUNT =
    sizeof(xplorer_dev_unlock) / sizeof(xplorer_dev_unlock[0]);

// True if `name` is covered by the dev unlock list (prefix match).
static inline bool xplorer_dev_unlocked(const char *name)
{
    if (name == nullptr) {
        return false;
    }
    for (uint8_t i = 0; i < XPLORER_DEV_UNLOCK_COUNT; i++) {
        const char *p = xplorer_dev_unlock[i];
        if (strncmp(name, p, strlen(p)) == 0) {
            return true;
        }
    }
    return false;
}

#endif // XPLORER_DEV_UNLOCK_ENABLED
