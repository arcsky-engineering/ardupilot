/*
   Xplorer parameter hard-bound clamp table.

   Defines per-parameter numeric min/max bounds enforced both at boot
   (scrubs any stored out-of-range value) and at MAVLink write time
   (rejects PARAM_SET requests with values outside the allowed window).

   Bounds source: xplorer-params-disposition.xlsx (Range min / Range max columns).
 */

#pragma once

#include <string.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

struct XplorerParamClamp {
    const char *name;
    float min_val;
    float max_val;
};

static const XplorerParamClamp xplorer_param_clamps[] = {
    { "ANGLE_MAX",        2000.0f,  3500.0f },
    { "ATC_SLEW_YAW",     3000.0f,  7000.0f },
    { "AUTO_TILT_DN",      -90.0f,     0.0f },
    { "AUTO_TILT_EN",        0.0f,     1.0f },
    { "AUTO_TILT_UP",      -45.0f,    20.0f },
    { "BATT_CAPACITY",    8000.0f, 30000.0f },
    { "BATT2_CAPACITY",   8000.0f, 20000.0f },
    { "BATT3_CAPACITY",   8000.0f, 20000.0f },
    { "FWDAVD_DIST",         3.0f,    40.0f },
    { "FWDAVD_SAMP",         1.0f,    20.0f },
    { "LAND_RNG_ALT",      600.0f,  2000.0f },
    { "LAND_RNG_SPD",       30.0f,   200.0f },
    { "LAND_SPEED",         20.0f,   200.0f },
    { "LAND_SPEED_HIGH",     0.0f,   200.0f },
    { "LOIT_ACC_MAX",      300.0f,   800.0f },
    { "LOIT_BRK_ACCEL",    200.0f,   400.0f },
    { "LOIT_BRK_JERK",     300.0f,   600.0f },
    { "LOIT_SPEED",         50.0f,  1800.0f },
    { "PILOT_ACCEL_Z",     100.0f,   400.0f },
    { "PILOT_SPEED_DN",     50.0f,   300.0f },
    { "PILOT_SPEED_UP",     50.0f,   350.0f },
    { "PILOT_Y_RATE",       30.0f,    90.0f },
    { "WPNAV_ACCEL",       100.0f,   350.0f },
    { "WPNAV_ACCEL_C",     100.0f,   300.0f },
    { "WPNAV_ACCEL_Z",      50.0f,   200.0f },
    { "WPNAV_RADIUS",        5.0f,  1000.0f },
    { "WPNAV_SPEED",        30.0f,  1800.0f },
    { "WPNAV_SPEED_DN",     30.0f,   300.0f },
    { "WPNAV_SPEED_UP",     30.0f,   300.0f },
};

static const uint8_t XPLORER_PARAM_CLAMP_COUNT =
    sizeof(xplorer_param_clamps) / sizeof(xplorer_param_clamps[0]);

// Returns pointer to clamp entry for `name`, or nullptr if unclamped.
static inline const XplorerParamClamp* xplorer_param_clamp_lookup(const char *name)
{
    for (uint8_t i = 0; i < XPLORER_PARAM_CLAMP_COUNT; i++) {
        if (strncmp(name, xplorer_param_clamps[i].name, AP_MAX_NAME_SIZE) == 0
            && strlen(xplorer_param_clamps[i].name) == strlen(name)) {
            return &xplorer_param_clamps[i];
        }
    }
    return nullptr;
}

// Returns true if value is in range (or param is unclamped). Reject semantics.
static inline bool xplorer_param_clamp_in_range(const char *name, float value)
{
    const XplorerParamClamp *c = xplorer_param_clamp_lookup(name);
    if (c == nullptr) {
        return true;
    }
    return (value >= c->min_val) && (value <= c->max_val);
}

// Boot-time scrub. Any stored value outside its declared range is clamped to
// the nearest bound and persisted, with a STATUSTEXT recording the change.
// Call once during vehicle init after parameter storage is ready.
static inline void xplorer_param_clamp_boot_scrub(void)
{
    for (uint8_t i = 0; i < XPLORER_PARAM_CLAMP_COUNT; i++) {
        const XplorerParamClamp &c = xplorer_param_clamps[i];
        enum ap_var_type vtype;
        AP_Param *vp = AP_Param::find(c.name, &vtype);
        if (vp == nullptr) {
            continue;
        }
        const float cur = vp->cast_to_float(vtype);
        const float clamped = constrain_float(cur, c.min_val, c.max_val);
        if (!is_equal(cur, clamped)) {
            vp->set_float(clamped, vtype);
            vp->save(true);
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "Clamped %s: %.3f -> %.3f",
                          c.name, (double)cur, (double)clamped);
        }
    }
}
