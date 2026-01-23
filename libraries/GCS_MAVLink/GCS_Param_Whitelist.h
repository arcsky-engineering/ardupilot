/*
   GCS MAVLink Parameter Whitelist

   This file defines which parameters are visible and settable by end users
   via MAVLink (GCS). Parameters not in this whitelist are hidden and cannot
   be read or modified by users - they become internal firmware variables.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

#pragma once

#include <AP_Param/AP_Param.h>

// Set to 1 to enable parameter whitelist filtering, 0 to disable (all params visible)
#ifndef GCS_PARAM_WHITELIST_ENABLED
#define GCS_PARAM_WHITELIST_ENABLED 0
#endif

/*
  Whitelist of parameters that users are allowed to see and modify.

  Supports two formats:
  1. Exact match: "PARAM_NAME" - matches only that exact parameter
  2. Prefix match: "PREFIX_*" - matches all parameters starting with PREFIX_

  Examples:
    "RTL_ALT"     -> matches only RTL_ALT
    "BATT_*"      -> matches BATT_CAPACITY, BATT_LOW_VOLT, BATT_ARM_VOLT, etc.
    "BATT1_*"     -> matches BATT1_CAPACITY, BATT1_LOW_VOLT, etc.
*/
static const char* const user_param_whitelist[] = {
    // ===========================================
    // Flight Modes
    // ===========================================
    "FLTMODE*",         // All flight mode selections

    // ===========================================
    // Basic Flight Parameters
    // ===========================================
    "RTL_ALT",          // RTL altitude
    "RTL_ALT_FINAL",    // RTL final altitude
    "WPNAV_SPEED",      // Waypoint navigation speed
    "WPNAV_SPEED_DN",   // Waypoint descent speed
    "WPNAV_SPEED_UP",   // Waypoint climb speed
    "WPNAV_RADIUS",     // Waypoint radius
    "LOIT_SPEED",       // Loiter speed
    "LAND_SPEED",       // Landing speed
    "PILOT_SPEED*",     // Pilot speed settings

    // ===========================================
    // Battery Settings
    // ===========================================
    "BATT_CAPACITY",    // Battery capacity
    "BATT_LOW_VOLT",    // Low voltage threshold
    "BATT_CRT_VOLT",    // Critical voltage threshold
    "BATT_ARM_VOLT",    // Arming voltage check
    "BATT_LOW_MAH",     // Low mAh threshold
    "BATT_CRT_MAH",     // Critical mAh threshold
    "BATT_FS_LOW_ACT",  // Low battery failsafe action
    "BATT_FS_CRT_ACT",  // Critical battery failsafe action
    "BATT2_*",          // Second battery settings (if applicable)

    // ===========================================
    // Failsafe Settings
    // ===========================================
    "FS_THR_ENABLE",    // Throttle failsafe enable
    "FS_THR_VALUE",     // Throttle failsafe PWM value
    "FS_GCS_ENABLE",    // GCS failsafe enable
    "FS_EKF_ACTION",    // EKF failsafe action
    "FS_EKF_THRESH",    // EKF failsafe threshold
    "FS_CRASH_CHECK",   // Crash check enable
    "FS_OPTIONS",       // Failsafe options bitmask

    // ===========================================
    // Arming Checks
    // ===========================================
    "ARMING_CHECK",     // Arming checks bitmask

    // ===========================================
    // Geofence
    // ===========================================
    "FENCE_ENABLE",     // Fence enable
    "FENCE_TYPE",       // Fence type
    "FENCE_ACTION",     // Fence breach action
    "FENCE_ALT_MAX",    // Maximum altitude fence
    "FENCE_ALT_MIN",    // Minimum altitude fence
    "FENCE_RADIUS",     // Circular fence radius
    "FENCE_MARGIN",     // Fence margin

    // ===========================================
    // RC Input
    // ===========================================
    "RC_OPTIONS",       // RC options
    "RC_PROTOCOLS",     // RC protocols

    // ===========================================
    // Serial/Telemetry (user may need to configure)
    // ===========================================
    "SERIAL*_BAUD",     // Serial baud rates
    "SERIAL*_PROTOCOL", // Serial protocols

    // ===========================================
    // GPS
    // ===========================================
    "GPS*_TYPE",        // GPS type selection

    // ===========================================
    // System
    // ===========================================
    "SYSID_THISMAV",    // MAVLink system ID
    "FORMAT_VERSION",   // Parameter format version (read-only info)

    // ===========================================
    // Add your allowed parameters above this line
    // ===========================================
    nullptr  // Terminator - do not remove
};

/*
  Check if a parameter name is in the user whitelist.
  Returns true if the parameter should be visible/settable by users.
*/
static inline bool is_user_allowed_param(const char* param_name)
{
#if !GCS_PARAM_WHITELIST_ENABLED
    // Whitelist disabled - all parameters visible
    return true;
#else
    if (param_name == nullptr) {
        return false;
    }

    for (uint8_t i = 0; user_param_whitelist[i] != nullptr; i++) {
        const char* pattern = user_param_whitelist[i];
        const size_t pattern_len = strlen(pattern);

        if (pattern_len == 0) {
            continue;
        }

        // Check for wildcard prefix match (e.g., "BATT_*")
        if (pattern[pattern_len - 1] == '*') {
            // Match if param_name starts with the prefix (excluding the *)
            if (strncmp(param_name, pattern, pattern_len - 1) == 0) {
                return true;
            }
        } else {
            // Exact match
            if (strncmp(param_name, pattern, AP_MAX_NAME_SIZE) == 0) {
                return true;
            }
        }
    }

    return false;
#endif
}

/*
  Count parameters that pass the whitelist filter.
  This should be called instead of AP_Param::count_parameters() when
  reporting parameter count to GCS.
*/
static inline uint16_t count_user_parameters(void)
{
#if !GCS_PARAM_WHITELIST_ENABLED
    return AP_Param::count_parameters();
#else
    AP_Param::ParamToken token {};
    AP_Param *ap;
    uint16_t count = 0;

    for (ap = AP_Param::first(&token, nullptr);
         ap != nullptr;
         ap = AP_Param::next_scalar(&token, nullptr)) {

        char param_name[AP_MAX_NAME_SIZE + 1];
        ap->copy_name_token(token, param_name, sizeof(param_name), true);

        if (is_user_allowed_param(param_name)) {
            count++;
        }
    }

    return count;
#endif
}
