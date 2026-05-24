/*
   Xplorer compass motor-compensation pinning.

   ArduPilot stores CompassMot output (COMPASS_MOT_X/Y/Z, COMPASS_MOT2_*,
   COMPASS_MOT3_*) keyed by SLOT, not by device ID. When PRIO pinning is
   functioning, slot→device is stable and MOT values track the right physical
   compass. If anything ever shuffles the slot ordering, MOT values land on
   the wrong sensor and silently corrupt heading compensation.

   This boot-time scrub closes that gap. For each compass slot we look up
   the device currently in it (COMPASS_DEV_IDn), find its canonical MOT
   values from a static table, and write them into the slot's COMPASS_MOTn_*
   params if they don't already match. Result: MOT compensation is
   device-ID-keyed in effect, even though the underlying ArduPilot storage
   is slot-keyed.

   Trade-off (chosen "blind apply" policy):
     Running CompassMot to recalibrate on a deployed unit will not persist
     across reboots — the next boot will overwrite the fresh values with the
     canonical table below. To make a recalibration stick, update the table
     in this file and rebuild firmware. This is intentional for a locked
     production build where canonical values are part of the signed image.
 */

#pragma once

#include <string.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

struct XplorerCompassMot {
    uint32_t device_id;
    float    mot_x;
    float    mot_y;
    float    mot_z;
};

// Canonical compass-motor compensation per physical device.
// Values mirror defaults.parm; update both together if recalibrating.
static const XplorerCompassMot xplorer_compass_mot_table[] = {
    {   96515u,   5.4f,  -0.6f,  -3.2f },  // RM3100 (external, expected slot 1 via PRIO1_ID)
    {  162051u,   4.9f,   0.2f,  -2.8f },  // Here4 AKM via DroneCAN (expected slot 2 via PRIO2_ID)
    { 1313809u,   0.0f,   0.0f,   0.0f },  // CubeOrangePlus internal IST8310 (expected slot 3 via PRIO3_ID)
};

static const uint8_t XPLORER_COMPASS_MOT_COUNT =
    sizeof(xplorer_compass_mot_table) / sizeof(xplorer_compass_mot_table[0]);

static inline const XplorerCompassMot* xplorer_compass_mot_lookup(uint32_t device_id)
{
    for (uint8_t i = 0; i < XPLORER_COMPASS_MOT_COUNT; i++) {
        if (xplorer_compass_mot_table[i].device_id == device_id) {
            return &xplorer_compass_mot_table[i];
        }
    }
    return nullptr;
}

// Read the device ID for a single compass slot and, if recognised, align the
// slot's MOT_X/Y/Z to the canonical table. No-ops if the slot is empty or the
// device is unknown.
static inline void xplorer_compass_mot_apply_for_slot(const char *dev_id_name,
                                                      const char *mot_x_name,
                                                      const char *mot_y_name,
                                                      const char *mot_z_name)
{
    enum ap_var_type vt_dev;
    AP_Param *vp_dev = AP_Param::find(dev_id_name, &vt_dev);
    if (vp_dev == nullptr) {
        return;
    }
    const uint32_t dev_id = (uint32_t)vp_dev->cast_to_float(vt_dev);
    if (dev_id == 0) {
        return;  // slot empty this boot
    }

    const XplorerCompassMot *entry = xplorer_compass_mot_lookup(dev_id);
    if (entry == nullptr) {
        return;  // device not in our canonical table
    }

    enum ap_var_type vt_x, vt_y, vt_z;
    AP_Param *vp_x = AP_Param::find(mot_x_name, &vt_x);
    AP_Param *vp_y = AP_Param::find(mot_y_name, &vt_y);
    AP_Param *vp_z = AP_Param::find(mot_z_name, &vt_z);
    if (vp_x == nullptr || vp_y == nullptr || vp_z == nullptr) {
        return;
    }

    const float cur_x = vp_x->cast_to_float(vt_x);
    const float cur_y = vp_y->cast_to_float(vt_y);
    const float cur_z = vp_z->cast_to_float(vt_z);

    if (is_equal(cur_x, entry->mot_x)
        && is_equal(cur_y, entry->mot_y)
        && is_equal(cur_z, entry->mot_z)) {
        return;  // already aligned
    }

    vp_x->set_float(entry->mot_x, vt_x); vp_x->save(true);
    vp_y->set_float(entry->mot_y, vt_y); vp_y->save(true);
    vp_z->set_float(entry->mot_z, vt_z); vp_z->save(true);

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                  "Aligned %s for dev %lu: (%.3f, %.3f, %.3f)",
                  mot_x_name, (unsigned long)dev_id,
                  (double)entry->mot_x, (double)entry->mot_y, (double)entry->mot_z);
}

// Boot-time alignment for all three compass slots. Call once during init,
// after parameter storage and compass detection are complete.
static inline void xplorer_compass_mot_boot_align(void)
{
    xplorer_compass_mot_apply_for_slot("COMPASS_DEV_ID",
                                       "COMPASS_MOT_X",
                                       "COMPASS_MOT_Y",
                                       "COMPASS_MOT_Z");
    xplorer_compass_mot_apply_for_slot("COMPASS_DEV_ID2",
                                       "COMPASS_MOT2_X",
                                       "COMPASS_MOT2_Y",
                                       "COMPASS_MOT2_Z");
    xplorer_compass_mot_apply_for_slot("COMPASS_DEV_ID3",
                                       "COMPASS_MOT3_X",
                                       "COMPASS_MOT3_Y",
                                       "COMPASS_MOT3_Z");
}
