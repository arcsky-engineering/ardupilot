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

   Device IDs below were validated live over MAVLink on 2026-09-01 against the
   dev airframe: Ark Mag + Cube Orange+ internal + an OLDER single-compass Here4
   (RM3100 only). The NEWER Here4 is the dual-compass one and adds a second
   sensor on the same node; both variants are covered by the table.
   The DroneCAN IDs are deterministic: node IDs on both peripherals are set
   STATICALLY, and CAN_P1_DRIVER == CAN_P2_DRIVER == 1 puts both physical CAN
   ports on driver index 0, so neither the node-ID nor the bus field of the
   devid can shift with DNA allocation order or a harness reroute.
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
// MOT values are NOT seeded in defaults.parm (stripped in 6f1d594) — this table
// is their only source of truth. Devid encoding for DroneCAN mags is
// make_bus_id(UAVCAN, driver_index, node_id, sensor_id + 1); see
// AP_Compass_DroneCAN::get_dronecan_backend().
static const XplorerCompassMot xplorer_compass_mot_table[] = {
    //  devid        MOT_X   MOT_Y   MOT_Z
    {   96259u,   0.0f,   0.0f,   0.0f },  // Ark Mag, node 120 sensor 0 — pinned to PRIO1. No
                                           // compassmot by design: measured clean, left at zero.
    {   96515u,   5.4f,   0.0f,  -3.2f },  // Here4 RM3100, node 121 sensor 0 — pinned to PRIO2.
                                           // Present on every Here4 version, old and new.
                                           // Y deliberately 0: its sign was ambiguous across
                                           // battery packs, so zero was chosen as the safe value.
    {  162051u,   0.0f,   0.0f,   0.0f },  // Here4 second compass, node 121 sensor 1. Present
                                           // only on the NEWER dual-compass Here4, and not used
                                           // for yaw (USE3=0), so held at zero rather than
                                           // carrying the old unverified 4.9/0.2/-2.8 forward.
    { 1313809u,   0.0f,   0.0f,   0.0f },  // Cube Orange+ internal AK09918, I2C2 0x0C.
    {  592913u,   0.0f,   0.0f,   0.0f },  // Same slot on AK09916-fitted Cube batches. devtype is
                                           // read from the chip at runtime, so the devid differs.
                                           // Computed, not yet observed on hardware.
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
//
// Slots 1 and 2 are pinned by PRIO1_ID/PRIO2_ID to the Ark Mag and the Here4
// RM3100. Slot 3 is left floating: it holds the Cube internal on airframes with an
// older single-compass Here4, and the Here4 second compass on airframes with the
// newer dual-compass Here4. That is safe here because every device that can land
// there is a zero entry above, and an unrecognised device is a no-op.
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
