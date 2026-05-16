#include "Copter.h"

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;
}

#if AP_RANGEFINDER_ENABLED
void Copter::init_rangefinder(void)
{
   rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
   rangefinder.init(ROTATION_PITCH_270);
   rangefinder_state.alt_cm_filt.set_cutoff_frequency(g2.rangefinder_filt);
   // RFND_BTN_EN is the master gate; require hardware AND param ON at boot
   rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270) && (g2.rfnd_btn_en.get() != 0);

   // upward facing range finder
   rangefinder_up_state.alt_cm_filt.set_cutoff_frequency(g2.rangefinder_filt);
   rangefinder_up_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_90);
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
    rangefinder.update();

    // Master enable evaluation. RFND_BTN_EN is authoritative: 0 = off regardless of switch.
    // When RFND_BTN_EN is 1, a configured RANGEFINDER aux switch acts as the runtime gate
    // (LOW = off, HIGH = on). With no aux switch configured, the param alone controls.
    {
        const bool param_on = (g2.rfnd_btn_en.get() != 0);
        const bool hw_present = rangefinder.has_orientation(ROTATION_PITCH_270);
        bool desired = param_on && hw_present;
        if (desired) {
            RC_Channel *rngfnd_chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::RANGEFINDER);
            if (rngfnd_chan != nullptr) {
                desired = (rngfnd_chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH);
            }
        }
        rangefinder_state.enabled = desired;
    }

    rangefinder_state.update();
    rangefinder_up_state.update();

#if HAL_PROXIMITY_ENABLED
    if (rangefinder_state.enabled_and_healthy() || rangefinder_state.data_stale()) {
        g2.proximity.set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
    }
#endif

    update_rfnd_status();
}

// compute 4-state QGC indicator for downward rangefinder.
// 0 = disabled (param off, no hardware, or RANGEFINDER aux switch LOW)
// 1 = enabled but no valid data (sensor unhealthy or out of range)
// 2 = standby (enabled and healthy, but surface tracking not currently engaged
//     — wrong flight mode, SURFTRAK_MODE != GROUND, persistent glitch, etc.)
// 3 = active (enabled, healthy, AND surface tracking is currently running)
uint8_t Copter::compute_rfnd_status()
{
    if (!rangefinder_state.enabled) {
        return 0;
    }
    if (!rangefinder_state.alt_healthy) {
        return 1;
    }
    if (!surface_tracking.is_active()) {
        return 2;
    }
    return 3;
}

// broadcast the RFND_ST status as NAMED_VALUE_INT to all active GCS channels.
void Copter::send_rfnd_status(int32_t value)
{
    mavlink_named_value_int_t packet {};
    packet.time_boot_ms = AP_HAL::millis();
    packet.value = value;
    const char name[] = "RFND_ST";
    memcpy(packet.name, name, MIN(sizeof(name) - 1, (size_t)MAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_LEN));
    gcs().send_to_active_channels(MAVLINK_MSG_ID_NAMED_VALUE_INT, (const char *)&packet);
}

// send the QGC status on transition or every ~2s as a heartbeat.
void Copter::update_rfnd_status()
{
    const uint8_t status = compute_rfnd_status();
    const uint32_t now_ms = AP_HAL::millis();

    const bool changed = (status != rfnd_status_state.last_status_sent);
    const bool heartbeat = (now_ms - rfnd_status_state.last_status_send_ms) >= 2000;

    if (changed || heartbeat) {
        send_rfnd_status(status);
        rfnd_status_state.last_status_sent = status;
        rfnd_status_state.last_status_send_ms = now_ms;
    }
}
#endif  // AP_RANGEFINDER_ENABLED

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok() const
{
    return rangefinder_state.enabled_and_healthy();
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_up_ok() const
{
    return rangefinder_up_state.enabled_and_healthy();
}

// update rangefinder based terrain offset
// terrain offset is the terrain's height above the EKF origin
void Copter::update_rangefinder_terrain_offset()
{
    float terrain_offset_cm = rangefinder_state.inertial_alt_cm - rangefinder_state.alt_cm_glitch_protected;
    rangefinder_state.terrain_offset_cm += (terrain_offset_cm - rangefinder_state.terrain_offset_cm) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    terrain_offset_cm = rangefinder_up_state.inertial_alt_cm + rangefinder_up_state.alt_cm_glitch_protected;
    rangefinder_up_state.terrain_offset_cm += (terrain_offset_cm - rangefinder_up_state.terrain_offset_cm) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    if (rangefinder_state.alt_healthy || rangefinder_state.data_stale()) {
        wp_nav->set_rangefinder_terrain_offset(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.terrain_offset_cm);
#if MODE_CIRCLE_ENABLED
        circle_nav->set_rangefinder_terrain_offset(rangefinder_state.enabled && wp_nav->rangefinder_used(), rangefinder_state.alt_healthy, rangefinder_state.terrain_offset_cm);
#endif
    }
}

// helper function to get inertially interpolated rangefinder height.
bool Copter::get_rangefinder_height_interpolated_cm(int32_t& ret) const
{
#if AP_RANGEFINDER_ENABLED
    return rangefinder_state.get_rangefinder_height_interpolated_cm(ret);
#else
    return false;
#endif
}
