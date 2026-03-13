/*
   Forward Rangefinder Avoidance

   This module implements a simple forward-looking rangefinder obstacle
   avoidance system. When obstacles are detected within the configured
   distance for a sufficient number of samples, a failsafe action is
   triggered (RTL, SmartRTL, Land, or Stop with stick lockout).

   The behavior differs between auto modes and manual modes via separate parameters:
   - Auto modes (FWDAVD_ACT_AUTO): RTL, SmartRTL, or Land
   - Manual modes (FWDAVD_ACT_MAN): RTL, SmartRTL, Land, or Stop with stick lockout

   The system includes activation gating based on altitude, terrain, or RC channel.
*/

#include "Copter.h"

#if AP_RANGEFINDER_ENABLED

// update forward rangefinder avoidance - called at 10Hz
void Copter::avoidance_rfnd_update()
{
    // check if feature is enabled at all
    if (g2.fwdavd_enable == 0) {
        return;
    }

    // check if we're armed
    if (!motors->armed()) {
        // reset state when disarmed
        fwdavd_state.thresh_count = 0;
        fwdavd_state.triggered = false;
        fwdavd_state.stick_locked = false;
        fwdavd_state.active = false;
        fwdavd_state.last_active = false;
        return;
    }

    // check activation condition (altitude/terrain/RC gating)
    bool activation_ok = check_fwdavd_activation();

    // notify on state change
    if (activation_ok != fwdavd_state.last_active) {
        if (activation_ok) {
            gcs().send_text(MAV_SEVERITY_INFO, "Forward Avoidance: Active");
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Forward Avoidance: Inactive");
        }
        fwdavd_state.last_active = activation_ok;
    }

    fwdavd_state.active = activation_ok;

    if (!activation_ok) {
        // reset all state when feature is deactivated (RC switch off)
        fwdavd_state.thresh_count = 0;
        fwdavd_state.triggered = false;
        if (fwdavd_state.stick_locked) {
            fwdavd_state.stick_locked = false;
            gcs().send_text(MAV_SEVERITY_INFO, "Forward Avoidance: Control restored (disabled)");
        }
        return;
    }

    // check if enabled for current flight mode
    if (!is_fwdavd_enabled_for_mode()) {
        fwdavd_state.thresh_count = 0;
        // clear lockout if pilot switched to a non-enabled mode
        if (fwdavd_state.stick_locked) {
            fwdavd_state.stick_locked = false;
            fwdavd_state.triggered = false;
            gcs().send_text(MAV_SEVERITY_INFO, "Forward Avoidance: Control restored (mode change)");
        }
        return;
    }

    // if stick is locked, check for unlock condition
    if (fwdavd_state.stick_locked) {
        check_fwdavd_stick_unlock();
    }

    // get forward rangefinder reading
    const RangeFinder *rngfnd = RangeFinder::get_singleton();
    if (rngfnd == nullptr) {
        return;
    }

    // check forward-facing rangefinder (ROTATION_NONE or ROTATION_YAW_0 = forward)
    const RangeFinder::Status rf_status = rngfnd->status_orient(ROTATION_NONE);
    const float distance_m = (rf_status == RangeFinder::Status::Good) ?
                             rngfnd->distance_cm_orient(ROTATION_NONE) * 0.01f : -1.0f;
    const float threshold_m = g2.fwdavd_dist;

    // check if obstacle within threshold (requires valid reading AND distance below threshold)
    if (rf_status == RangeFinder::Status::Good && distance_m < threshold_m && distance_m > 0.0f) {
        // obstacle detected - increment counter
        if (fwdavd_state.thresh_count < g2.fwdavd_samp) {
            fwdavd_state.thresh_count++;
        }

        // check if we've exceeded the sample threshold
        if (fwdavd_state.thresh_count >= g2.fwdavd_samp) {
            // trigger avoidance action if not already triggered
            if (!fwdavd_state.triggered) {
                handle_fwdavd_action();
            }
        }
    } else {
        // no obstacle - decrement counter
        if (fwdavd_state.thresh_count > 0) {
            fwdavd_state.thresh_count--;
        }

        // check for reset condition (obstacle cleared)
        // don't reset triggered if stick_locked is still true to prevent re-triggering
        if (fwdavd_state.triggered && fwdavd_state.thresh_count == 0 && !fwdavd_state.stick_locked) {
            // obstacle has cleared and sticks aren't locked - allow reset
            fwdavd_state.triggered = false;
            gcs().send_text(MAV_SEVERITY_INFO, "Forward Avoidance: Cleared");
        }
    }
}

// check if forward avoidance is enabled for current flight mode
bool Copter::is_fwdavd_enabled_for_mode()
{
    const uint16_t enable_mask = g2.fwdavd_enable;
    const Mode::Number mode_num = flightmode->mode_number();

    // Bit 0: Auto modes (Auto, Guided, RTL, SmartRTL, etc.)
    // Bit 1: Loiter
    // Bit 2: PosHold
    // Bit 3: AltHold
    // Bit 4: Land (special - only escalate, don't interrupt)

    switch (mode_num) {
        case Mode::Number::AUTO:
#if MODE_GUIDED_ENABLED
        case Mode::Number::GUIDED:
#endif
            return (enable_mask & (1 << 0)) != 0;

#if MODE_LOITER_ENABLED
        case Mode::Number::LOITER:
            return (enable_mask & (1 << 1)) != 0;
#endif

#if MODE_POSHOLD_ENABLED
        case Mode::Number::POSHOLD:
            return (enable_mask & (1 << 2)) != 0;
#endif

        case Mode::Number::ALT_HOLD:
            return (enable_mask & (1 << 3)) != 0;

#if MODE_RTL_ENABLED
        case Mode::Number::RTL:
#endif
#if MODE_SMARTRTL_ENABLED
        case Mode::Number::SMART_RTL:
#endif
            // For RTL/SmartRTL, we may want to handle specially
            // (escalate to Land if obstacle during RTL)
            return (enable_mask & (1 << 0)) != 0;

        default:
            return false;
    }
}

// check activation condition (RC master switch AND altitude gating)
bool Copter::check_fwdavd_activation()
{
    // Check RC channel master switch - must be configured to valid channel (6-11)
    const int8_t rc_chan_num = g2.fwdavd_chan.get();
    if (rc_chan_num < 5 || rc_chan_num > 11) {
        // No valid RC channel configured - feature disabled
        return false;
    }

    const RC_Channel *rc_chan = rc().channel(rc_chan_num - 1);
    if (rc_chan == nullptr || failsafe.radio) {
        return false;
    }
    if (rc_chan->get_radio_in() <= 1500) {
        // RC switch is low - feature disabled
        return false;
    }

    // Now check altitude condition
    switch (g2.fwdavd_cond.get()) {
        case 0:
            // no altitude check - active if RC enabled
            return true;

        case 1: {
            // altitude above home/EKF origin
            const float alt_m = inertial_nav.get_position_z_up_cm() * 0.01f;
            const float threshold_m = g2.fwdavd_alt;
            if (alt_m < threshold_m) {
                // notify periodically when RC is on but altitude is too low
                static uint32_t last_alt_warn_ms = 0;
                if (last_alt_warn_ms == 0 || (AP_HAL::millis() - last_alt_warn_ms) > 5000) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Forward Avoidance: Below alt (%.1fm < %.1fm)",
                                   (double)alt_m, (double)threshold_m);
                    last_alt_warn_ms = AP_HAL::millis();
                }
                return false;
            }
            return true;
        }

        case 2: {
            // terrain altitude
#if AP_TERRAIN_AVAILABLE
            if (terrain.enabled()) {
                float terr_alt = 0.0f;
                if (terrain.height_above_terrain(terr_alt, true)) {
                    fwdavd_state.terrain_problem = false;
                    const float threshold_m = g2.fwdavd_alt;
                    if (terr_alt < threshold_m) {
                        // notify periodically when RC is on but terrain altitude is too low
                        static uint32_t last_terr_alt_warn_ms = 0;
                        if (last_terr_alt_warn_ms == 0 || (AP_HAL::millis() - last_terr_alt_warn_ms) > 5000) {
                            gcs().send_text(MAV_SEVERITY_INFO, "Forward Avoidance: Below terrain alt (%.1fm < %.1fm)",
                                           (double)terr_alt, (double)threshold_m);
                            last_terr_alt_warn_ms = AP_HAL::millis();
                        }
                        return false;
                    }
                    return true;
                } else {
                    // no valid terrain data
                    fwdavd_state.terrain_problem = true;
                    // warn periodically
                    if (fwdavd_state.last_terrain_warn_ms == 0 ||
                        (AP_HAL::millis() - fwdavd_state.last_terrain_warn_ms) > 10000) {
                        gcs().send_text(MAV_SEVERITY_WARNING, "Forward Avoidance: No terrain data");
                        fwdavd_state.last_terrain_warn_ms = AP_HAL::millis();
                    }
                    return false;
                }
            } else {
                fwdavd_state.terrain_problem = true;
                return false;
            }
#else
            fwdavd_state.terrain_problem = true;
            return false;
#endif
        }

        default:
            return false;
    }
}

// handle the avoidance action
void Copter::handle_fwdavd_action()
{
    const Mode::Number current_mode = flightmode->mode_number();

    // check if we're already landing - respect continue-if-landing behavior
    if (flightmode->is_landing()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Forward Avoidance: Continuing landing");
        fwdavd_state.triggered = true;
        return;
    }

    // check if we're already in RTL/SmartRTL and hit another obstacle - escalate to Land
    if (current_mode == Mode::Number::RTL ||
#if MODE_SMARTRTL_ENABLED
        current_mode == Mode::Number::SMART_RTL ||
#endif
        current_mode == Mode::Number::LAND) {
        // already in a return mode, escalate to Land
        if (current_mode != Mode::Number::LAND) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Forward Avoidance: Obstacle during RTL - Landing");
            set_mode(Mode::Number::LAND, ModeReason::AVOIDANCE);
        }
        fwdavd_state.triggered = true;
        return;
    }

    // determine if this is an auto mode or manual mode
    bool is_auto = flightmode->is_autopilot();

    FailsafeAction action;
    if (is_auto) {
        // auto mode - use auto action parameter (validate range 0-3)
        int8_t action_val = g2.fwdavd_act_auto.get();
        if (action_val < 0 || action_val > 3) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Forward Avoidance: Invalid auto action %d, using None", (int)action_val);
            action = FailsafeAction::NONE;
        } else {
            action = (FailsafeAction)action_val;
        }
    } else {
        // manual mode - use manual action parameter (validate range 0-4)
        int8_t action_val = g2.fwdavd_act_man.get();
        if (action_val < 0 || action_val > 4) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Forward Avoidance: Invalid manual action %d, using None", (int)action_val);
            action = FailsafeAction::NONE;
        } else if (action_val == 4) {
            // Stop with stick lockout for manual mode
            handle_fwdavd_stop_lockout();
            return;
        } else {
            action = (FailsafeAction)action_val;
        }
    }

    // execute the failsafe action
    gcs().send_text(MAV_SEVERITY_WARNING, "Forward Avoidance: Obstacle detected");
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FWDAVD, LogErrorCode::FAILSAFE_OCCURRED);

    fwdavd_state.triggered = true;
    fwdavd_state.prev_mode = current_mode;

    do_failsafe_action(action, ModeReason::AVOIDANCE);
}

// handle stop with stick lockout for manual modes
void Copter::handle_fwdavd_stop_lockout()
{
    gcs().send_text(MAV_SEVERITY_WARNING, "Forward Avoidance: Stop - Return sticks to neutral");
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FWDAVD, LogErrorCode::FAILSAFE_OCCURRED);

    fwdavd_state.triggered = true;
    fwdavd_state.stick_locked = true;
    fwdavd_state.prev_mode = flightmode->mode_number();

    // switch to Loiter if not already there
#if MODE_LOITER_ENABLED
    if (flightmode->mode_number() != Mode::Number::LOITER) {
        if (!set_mode(Mode::Number::LOITER, ModeReason::AVOIDANCE)) {
            // if we can't get to Loiter, try Brake then Land
#if MODE_BRAKE_ENABLED
            if (!set_mode(Mode::Number::BRAKE, ModeReason::AVOIDANCE)) {
#endif
                set_mode(Mode::Number::LAND, ModeReason::AVOIDANCE);
#if MODE_BRAKE_ENABLED
            }
#endif
        }
    }
#else
    // no Loiter, try Brake or Land
#if MODE_BRAKE_ENABLED
    if (!set_mode(Mode::Number::BRAKE, ModeReason::AVOIDANCE)) {
#endif
        set_mode(Mode::Number::LAND, ModeReason::AVOIDANCE);
#if MODE_BRAKE_ENABLED
    }
#endif
#endif
}

// check if pitch stick is neutral and obstacle is clear to unlock
void Copter::check_fwdavd_stick_unlock()
{
    if (!fwdavd_state.stick_locked) {
        return;
    }

    // check if pitch stick is neutral (within deadzone)
    // only check pitch since we only block forward pitch - roll is always allowed
    const int16_t pitch_in = channel_pitch->get_control_in();
    const int16_t deadzone = 100;  // ~10% stick movement allowed

    bool pitch_neutral = (abs(pitch_in) < deadzone);

    // check if obstacle is clear (counter back to zero)
    bool obstacle_clear = (fwdavd_state.thresh_count == 0);

    if (pitch_neutral && obstacle_clear) {
        fwdavd_state.stick_locked = false;
        fwdavd_state.triggered = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Forward Avoidance: Control restored");
    }
}

// returns true if sticks should be locked out due to forward avoidance
bool Copter::fwdavd_stick_locked() const
{
    return fwdavd_state.stick_locked;
}

#endif  // AP_RANGEFINDER_ENABLED
