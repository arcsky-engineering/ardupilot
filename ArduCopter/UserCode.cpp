#include "Copter.h"
#include <AP_Stats/AP_Stats.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

#define PWR_STATUS_BAD_SEND_INTERVAL 99// cycles at 3.3 Hz (about every 30 seconds)

uint16_t pwrStatusFlags = 0;
uint8_t pwrStatusFlagsInitialized = 0;
uint8_t pwrStatusGood = 1; // assume good from the start
uint8_t pwrStatusBadSendCnt = 0;

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
    // check to read power flags
    if (!pwrStatusFlagsInitialized)
    {
        pwrStatusFlags = hal.analogin->power_status_flags();
        if (pwrStatusFlags != 0)
        {
            pwrStatusFlagsInitialized = 1;
        }
    }
    else
    {
        // we have already initialized the power status flags
        // now we can compare for changes - specifically whether or not we lose or change power
        pwrStatusFlags = hal.analogin->power_status_flags();
        if ((pwrStatusFlags & MAV_POWER_STATUS_BRICK_VALID) && (pwrStatusFlags & MAV_POWER_STATUS_SERVO_VALID))
        {
            // power status is good (i.e. at least we have 2 sources available)
            pwrStatusGood = 1;
        }
        else
        {
            // power status is bad (i.e. at least 1 source is not working now)
            pwrStatusGood = 0;
        }
    }

    // report status of power status if bad
    if (!pwrStatusGood)
    {
        if (pwrStatusBadSendCnt < PWR_STATUS_BAD_SEND_INTERVAL)
        {
            pwrStatusBadSendCnt++;
        }
        else
        {
            uint8_t lostPwrSrc = 0;
            if (pwrStatusFlags & MAV_POWER_STATUS_BRICK_VALID)
            {
                // if this is true, we still have the primary, so the
                // secondary must have been the cause
                lostPwrSrc = 2;
            }
            else
            {
                // the primary must have been the cause
                lostPwrSrc = 1;
            }

            gcs().send_text(MAV_SEVERITY_CRITICAL, "Power Source %u Lost. Land Immediately!", lostPwrSrc);
            pwrStatusBadSendCnt = 0;
        }
    }
    else
    {
        // reset any counters
        pwrStatusBadSendCnt = 0;
    } // end of else - from if (!pwrStatusGood)
    
    
    // --------------------------------------------------------------------------------------------------------
    //      LOGIC FOR LOW VOLTAGE CHECKING
    // --------------------------------------------------------------------------------------------------------

    // --- Board Voltage Monitor ---
    static bool board_voltage_warn_triggered = false;
    static uint8_t board_voltage_warn_counter = 0;

    float board_voltage = hal.analogin->board_voltage();


    if (!board_voltage_warn_triggered && board_voltage < 4.9f) {
        // One-time trigger
        board_voltage_warn_triggered = true;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Internal Power Issue! Land Immediately!");
    } else if (board_voltage_warn_triggered) {
        // Already triggered, send periodic warnings every ~30s (3.3 Hz * 30 = ~100 loops)
        if (++board_voltage_warn_counter >= 100) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Internal Power Issue! Land Immediately!");
            board_voltage_warn_counter = 0;
        }
    } // end of else if (!board_voltage_warn_triggered)

    // --------------------------------------------------------------------------------------------------------
    //      ARCSKY TELEMETRY METADATA (~ prefix: logged to tlog, hidden from GCS UI)
    //      Messages are staggered: one message every 30s, alternating between types
    // --------------------------------------------------------------------------------------------------------
    static uint8_t metadataCnt;
    static uint8_t metadataSlot; // 0 = drone identity, 1+ = batteries
    if (++metadataCnt >= 100) // 100 ticks at 3.3 Hz ≈ 30 seconds
    {
        if (metadataSlot == 0)
        {
            // --- Drone identity: ~ARCSKY,XPLORER,<serial>,<flight_hours> ---
#if AP_STATS_ENABLED
            AP_Stats *ap_stats = AP::stats();
            float fltHours = (ap_stats != nullptr) ? (float)(ap_stats->flttime) / 3600.0f : 0.0f;
#else
            float fltHours = 0.0f;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            const char *uas_id = "1924A0226040004"; // test serial for SITL
#elif AP_OPENDRONEID_ENABLED
            const char *uas_id = copter.opendroneid.get_uas_id();
#else
            const char *uas_id = nullptr;
#endif
            if (uas_id != nullptr) {
                gcs().send_text(MAV_SEVERITY_INFO, "~ARCSKY,XPLORER,%s,%.1f", uas_id, fltHours);
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "~ARCSKY,XPLORER,NO_SN,%.1f", fltHours);
            }
        }
        else
        {
            // --- Battery serial + cycle count: ~BAT<n>,<model_name>,<cycles> ---
            // Send one battery per slot (slot 1 = battery 0, slot 2 = battery 1, etc.)
            uint8_t batIdx = metadataSlot - 1;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            // Simulated DroneCAN batteries for SITL
            static const char *simBatNames[] = {"Arcsky-2603001", "Arcsky-2603002"};
            static const uint16_t simBatCycles[] = {42, 87};
            if (batIdx < 2) {
                gcs().send_text(MAV_SEVERITY_INFO, "~BAT%u,%s,%u",
                                (unsigned)batIdx, simBatNames[batIdx], (unsigned)simBatCycles[batIdx]);
            }
#else
            if (batIdx < AP::battery().num_instances()) {
                if (AP::battery().option_is_set(batIdx, AP_BattMonitor_Params::Options::InternalUseOnly)) {
                    // skip internal-only batteries
                } else {
                    const char *model = AP::battery().get_model_name(batIdx);
                    uint16_t cycles = 0;
                    AP::battery().get_cycle_count(batIdx, cycles);
                    if (model != nullptr) {
                        gcs().send_text(MAV_SEVERITY_INFO, "~BAT%u,%s,%u",
                                        (unsigned)batIdx, model, (unsigned)cycles);
                    } else {
                        int32_t sn = AP::battery().get_serial_number(batIdx);
                        if (sn > 0) {
                            gcs().send_text(MAV_SEVERITY_INFO, "~BAT%u,%ld,%u",
                                            (unsigned)batIdx, (long)sn, (unsigned)cycles);
                        }
                    }
                }
            }
#endif
        }

        // Rotate slots: 0=drone, 1=bat0, 2=bat1, then wrap
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        const uint8_t totalSlots = 3; // drone + 2 simulated batteries
#else
        const uint8_t totalSlots = 1 + AP::battery().num_instances();
#endif
        metadataSlot = (metadataSlot + 1) % totalSlots;
        metadataCnt = 0;
    }

} // end of userhook_SlowLoop
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
