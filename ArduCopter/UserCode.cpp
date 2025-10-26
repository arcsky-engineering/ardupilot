#include "Copter.h"

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
