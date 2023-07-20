#include <AP_SerialManager/AP_SerialManager.h>
#include "Copter.h"

//TODO - calculate current difference between the generator current and the system current


// used for analog stuff only
//extern const AP_HAL::HAL& hal;

// used for failsafe actions?
//extern Copter copter;

// adding this to access voltage and current from battery monitoring
//const AP_BattMonitor &battery = AP::battery();

//#define TEMP_ERROR_SEND_INTERVAL 602 // cycles at 50 Hz (0.02 seconds)
//#define CURRENT_ERROR_SEND_INTERVAL 501 // cycles at 50 Hz (0.02) seconds
//#define FUEL_WARNING_SEND_INTERVAL 610 // cycles at 50 Hz

#define RFND_THRESH_READINGS 10 // consecutive counts of the 10 Hz loop before rangefinder failsafe triggered

#define TEMP_ERROR_SEND_INTERVAL 41 // cycles at 3.3 Hz (0.303 seconds)
#define CURRENT_ERROR_SEND_INTERVAL 32 // cycles at 3.3 Hz (0.303) seconds
#define FUEL_WARNING_SEND_INTERVAL 42 // cycles at 3.3 Hz (0.303) seconds
#define GENERATOR_TIMEOUT_SEND_INTERVAL 43 // cycles at 3.3 Hz
#define VOLTAGE_WRN_SEND_INTERVAL 37 // cycles at 3.3 Hz

#define PWR_STATUS_BAD_SEND_INTERVAL 99// cycles at 3.3 Hz (about every 30 seconds)

#define GENERATOR_TIMEOUT_ERROR_CNT 33 // cycles at 3.3 Hz (equates to about 10 seconds)

#define LOW_VOLTAGE_WARNING_CNT 33 // cycles at 3.3 Hz (equates to about 10 seconds)

#define ALFA_VOLTAGE_FILTER 0.003 // filter amount for new value
#define ALFA_VOLTAGE_FILTER_INV 0.997 // so we don't have to do this math every time

#define GENERATOR_LOW_VOLTAGE_LEVEL 47.0 // low voltage level after which we should be concerned about being able to land

// reported mode from the generator:
enum GenMode {
    IDLE = 0,
    RUN = 1,
    CHARGE = 2,
    BALANCE = 3,
    OFF = 4,
};

// un-packed data from the generator:
struct Reading {
    uint32_t    runtime; //seconds
    int32_t    seconds_until_maintenance;
    uint16_t    errors;
    uint16_t    rpm;
    float       output_voltage;
    float       filtered_output_voltage;
    uint8_t     filtered_voltage_initialized;
    uint8_t     lowVoltageWarningSent;
    uint8_t     low_voltage_wrn_cnt;
    uint8_t     low_voltage_warning;
    uint8_t     lowVoltageWrnSendCnt;
    float       output_current;
    GenMode     mode;
    float       pwrIntegral;
    float       pwrGenerated;
    float       batt_current;
    float       batt_current_setpoint;
    int16_t     rectTemp;
    int16_t     genTemp;
    uint16_t    servoCmd;
    // these are from status flag
    uint16_t    pwm_avg;
    uint8_t     currentPwmInputState;
    uint8_t     detectedPwmState;
    uint8_t     operateMode;
    uint8_t     requestedOperateMode;
    uint8_t     operateModeTransitionActive;
    uint8_t     engineKillState;
    //uint16_t  servoSetVal;
    uint16_t    errorStatus;
    uint16_t    ctrlOutputFilt;

    // new things (Jan 19, 2022)
    uint8_t     fuelPct;
    uint8_t     engineDied;
    uint8_t     engineDiedNoticeSent;

    uint8_t tempError;
    uint16_t tempErrorSendCnt;
    uint8_t tempErrorSet;

    uint8_t currentError;
    uint16_t currentErrorSendCnt;
    uint8_t currentErrorSet;

    uint8_t currentErrorFsHandled;

    uint16_t fuelWarningSendCnt;
    uint8_t fuelWarningSet;

    uint8_t fuelFailsafeTriggered;

    uint8_t generatorDetected;

    uint16_t generatorTimeoutCnt;
    uint8_t generatorTimeoutError;
    uint16_t generatorTimeoutErrorSendCnt;
    uint8_t generatorTimeoutErrorSet;

    // new EFI data stuff added June 2023
    uint8_t efiMsgReceived;

    // pulseWidth1
    uint16_t pulseWidthms;
    // rpm
    uint16_t efiRPM;
    // afrtgt1
    uint8_t afrtgt;
    // baro
    int16_t baro;
    // mat
    int16_t mat;
    // clt
    int16_t clt;
    // tps
    int16_t tps;
    // bv
    int16_t bv;
    // afr1
    int16_t afr1;
    // aircor
    int16_t aircor;
    // warmcor
    int16_t warmcor;
    // accelEnrich
    int16_t accelEnrich;
    // tpsfuelcut
    int16_t tpsfuelcut;
    // baroCorrection
    int16_t baroCorrection;
    // gammaEnrich
    int16_t gammaEnrich;
    // ve1
    int16_t ve1;
    // TPSdot
    int16_t TPSdot;
    // fuelload
    uint8_t fuelload;    

    uint8_t msgType; // type of message received by system (1 = GENERATOR / 2 = EFI)
};

// declare some variables to use
struct Reading last_reading;

uint8_t startingFuelPct;
uint8_t fuelPctInitialized;
float lastCurrent;
float fuelPctLocal;
uint16_t fuelSendCnt;

uint16_t genRadioCmd;
//uint16_t lastGenRadioCmd;
uint16_t genCmdOut;

uint8_t killState = 0;
uint8_t killOverride = 0;

uint64_t status;

GenMode currentGenMode = IDLE;

AP_HAL::UARTDriver *uart;

uint8_t RxBuf[40] = {0};
// number of bytes currently in the buffer
uint8_t body_length;

uint8_t fs_engine = 0;
uint8_t fs_oc = 0;
uint8_t fs_fuel = 0;
uint8_t fs_fuel_pct = 0;
uint8_t fuel_warn_pct = 0;

uint32_t droneFlightTime = 0;
uint8_t runtimesent = 0;

uint8_t rfnd_thresh_cnt = 0;
uint8_t rfnd_fs_triggered = 0;
uint8_t rfnd_fs = 0;
float rfnd_avoid_thresh = 0;
float rfnd_active_alt = 0;
uint8_t rfnd_avoid_active = 0;
uint8_t rfnd_avoid_cnts = 0;

uint16_t pwrStatusFlags = 0;
uint8_t pwrStatusFlagsInitialized = 0;
uint8_t pwrStatusGood = 1; // assume good from the start
uint8_t pwrStatusBadSendCnt = 0;

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    fs_engine = (uint8_t)(g.gen_fs);
    fs_oc = (uint8_t)(g.gen_fs_oc);
    fs_fuel = (uint8_t)(g.gen_fuel_fs_action);
    fs_fuel_pct = (uint8_t)(g.gen_fuel_fs_pct);
    fuel_warn_pct = (uint8_t)(g.gen_fuel_warn_pct);
    droneFlightTime = g2.stats.flttime;

    // rangefinder stuff
    rfnd_fs = (uint8_t)(g.rfnd_fs_action);
    rfnd_avoid_thresh = (float)(g.rfnd_avoid_dist_cm);
    rfnd_active_alt = (float)(g.rfnd_avoid_active_alt_cm);
    rfnd_avoid_active = (uint8_t)(g.rfnd_avoid_active);
    rfnd_avoid_cnts = (uint8_t)(g.rfnd_avoid_cnts);



    last_reading.engineDiedNoticeSent = 0;
    last_reading.generatorDetected = 0;
    last_reading.tempErrorSet = 0;
    last_reading.currentErrorSet = 0;
    last_reading.currentErrorFsHandled = 0;
    last_reading.fuelFailsafeTriggered = 0;
    last_reading.tempErrorSendCnt = TEMP_ERROR_SEND_INTERVAL; // so that they send right away the first time
    last_reading.currentErrorSendCnt = CURRENT_ERROR_SEND_INTERVAL; // so that they send right away the first time
    last_reading.generatorTimeoutCnt = 0;
    last_reading.generatorTimeoutErrorSendCnt = GENERATOR_TIMEOUT_SEND_INTERVAL;
    fuelSendCnt = 0;
    last_reading.filtered_output_voltage = 0;

    last_reading.lowVoltageWarningSent = 0;
    last_reading.low_voltage_wrn_cnt = 0;
    last_reading.low_voltage_warning = 0;
    last_reading.lowVoltageWrnSendCnt = 0;

    // initialize the serial manager, according to how it's done in RichenPower
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Generator, 0);
    if (uart != nullptr) {
        //const uint32_t baud = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Generator, 0);
        //uart->begin(baud, 256, 256);
        // try 57600 directly
        uart->begin(57600,256,256);
    }

    fuelPctInitialized = 0;

    status = 0;

    // set output to be "OFF" initially for the generator
    genCmdOut = 1500;
    SRV_Channels::set_output_pwm(SRV_Channel::k_generator_control, genCmdOut);
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

} // end of 50 hz loop
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
#if RANGEFINDER_ENABLED == ENABLED

    rfnd_avoid_active = (uint8_t)(g.rfnd_avoid_active);

    if (rfnd_avoid_active)
    {
        // check if we are above the active altitude for this feature
        if (copter.inertial_nav.get_position_z_up_cm() >= rfnd_active_alt)
        {
            if (rangefinder_forward_state.enabled)
            {
                if (rangefinder_forward_state.alt_healthy)
                {
                    // read and display the rangefinder value here
                    float tempAlt;

                    tempAlt = rangefinder_forward_state.alt_cm_filt.get();

                    // keep checking if the rangefinder value is equal to or less than the
                    // set threshold for taking action
                    if (tempAlt < rfnd_avoid_thresh)
                    {
                        // another reading less than threshold value has processed
                        if (rfnd_thresh_cnt < rfnd_avoid_cnts)
                        {
                            // increment the counter
                            rfnd_thresh_cnt++;
                        }
                        else
                        {
                            // we have hit the threshold already and need to decide whether
                            // to take action, or make sure we have already taken action.
                            if (!rfnd_fs_triggered)
                            {
                                // trigger failsafe based on user parameter configured
                                gcs().send_text(MAV_SEVERITY_WARNING, "Range Finder Avoid Fail Safe");
                                gcs().send_text(MAV_SEVERITY_INFO, "Range Finder Avoid Fail Safe %u",rfnd_fs);

                                FailsafeAction desired_rfnd_fs_action;
                                desired_rfnd_fs_action = (FailsafeAction)(rfnd_fs);

                                // call functions (possibly write custom function) in events.cpp file

                                // Support the CONTINUE_IF_LANDING feature to prevent return to RTL height
                                if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING) && desired_rfnd_fs_action != FailsafeAction::NONE)
                                {
                                    desired_rfnd_fs_action = FailsafeAction::LAND;
                                    announce_failsafe("Range", "Continuing Landing");
                                }
                                copter.do_failsafe_action(desired_rfnd_fs_action, ModeReason::FAILSAFE);
                                //do_failsafe_action(Failsafe_Action action, ModeReason reason)

                                rfnd_fs_triggered = 1;
                            }
                            else
                            {
                                // still warn periodically if we see something again in range?
                            }
                            // once failsafe has already been triggered, do not allow it to
                            // be triggered again, even if we see something that could
                            // trigger it
                        } // else - from if counter < threshold
                    } // if the rangefinder reading was less than the threshold
                    else
                    {
                        // rangefinder reading within proper range

                        // decrement counter if non-zero
                        if (rfnd_thresh_cnt)
                        {
                            rfnd_thresh_cnt--;
                        }
                    }


//                    // TEMPORARY FOR DEBUGGING - send every 5 seconds (50 iterations at 10 Hz)
//                    static uint8_t counterRF;
//
//                    if (counterRF < 50)
//                    {
//                        counterRF++;
//                    }
//                    else
//                    {
//                        // send message to GCS to indicate we read the message successfully
//                        gcs().send_text(MAV_SEVERITY_INFO,"RF READING: %.1f cm",tempAlt);
//                        counterRF = 0;
//                    }
                } // if forward facing rangefinder is healthy
            } // if forward facing rangefinder is enabled
        } // if we are above or equal to the active altitude
        else
        {
            // we are below the active altitude

            // don't do any other action
        }
    } // if avoidance is active (set by parameter)
#endif // if rangefinder is enabled by firmware
} // Medium loop
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

            gcs().send_text(MAV_SEVERITY_CRITICAL, "Power Source %u Lost", lostPwrSrc);
            pwrStatusBadSendCnt = 0;
        }
    }
    else
    {
        // reset any counters
        pwrStatusBadSendCnt = 0;
    }

    static uint8_t counter1;
    // initialization only - for displaying generator (and UAV) runtime
    if (!runtimesent)
    {
        if (counter1 < 180) // 180 loops of 3.3 Hz (about 60 seconds)
        {
            counter1++;
        }
        else
        {
            if(last_reading.generatorDetected)
            {
                // only send the generator runtime message if generator has been detected
                float genHours;
                genHours = (float)(last_reading.runtime) / 3600.0; 
                gcs().send_text(MAV_SEVERITY_INFO,"GEN HOURS: %.1f",genHours);
            }

            // send the drone runtime message regardless
            float flightHours;
            flightHours = (float)(droneFlightTime) / 3600.0;
            gcs().send_text(MAV_SEVERITY_INFO,"FLIGHT HOURS: %.1f",flightHours);

            runtimesent = 1;
        }
    } // if (!runtimeSent)



    // check for value on channel 9
    // channels are 0 index based
    // only do so if failsafe is not active
    if (!(failsafe.radio))
    {
        genRadioCmd = hal.rcin->read(8);
    }
    else
    {
        genRadioCmd = 1000; // low is default value - no kill
    }

    if (!killOverride)
    {
        if (genRadioCmd > 1500)
        {
            // request kill state
            killState = 1;
        }
        else if (genRadioCmd > 800)
        {
            // normal state - no kill
            killState = 0;
        }
        else
        {
            // kill if something is weird?
            // TODO - figure out if this is a good way to handle it
            killState = 0;
        }
    }

    // if we are not in kill state, go ahead and set run/idle commands
    if (!killState)
    {
        // check for vehicle arm state
        if(AP::arming().is_armed())
        {
            if (currentGenMode != RUN)
            {
                currentGenMode = RUN;
                // set pwm output
                genCmdOut = 2000;
                SRV_Channels::set_output_pwm(SRV_Channel::k_generator_control, genCmdOut);
            }
        } // if armed
        else
        {
            // not armed
            if (currentGenMode != IDLE)
            {
                currentGenMode = IDLE;
                // set pwm output
                genCmdOut = 1500;
                SRV_Channels::set_output_pwm(SRV_Channel::k_generator_control, genCmdOut);
            }
        } // else - from if vehicle was armed
    } // if (!killState)
    else // kill state is active - keep setting the kill command on PWM output
    {
        currentGenMode = OFF;
        genCmdOut = 1000;
        SRV_Channels::set_output_pwm(SRV_Channel::k_generator_control, genCmdOut);
    }


    // check for UART and process it into data, populate last_reading structure

    if (get_reading())
    {
        if (last_reading.msgType == 1) // generator message
        {
            if (!last_reading.filtered_voltage_initialized)
            {
                if (last_reading.output_voltage > 20.0)
                {
                    last_reading.filtered_output_voltage = last_reading.output_voltage;
                    last_reading.filtered_voltage_initialized = 1;
                }
            } // if !filtered_voltage_initialized
            else
            {
                // it has been initialized - do the filtering here
                last_reading.filtered_output_voltage = (ALFA_VOLTAGE_FILTER * last_reading.output_voltage) + (ALFA_VOLTAGE_FILTER_INV * last_reading.filtered_output_voltage);
            }

            // do the low voltage check algorithm here
            if (last_reading.filtered_output_voltage < GENERATOR_LOW_VOLTAGE_LEVEL)
            {
                if (last_reading.low_voltage_wrn_cnt < LOW_VOLTAGE_WARNING_CNT)
                {
                    last_reading.low_voltage_wrn_cnt++;
                }
                else
                {
                    last_reading.low_voltage_warning = 1;
                }
            }
            else
            { // voltage is above low warning level
                if (last_reading.low_voltage_warning)
                { // if the warning is already active
                    if (last_reading.low_voltage_wrn_cnt)
                    {
                        last_reading.low_voltage_wrn_cnt--;
                    }
                    else
                    {
                        last_reading.low_voltage_warning = 0;
                    }
                }
            }

            if (last_reading.low_voltage_warning)
            {
                // if it's active at all
                if (last_reading.lowVoltageWrnSendCnt < VOLTAGE_WRN_SEND_INTERVAL)
                {
                    last_reading.lowVoltageWrnSendCnt++;
                }
                else
                {

                    // warn the user
                    gcs().send_text(MAV_SEVERITY_WARNING,"GEN VOLTAGE LOW!");

                    // set sent flag
                    last_reading.lowVoltageWarningSent = 1;

                    // reset counter
                    last_reading.lowVoltageWrnSendCnt = 0;
                }
            } // if last_reading.low_voltage_warning
            else
            {
                if (last_reading.lowVoltageWarningSent)
                {
                    gcs().send_text(MAV_SEVERITY_WARNING,"GEN VOLTAGE NORMAL");
                    last_reading.lowVoltageWarningSent = 0;
                }
            }


            // temporary
            //gcs().send_text(MAV_SEVERITY_INFO, "DBGFUEL: %u%%",last_reading.fuelPct);

            // update the flag to indicate that we have detected the generator in the system
            if (!last_reading.generatorDetected)
            {
                last_reading.generatorDetected = 1;
            }

            // reset timeout count since we have received data
            last_reading.generatorTimeoutCnt = 0;

            if (last_reading.generatorTimeoutErrorSet)
            {
                last_reading.generatorTimeoutErrorSet = 0;
            }
        }
        else if (last_reading.msgType == 2)
        {
            // EFI message received

            // do any updating here that is necessary
        }

    } // if (get_reading())

    if (last_reading.engineDied)
    {
        if (!last_reading.engineDiedNoticeSent)
        {
            // failsafe stuff for engine stoppage
            gcs().send_text(MAV_SEVERITY_WARNING, "Generator Failsafe");
            gcs().send_text(MAV_SEVERITY_INFO, "Generator Failsafe %u",fs_engine);
            // TODO - in the future, add failsafe actions and configurable ways of
            // managing this, possibly by modifying parameters and using the previous
            // GEN parameters (e.g. g.gen_fuel_pct) to be g.gen_fs_action.


            FailsafeAction desired_gen_fs_action;
            desired_gen_fs_action = (FailsafeAction)(fs_engine);

            // call functions (possibly write custom function) in events.cpp file
            copter.do_failsafe_action(desired_gen_fs_action, ModeReason::FAILSAFE);
            //do_failsafe_action(Failsafe_Action action, ModeReason reason)

            last_reading.engineDiedNoticeSent = 1;
        }
    } // if (last_reading.engineDied)
    else
    {
        // check if we had previously died and need to recover
        // TODO
        if (last_reading.engineDiedNoticeSent)
        {
            last_reading.engineDiedNoticeSent = 0;
        }
    }


    if (last_reading.tempError)
    {
        // if it's active at all
        if (last_reading.tempErrorSendCnt < TEMP_ERROR_SEND_INTERVAL)
        {
            last_reading.tempErrorSendCnt++;
        }
        else
        {
            if (!last_reading.tempErrorSet)
            {
                last_reading.tempErrorSet = 1;
            }
            // send custom message based on the type of error
            switch(last_reading.tempError)
            {
            case 1:
                gcs().send_text(MAV_SEVERITY_WARNING, "RECTIFIER TEMP WARNING!");
                break;
            case 2:
                gcs().send_text(MAV_SEVERITY_CRITICAL, "RECTIFIER TEMP ERROR!");
                break;
            case 3:
                gcs().send_text(MAV_SEVERITY_WARNING, "RECT TEMP SENS FAIL!");
                break;
            default:
                break;
            }

            // reset counter
            last_reading.tempErrorSendCnt = 0;
        }
    } // if last_reading.tempError
    else
    {
        if(last_reading.tempErrorSet)
        {
            // tell GCS that it's cleared
            gcs().send_text(MAV_SEVERITY_WARNING, "TEMP SENS OK!");
            last_reading.tempErrorSet = 0;
        }
    }

    if (last_reading.currentError)
    {
        if (last_reading.currentError == 2)
        {
            if (!last_reading.currentErrorFsHandled)
            {
                // kill engine should be one of those options
                    gcs().send_text(MAV_SEVERITY_ERROR, "Generator Batt Failsafe");
                    gcs().send_text(MAV_SEVERITY_INFO, "Generator Batt Failsafe %u",fs_oc);

                    if (fs_oc < 6)
                    {
                        FailsafeAction desired_gen_fs_oc_action;
                        desired_gen_fs_oc_action = (FailsafeAction)(fs_oc);

                        // call functions (possibly write custom function) in events.cpp file
                        copter.do_failsafe_action(desired_gen_fs_oc_action, ModeReason::FAILSAFE);
                        //do_failsafe_action(Failsafe_Action action, ModeReason reason)
                    }
                    else if (fs_oc == 6)
                    {
                        // kill engine
                        killOverride = 1;
                        killState = 1;

                    }
                    last_reading.currentErrorFsHandled = 1;
            }
        }
        else
        {
            if (killOverride)
            {
                killOverride = 0;
            }
        }
        // if it's active at all
        if (last_reading.currentErrorSendCnt < CURRENT_ERROR_SEND_INTERVAL)
        {
            last_reading.currentErrorSendCnt++;
        }
        else
        {
            if (!last_reading.currentErrorSet)
            {
                last_reading.currentErrorSet = 1;
            }
            // send custom message based on the type of error
            switch(last_reading.currentError)
            {
            case 1:
                gcs().send_text(MAV_SEVERITY_WARNING, "BATT CURRENT WARNING!");
                break;
            case 2:
                gcs().send_text(MAV_SEVERITY_CRITICAL, "BATT OVERCHARGE!");
                break;
            case 3:
                gcs().send_text(MAV_SEVERITY_WARNING, "BATT CUR SENS FAIL!");
                break;
            default:
                break;
            }

            // reset counter
            last_reading.currentErrorSendCnt = 0;
        }
    } // if last_reading.currentError
    else
    {
        if(last_reading.currentErrorSet)
        {
            gcs().send_text(MAV_SEVERITY_WARNING, "BATT CURR OK!");
            last_reading.currentErrorSet = 0;
        }
    }

    if(last_reading.generatorDetected)
    {
        if (last_reading.fuelPct < fs_fuel_pct)
        {
            if (!last_reading.fuelFailsafeTriggered)
            {
                // kill engine should be one of those options
                    gcs().send_text(MAV_SEVERITY_ERROR, "Generator Fuel Failsafe");
                    gcs().send_text(MAV_SEVERITY_INFO, "Generator Fuel Failsafe %u",fs_fuel);
                    FailsafeAction desired_gen_fs_fuel_action;
                    desired_gen_fs_fuel_action = (FailsafeAction)(fs_fuel);

                    // call functions (possibly write custom function) in events.cpp file

                    // Support the CONTINUE_IF_LANDING feature to prevent return to RTL height
                    if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING) && desired_gen_fs_fuel_action != FailsafeAction::NONE)
                    {
                        desired_gen_fs_fuel_action = FailsafeAction::LAND;
                        announce_failsafe("GEN", "Continuing Landing");
                    }
                    copter.do_failsafe_action(desired_gen_fs_fuel_action, ModeReason::FAILSAFE);
                    //do_failsafe_action(Failsafe_Action action, ModeReason reason)

                    last_reading.fuelFailsafeTriggered = 1;
            }


            if (!last_reading.fuelWarningSet)
            {
                last_reading.fuelWarningSet = 1;
            }
            if (last_reading.fuelWarningSendCnt < FUEL_WARNING_SEND_INTERVAL)
            {
                last_reading.fuelWarningSendCnt++;
            }
            else
            {
                // send mavlink message
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Low Fuel %u%%",last_reading.fuelPct);
                last_reading.fuelWarningSendCnt = 0;
            }
        } // if fuel level is less than failsafe level
        else if (last_reading.fuelPct < fuel_warn_pct)
        {
            if (!last_reading.fuelWarningSet)
            {
                last_reading.fuelWarningSet = 1;
            }
            if (last_reading.fuelWarningSendCnt < FUEL_WARNING_SEND_INTERVAL)
            {
                last_reading.fuelWarningSendCnt++;
            }
            else
            {
                // send mavlink message
                gcs().send_text(MAV_SEVERITY_WARNING, "Low Fuel %u%%",last_reading.fuelPct);
                last_reading.fuelWarningSendCnt = 0;
            }
        }
        else
        {
            if (last_reading.fuelWarningSet)
            {
                last_reading.fuelWarningSet = 0;
                last_reading.fuelFailsafeTriggered = 0;
            }
        }
    } // only do this section if the generator is detected

    last_reading.mode = currentGenMode;


    // LOGGING SECTION ****************************************************

    static uint8_t counter2;
    counter2++;
    if (counter2 > 1)
    {

        counter2 = 0;
        // log //    log runtime, current, power, mode, etc.
        if (last_reading.generatorDetected)
        {
            AP::logger().Write(
                "GEN",
                "TimeUS,trn,tma,thr,rpm,V,A,Ab,Tm,Tg,md,pa,om,rm,omt,fp",
                "s---qvAAOO------",
                "F---------------",
                "QIIHHfffhhBHBBBB",
                AP_HAL::micros64(),
                last_reading.runtime,
                last_reading.seconds_until_maintenance,
                last_reading.servoCmd,
                last_reading.rpm,
                //last_reading.output_voltage,
                last_reading.filtered_output_voltage,
                last_reading.output_current,
                last_reading.batt_current,
                last_reading.rectTemp,
                last_reading.genTemp,
                last_reading.mode,
                last_reading.pwm_avg,
                last_reading.operateMode,
                last_reading.requestedOperateMode,
                last_reading.operateModeTransitionActive,
                last_reading.fuelPct
                );
        }

        if (last_reading.efiMsgReceived)
        {
            // EFI Message logging
            AP::logger().Write(
                "EFI",
                "TimeUS,rpm,afrt,bar,mat,clt,tps,bv,af1,ac,wc,ae,tfc,bc,ge,pw",
                "sq--OO%v-%%%%%%-",
                "F-AAAAAAAAAAAAAC",
                "QHBhhhhhhhhhhhhH",
                AP_HAL::micros64(),
                last_reading.efiRPM,
                last_reading.afrtgt,
                last_reading.baro,
                last_reading.mat,
                last_reading.clt,
                last_reading.tps,
                last_reading.bv,
                last_reading.afr1,
                last_reading.aircor,
                last_reading.warmcor,
                last_reading.accelEnrich,
                last_reading.tpsfuelcut,
                last_reading.baroCorrection,
                last_reading.gammaEnrich,
                last_reading.pulseWidthms
                );
        }        

    } // counter 2

    // only do this if we have detected the generator at least once
    if (last_reading.generatorDetected)
    {
        // increment timeout counter (it is reset earlier if we receive a reading)
        if (last_reading.generatorTimeoutCnt < GENERATOR_TIMEOUT_ERROR_CNT)
        {
            last_reading.generatorTimeoutCnt++;
        }
        else
        {
            // we have hit the timeout count
            if (!last_reading.generatorTimeoutErrorSet)
            {
                last_reading.generatorTimeoutErrorSet = 1;
            }

            if (last_reading.generatorTimeoutErrorSendCnt < GENERATOR_TIMEOUT_SEND_INTERVAL)
            {
                last_reading.generatorTimeoutErrorSendCnt++;
            }
            else
            {
                // send mavlink message
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Hybrid Module Communication Lost");
                last_reading.generatorTimeoutErrorSendCnt = 0;
            }
        }
    }
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

//send mavlink efi status
void Copter::send_efi_status(const GCS_MAVLINK &channel)
{

    // only send the MAVLink message if the Hybrid Module is detected in the system
    if (last_reading.efiMsgReceived)
    {

        // populate some dummy variables for things we don't have
        // available

        // if these variables don't work for some reason (following data type of EFI_STATUS message online)
        // then possibly use the data types found in the AP_EFI.cpp file
        float ecu_index = 1;
        float dummyZero = 0.001;
        float erpm = ((float)(last_reading.efiRPM));
        float engineLoad = ((float)(last_reading.fuelload))*0.1;
        float tps = ((float)(last_reading.tps))*0.1;
        float baroPress = ((float)(last_reading.baro))*0.1;
        float mat = ((float)(last_reading.mat))*0.1;
        float clt = ((float)(last_reading.clt))*0.1;
        float injtime = ((float)(last_reading.pulseWidthms))*0.001;
        float ptcomp = ((float)(last_reading.baroCorrection))*0.1;
        float bv = ((float)(last_reading.bv))*0.1;

        mavlink_msg_efi_status_send(
        channel.get_chan(),
        (float)(last_reading.efiMsgReceived),
        ecu_index,
        erpm,
        dummyZero,
        dummyZero,
        engineLoad,
        tps,
        dummyZero,
        baroPress,
        dummyZero,
        mat,
        clt,
        dummyZero,
        injtime,
        dummyZero,
        tps,
        ptcomp,
        bv
        );
    }


} // end of send_efi_status

//send mavlink generator status
void Copter::send_generator_status(const GCS_MAVLINK &channel)
{

    status = 0;

    switch(last_reading.mode)
    {
    case GenMode::OFF:
        status |= MAV_GENERATOR_STATUS_FLAG_OFF;
        break;
    case GenMode::IDLE:
        status |= MAV_GENERATOR_STATUS_FLAG_IDLE;
        break;
    case GenMode::RUN:
        status |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
        status |= MAV_GENERATOR_STATUS_FLAG_CHARGING;
        break;
    default:
        status |= MAV_GENERATOR_STATUS_FLAG_OFF;
        break;
    }

    // only send the MAVLink message if the Hybrid Module is detected in the system
    if (last_reading.generatorDetected)
    {
        mavlink_msg_generator_status_send(
        channel.get_chan(),
        status,//
        last_reading.rpm, // generator_speed
        last_reading.batt_current, // battery_current; current into/out of battery
        last_reading.output_current, // load_current; Current going to UAV
        last_reading.pwrGenerated,
        last_reading.output_voltage, // bus_voltage; Voltage of the bus seen at the generator
        last_reading.rectTemp, // rectifier_temperature
        last_reading.batt_current_setpoint, // bat_current_setpoint; The target battery current
        last_reading.genTemp, // generator temperature
        last_reading.runtime,
        last_reading.seconds_until_maintenance
        );
    }


} // end of send_generator_status

// read - read serial port, return true if a new reading has been found
bool Copter::get_reading()
{

    // fill our buffer some more:
    uint32_t nbytes = uart->read(&RxBuf[body_length],
                                 ARRAY_SIZE(RxBuf)-body_length);
    if (nbytes == 0) {
        return false;
    }
    body_length += nbytes;

    move_header_in_buffer(0);

    // header byte 1 is correct.
    if (body_length < ARRAY_SIZE(RxBuf)) {
        // need a full buffer to have a valid message...
        return false;
    }

    if (RxBuf[1] != HEADER_MAGIC2) {
        move_header_in_buffer(1);
        return false;
    }

    // check for the footer signature:
    if ((RxBuf[38] != FOOTER_MAGIC1) && (RxBuf[38] != FOOTER_MAGIC3)) {
        move_header_in_buffer(1);
        return false;
    }
    if ((RxBuf[39] != FOOTER_MAGIC2) && (RxBuf[39] != FOOTER_MAGIC4)) {
        move_header_in_buffer(1);
        return false;
    }

    // calculate checksum
    uint8_t cs1 = 0;
    uint8_t cs2 = 0;
    for (int i = 0; i < 36; i++)
    {
        cs1 += RxBuf[i];
        cs2 += cs1;
    }

    // check that it matches
    if ((cs1 == RxBuf[36]) && (cs2 == RxBuf[37]))
    {
        // check which message we have received
        if (RxBuf[38] == FOOTER_MAGIC1) // this is the normal generator message
        {
            last_reading.msgType = 1;
            // process the data
        // define some temporary variables for unpacking the data
            uint32_t tempUint32 = 0;
            int32_t tempInt32 = 0;
            uint16_t tempUint16 = 0;
            int16_t tempInt16 = 0;

            // get the status flags
            tempUint16 = ((RxBuf[3] << 8) | RxBuf[2]);
            last_reading.pwm_avg = tempUint16;
            last_reading.currentPwmInputState = (RxBuf[4] & 0x0F); // Least significant 4 bits
            last_reading.detectedPwmState = ((RxBuf[4] >> 4) & 0x0F); // Most significant 4 bits
            last_reading.operateMode = (RxBuf[5] & 0x03); // least significant 2 bits
            last_reading.requestedOperateMode = ((RxBuf[5] >> 2) & 0x03); // next 2 bits
            last_reading.operateModeTransitionActive = ((RxBuf[5] >> 4) & 0x03); // next 2 bits
            last_reading.engineKillState = ((RxBuf[5] >> 6) & 0x03); // next 2 bits
            tempUint16 = ((RxBuf[7] << 8) | RxBuf[6]);
            last_reading.errorStatus = tempUint16;
            last_reading.tempError = (RxBuf[6] & 0x07); // only 3 LSBits
            last_reading.currentError = ((RxBuf[6]>>3) & 0x07); // next 3 bits
            last_reading.engineDied = ((RxBuf[8]) & 0x03); // only 2 LSBits
            last_reading.fuelPct = RxBuf[9];

            // old
            //tempUint16 = ((RxBuf[9] << 8) | RxBuf[8]);
            //last_reading.ctrlOutputFilt = tempUint16;


            tempUint32 = (RxBuf[23] << 24) | (RxBuf[22] << 16) | (RxBuf[21] << 8) | (RxBuf[20]);
            last_reading.runtime = tempUint32;
            tempInt32 = (RxBuf[27] << 24) | (RxBuf[26] << 16) | (RxBuf[25] << 8) | (RxBuf[24]);
            last_reading.seconds_until_maintenance = tempInt32;
            last_reading.errors = 0;
            // get RPM
            tempUint16 = (RxBuf[29] << 8) | RxBuf[28];
            last_reading.rpm = tempUint16;
            // get voltage
            tempUint16 = (RxBuf[17] << 8) | RxBuf[16];
            last_reading.output_voltage = ((float)(tempUint16)) * 0.01;
            // get current
            tempUint16 = (RxBuf[13] << 8) | RxBuf[12];
            last_reading.output_current = ((float)(tempUint16)) * 0.01;
            // get power generated
            tempUint16 = (RxBuf[15] << 8) | RxBuf[14];
            last_reading.pwrGenerated = ((float)(tempUint16)) * 0.1;
            // get batt current setpoint
            tempUint16 = (RxBuf[19] << 8) | RxBuf[18];
            last_reading.batt_current_setpoint = ((float)(tempUint16)) * 0.01;
            // get batt current
            tempInt16 = (RxBuf[11] << 8) | RxBuf[10];
            last_reading.batt_current = ((float)(tempInt16)) * 0.01;
            // get temperatures
            tempInt16 = (RxBuf[31] << 8) | RxBuf[30];
            last_reading.rectTemp = tempInt16 * 0.01;
            tempInt16 = (RxBuf[33] << 8) | RxBuf[32];
            last_reading.genTemp = tempInt16 * 0.01;

            // get servo ctrl value
            tempUint16 = (RxBuf[35] << 8) | RxBuf[34];
            last_reading.servoCmd = tempUint16;

        } // if the generator status message is found
        else if (RxBuf[38] == FOOTER_MAGIC3) // EFI message        
        {
            last_reading.msgType = 2;
            uint16_t tempUint16 = 0;
            int16_t tempInt16 = 0;

            if (!last_reading.efiMsgReceived)
            {
                last_reading.efiMsgReceived = 1;
            }
            
            // PROCESS EFI DATA into variables
            // pulseWidth1
            tempUint16 = (RxBuf[3] << 8) | RxBuf[2];
            last_reading.pulseWidthms = tempUint16;

            // rpm
            tempUint16 = (RxBuf[5] << 8) | RxBuf[4];
            last_reading.efiRPM = tempUint16;
            
            // afrtgt1
            last_reading.afrtgt = RxBuf[6];
            
            // baro
            tempInt16 = (RxBuf[8] << 8) | RxBuf[7];
            last_reading.baro = tempInt16;

            // mat
            tempInt16 = (RxBuf[10] << 8) | RxBuf[9];
            last_reading.mat = tempInt16;            
            
            // clt
            tempInt16 = (RxBuf[12] << 8) | RxBuf[11];
            last_reading.clt = tempInt16;            

            // tps
            tempInt16 = (RxBuf[14] << 8) | RxBuf[13];
            last_reading.tps = tempInt16;

            // bv
            tempInt16 = (RxBuf[16] << 8) | RxBuf[15];
            last_reading.bv = tempInt16;

            // afr1
            tempInt16 = (RxBuf[18] << 8) | RxBuf[17];
            last_reading.afr1 = tempInt16;

            // aircor
            tempInt16 = (RxBuf[20] << 8) | RxBuf[19];
            last_reading.aircor = tempInt16;

            // warmcor
            tempInt16 = (RxBuf[22] << 8) | RxBuf[21];
            last_reading.warmcor = tempInt16;            

            // accelEnrich
            tempInt16 = (RxBuf[24] << 8) | RxBuf[23];
            last_reading.accelEnrich = tempInt16;

            // tpsfuelcut
            tempInt16 = (RxBuf[26] << 8) | RxBuf[25];
            last_reading.tpsfuelcut = tempInt16;

            // baroCorrection
            tempInt16 = (RxBuf[28] << 8) | RxBuf[27];
            last_reading.baroCorrection = tempInt16;

            // gammaEnrich
            tempInt16 = (RxBuf[30] << 8) | RxBuf[29];
            last_reading.gammaEnrich = tempInt16;

            // ve1
            tempInt16 = (RxBuf[32] << 8) | RxBuf[31];
            last_reading.ve1 = tempInt16;

            // TPSdot
            tempInt16 = (RxBuf[34] << 8) | RxBuf[33];
            last_reading.TPSdot = tempInt16;

            // fuelload
            last_reading.fuelload = RxBuf[35];

        }

        // reset body length regardless of message type
        body_length = 0;
        return true;
    } // if the checksum is correct
    else // checksum not correct
    {
        // reject data (possibly in the future increment a counter?)
        body_length = 0;
        return false;
    }

} // end of get_reading

// find a Generator message in the buffer, starting at
// initial_offset.  If found, that message (or partial message) will
// be moved to the start of the buffer.
void Copter::move_header_in_buffer(uint8_t initial_offset)
{
    uint8_t header_offset;
    for (header_offset=initial_offset; header_offset<body_length; header_offset++) {
        if (RxBuf[header_offset] == HEADER_MAGIC1) {
            break;
        }
    }
    if (header_offset != 0) {
        // header was found, but not at index 0; move it back to start of array
        memmove(RxBuf, &RxBuf[header_offset], body_length - header_offset);
        body_length -= header_offset;
    }
} // end of move_header_in_buffer

uint8_t Copter::get_fuel_pct(void)
{
    return last_reading.fuelPct;
}

uint8_t Copter::get_gen_detected(void)
{
    return(last_reading.generatorDetected);
}
