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

#define TEMP_ERROR_SEND_INTERVAL 41 // cycles at 3.3 Hz (0.303 seconds)
#define CURRENT_ERROR_SEND_INTERVAL 32 // cycles at 3.3 Hz (0.303) seconds
#define FUEL_WARNING_SEND_INTERVAL 42 // cycles at 3.3 Hz (0.303) seconds
#define GENERATOR_TIMEOUT_SEND_INTERVAL 43 // cycles at 3.3 Hz

#define GENERATOR_TIMEOUT_ERROR_CNT 33 // cycles at 3.3 Hz (equates to about 10 seconds)

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

    //(void)get_reading();
    if (get_reading())
    {
        // temporary
        //gcs().send_text(MAV_SEVERITY_INFO, "DBGFUEL: %u%%",last_reading.fuelPct);

        // update the flag to indicate that we have detected the generator in the system
        if (!last_reading.generatorDetected)
        {
            last_reading.generatorDetected = 1;
        }


        // commented out on Oct 30, 2022 because fuel
        // percentage will now be transmitted as percentage gauge
        // on the battery monitor
        /*
        if(!fuelPctInitialized)
        {
            // wait for a while (e.g. to allow for connection to establish)
            // before transmitting the first initialized signal
            if (fuelSendCnt < 50)
            {
                fuelSendCnt++;
            }
            else
            {
                // send fuel percentage warnings every 10 percentage change
                startingFuelPct = last_reading.fuelPct;
                // broadcast fuel percent to ground station
                gcs().send_text(MAV_SEVERITY_INFO, "Initial Fuel: %u%%",last_reading.fuelPct);
                fuelPctInitialized = 1;
            }
        } // if (!fuelPctInitialized)
        else
        {
            if (startingFuelPct >= last_reading.fuelPct)
            {
                // if we have changed by more than 10, and we are at an even division of 10,
                // broadcast the current status
                if (((startingFuelPct - last_reading.fuelPct) >= 10) && ((last_reading.fuelPct % 10)==0))
                {
                    gcs().send_text(MAV_SEVERITY_INFO, "Remaining Fuel: %u%%",last_reading.fuelPct);
                    // reset starting fuel pct here?
                    startingFuelPct = last_reading.fuelPct;
                }
            }
            else
            {
                // the current fuel reading has exceeded the previous one (i.e. we have gained fuel)
                if ((last_reading.fuelPct - startingFuelPct) > 10)
                {
                    // reset the fuel system
                    fuelPctInitialized = 0;
                    fuelSendCnt = 0;
                }
            }
        } // else - from if (!fuelPctInitialized)
        */
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
                    // TODO - in the future, add failsafe actions and configurable ways of
                    // managing this, possibly by modifying parameters and using the previous
                    // GEN parameters (e.g. g.gen_fuel_pct) to be g.gen_fs_action.

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
            // trigger failsafe

        }
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

//
//  // try UART stuff
//
//  uint32_t nbytes = uart->read(RxBuf, 36);
//  if(nbytes>0)
//  {
//      gcs().send_text(MAV_SEVERITY_INFO, "Read UART bytes: %d",(uint8_t)nbytes);
//  }
//
//  if (nbytes >= 36)
//  {
//      // we have read at least a full packet
//      //gcs().send_text(MAV_SEVERITY_INFO, "Read UART bytes: %d",tempBuf[1]);
//
//      // transmit back
//      uart->printf("Byte %d\n",RxBuf[0]);
//      //uart->write(tempBuf, 4);
//  }


    last_reading.mode = currentGenMode;

    // get voltage from battery monitor
    //last_reading.output_voltage = battery.voltage();
    // TODO - remove this when rectifier PCB revision is calculating battery current directly

    //  float tempCur;
//    if(battery.current_amps(tempCur))
//  {
//      last_reading.batt_current = tempCur - last_reading.output_current;
//  }
//    else
//    {
//      last_reading.batt_current = 0;
//    }

//    // *************************************************************************************************
//    // *********** ENERGY INTEGRAL CALCULATION *********************************************************
//    // *************************************************************************************************
//
//  float dt;
//
//  if (!timeCaptured)
//  {
//      timeCaptured = 1;
//
//      dt = 0.0;
//      lastMs = AP_HAL::millis();
//  }
//  else
//  {
//      // we got the value once already
//      dt = ((float)(AP_HAL::millis() - lastMs)) * 0.001; // convert to seconds
//      // update previous time
//      lastMs = AP_HAL::millis();
//  }
//
//  if (last_reading.pwrIntegral > GEN_ENERGY_MAX_KJ)
//  {
//      last_reading.pwrIntegral = GEN_ENERGY_MAX_KJ;
//  }
//  else
//  {
//      last_reading.pwrIntegral += last_reading.pwrGenerated * dt * 0.001; // converting to kJ
//  }
//
//    // *************************************************************************************************
//    // *********** END OF ENERGY INTEGRAL CALCULATION **************************************************
//    // *************************************************************************************************
//
//
//  fuelPctLocal = (float)(startingFuelPct) - last_reading.pwrIntegral / (GEN_ENERGY_THRESH_KJ * energyScaleFact) * 100.0;
//
//  fuelPctLocal /= 100.0; // to keep within 0 and 1 bounds that is expected
//
//  if (fuelPctLocal > 100.0)
//  {
//      fuelPctLocal = 100.0;
//  }
//
//  if (fuelPctLocal < 0.0)
//  {
//      fuelPctLocal = 0.0;
//  }


//  static uint8_t counterSend = 0;
//  static uint8_t counter1 = 25;
//  counter1++;
//  if (counter1 > 100) {
//      counter1 = 0;
//      //temp
//      uint8_t fuelPctAdj;
//      fuelPctAdj = (uint8_t)(g.gen_fuel_pct);
//      if (fuelPctAdj != startingFuelPct)
//      {
//          // re-adjust and reset power integral
//          startingFuelPct = fuelPctAdj;
//          last_reading.pwrIntegral = 0;
//      }
//
//      float tempScaleFact;
//      tempScaleFact = (float)(g.gen_f_scale);
//      float diffScaleFac;
//      diffScaleFac = tempScaleFact - energyScaleFact;
//      if(diffScaleFac < 0.0)
//      {
//          diffScaleFac = -diffScaleFac;
//      }
//
//      if(diffScaleFac > 0.0001)
//      {
//          energyScaleFact = tempScaleFact;
//      }
//
//      // display output to console
//
//      if(counterSend < 30)
//      {
//          counterSend++;
//      }
//      else
//      {
//          counterSend = 0;
//          gcs().send_text(MAV_SEVERITY_INFO, "GEN: %.1f A, %.2f kW, %.1f %% ",last_reading.output_current,last_reading.pwrGenerated*0.001,fuelPctLocal*100);
//      }
//      //gcs().send_text(MAV_SEVERITY_INFO, "PWM: %d",genRadioCmd);
//  }

    static uint8_t counter2;
    counter2++;
    if (counter2 > 10)
    {

        // temp debug stuff
        //gcs().send_text(MAV_SEVERITY_INFO, "Engine Died: %d",last_reading.engineDied);


        counter2 = 0;
        // log //    log runtime, current, power, mode

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
            last_reading.output_voltage,
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

//      AP::logger().Write(
//          "GEN",
//          "TimeUS,runtime,current,power,mode",
//          //"ssAW-", // units
//          //"F----", // scaling
//          "QQffB",
//          AP_HAL::micros64(),
//          last_reading.runtime,
//          last_reading.output_current,
//          last_reading.pwrGenerated,
//          last_reading.mode
//          );

    } // counter 2
} // end of 50 hz loop
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

    static uint8_t counter2;
    counter2++;
    if (counter2 > 1)
    {

        counter2 = 0;
        // log //    log runtime, current, power, mode, etc.

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
            last_reading.output_voltage,
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
    if (RxBuf[38] != FOOTER_MAGIC1) {
        move_header_in_buffer(1);
        return false;
    }
    if (RxBuf[39] != FOOTER_MAGIC2) {
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

        //last_reading_ms = AP_HAL::millis();
        body_length = 0;

        return true;
    }
    else
    {
        // reject data (possibly in the future increment a counter?)
        body_length = 0;
        return false;
    }


    // check the version:
//    const uint16_t version = be16toh(u.packet.version);
//    const uint8_t major = version / 100;
//    const uint8_t minor = (version % 100) / 10;
//    const uint8_t point = version % 10;
//    if (!protocol_information_anounced) {
//        gcs().send_text(MAV_SEVERITY_INFO, "RichenPower: protocol %u.%u.%u", major, minor, point);
//        protocol_information_anounced = true;
//    }

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
