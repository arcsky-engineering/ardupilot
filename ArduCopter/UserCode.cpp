#include <AP_SerialManager/AP_SerialManager.h>
#include "Copter.h"

//TODO - calculate current difference between the generator current and the system current


// used for analog stuff only
//extern const AP_HAL::HAL& hal;

// adding this to access voltage and current from battery monitoring
const AP_BattMonitor &battery = AP::battery();

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
    float 		pwrIntegral;
    float		pwrGenerated;
    float		batt_current;
    float 		batt_current_setpoint;
    int16_t		rectTemp;
    int16_t		genTemp;
    uint16_t	servoCmd;
};

// declare some variables to use
struct Reading last_reading;

uint8_t startingFuelPct;
float lastCurrent;
float fuelPctLocal;
uint32_t last_reading_ms;
uint8_t timeCaptured;
uint32_t lastMs;

uint32_t runTimeMsLast;
uint8_t runTimeActive;
float runTimeDt;
float runTimeSec;

float energyScaleFact;

uint16_t genRadioCmd;
//uint16_t lastGenRadioCmd;
uint16_t genCmdOut;

uint8_t killState = 0;

uint64_t status;

GenMode currentGenMode = IDLE;

AP_HAL::UARTDriver *uart;

uint8_t RxBuf[38] = {0};
// number of bytes currently in the buffer
uint8_t body_length;

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

	// initialize the serial manager, according to how it's done in RichenPower
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Generator, 0);
    if (uart != nullptr) {
        //const uint32_t baud = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Generator, 0);
        //uart->begin(baud, 256, 256);
        // try 57600 directly
        uart->begin(57600,256,256);
    }


    startingFuelPct = (uint8_t)(g.gen_fuel_pct);

    energyScaleFact = g.gen_f_scale;

    status = 0;

    // set output to be "OFF" initially for the generator
    genCmdOut = 1000;
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
		killState = 1;
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

	(void)get_reading();
//
//	// try UART stuff
//
//	uint32_t nbytes = uart->read(RxBuf, 36);
//	if(nbytes>0)
//	{
//		gcs().send_text(MAV_SEVERITY_INFO, "Read UART bytes: %d",(uint8_t)nbytes);
//	}
//
//	if (nbytes >= 36)
//	{
//		// we have read at least a full packet
//		//gcs().send_text(MAV_SEVERITY_INFO, "Read UART bytes: %d",tempBuf[1]);
//
//		// transmit back
//		uart->printf("Byte %d\n",RxBuf[0]);
//		//uart->write(tempBuf, 4);
//	}

	last_reading.mode = currentGenMode;

    // get voltage from battery monitor
    //last_reading.output_voltage = battery.voltage();
	float tempCur;
    if(battery.current_amps(tempCur))
	{
    	last_reading.batt_current = tempCur - last_reading.output_current;
	}
    else
    {
    	last_reading.batt_current = 0;
    }

//    // *************************************************************************************************
//    // *********** ENERGY INTEGRAL CALCULATION *********************************************************
//    // *************************************************************************************************
//
//	float dt;
//
//	if (!timeCaptured)
//	{
//		timeCaptured = 1;
//
//		dt = 0.0;
//		lastMs = AP_HAL::millis();
//	}
//	else
//	{
//		// we got the value once already
//		dt = ((float)(AP_HAL::millis() - lastMs)) * 0.001; // convert to seconds
//		// update previous time
//		lastMs = AP_HAL::millis();
//	}
//
//	if (last_reading.pwrIntegral > GEN_ENERGY_MAX_KJ)
//	{
//		last_reading.pwrIntegral = GEN_ENERGY_MAX_KJ;
//	}
//	else
//	{
//		last_reading.pwrIntegral += last_reading.pwrGenerated * dt * 0.001; // converting to kJ
//	}
//
//    // *************************************************************************************************
//    // *********** END OF ENERGY INTEGRAL CALCULATION **************************************************
//    // *************************************************************************************************
//
//
//	fuelPctLocal = (float)(startingFuelPct) - last_reading.pwrIntegral / (GEN_ENERGY_THRESH_KJ * energyScaleFact) * 100.0;
//
//	fuelPctLocal /= 100.0; // to keep within 0 and 1 bounds that is expected
//
//	if (fuelPctLocal > 100.0)
//	{
//		fuelPctLocal = 100.0;
//	}
//
//	if (fuelPctLocal < 0.0)
//	{
//		fuelPctLocal = 0.0;
//	}


//	static uint8_t counterSend = 0;
//	static uint8_t counter1 = 25;
//	counter1++;
//	if (counter1 > 100) {
//	    counter1 = 0;
//		//temp
//	    uint8_t fuelPctAdj;
//	    fuelPctAdj = (uint8_t)(g.gen_fuel_pct);
//	    if (fuelPctAdj != startingFuelPct)
//	    {
//	    	// re-adjust and reset power integral
//	    	startingFuelPct = fuelPctAdj;
//	    	last_reading.pwrIntegral = 0;
//	    }
//
//	    float tempScaleFact;
//	    tempScaleFact = (float)(g.gen_f_scale);
//	    float diffScaleFac;
//	    diffScaleFac = tempScaleFact - energyScaleFact;
//	    if(diffScaleFac < 0.0)
//	    {
//	    	diffScaleFac = -diffScaleFac;
//	    }
//
//	    if(diffScaleFac > 0.0001)
//	    {
//	    	energyScaleFact = tempScaleFact;
//	    }
//
//	    // display output to console
//
//	    if(counterSend < 30)
//	    {
//	    	counterSend++;
//	    }
//	    else
//	    {
//	    	counterSend = 0;
//	    	gcs().send_text(MAV_SEVERITY_INFO, "GEN: %.1f A, %.2f kW, %.1f %% ",last_reading.output_current,last_reading.pwrGenerated*0.001,fuelPctLocal*100);
//	    }
//	    //gcs().send_text(MAV_SEVERITY_INFO, "PWM: %d",genRadioCmd);
//	}

	static uint8_t counter2;
	counter2++;
	if (counter2 > 10)
	{
		counter2 = 0;
		// log //	 log runtime, current, power, mode

	    AP::logger().Write(
	        "GEN",
	        "TimeUS,runTime,maintTime,errors,rpm,ovolt,ocurr,bcurr,trect,tgen,mode,throt",
	        "s---qvAAOO--",
	        "F-----------",
	        "QIIHHfffhhBH",
	        AP_HAL::micros64(),
	        last_reading.runtime,
	        last_reading.seconds_until_maintenance,
	        last_reading.errors,
	        last_reading.rpm,
	        last_reading.output_voltage,
	        last_reading.output_current,
			last_reading.batt_current,
			last_reading.rectTemp,
			last_reading.genTemp,
	        last_reading.mode,
			last_reading.servoCmd
	        );

//	    AP::logger().Write(
//	        "GEN",
//	        "TimeUS,runtime,current,power,mode",
//			//"ssAW-", // units
//			//"F----", // scaling
//	        "QQffB",
//	        AP_HAL::micros64(),
//	        last_reading.runtime,
//	        last_reading.output_current,
//			last_reading.pwrGenerated,
//	        last_reading.mode
//	        );

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
//    if (last_reading_ms == 0) {
//        // nothing to report
//        return;
//    }

    status = 0;

//    if(last_reading.mode == GenMode::OFF)
//    {
//    	status |= MAV_GENERATOR_STATUS_FLAG_OFF;
//    }
//    else if (last_reading.mode == GenMode::IDLE)
//    {
//    	status |= MAV_GENERATOR_STATUS_FLAG_IDLE;
//    }
//    else if (last_reading.mode == GenMode::RUN)
//    {
//        status |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
//        status |= MAV_GENERATOR_STATUS_FLAG_CHARGING;
//    }

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



//    if (last_reading.rpm == 0) {
//        status |= MAV_GENERATOR_STATUS_FLAG_OFF;
//    } else {
//        switch (last_reading.mode) {
//        case Mode::OFF:
//            status |= MAV_GENERATOR_STATUS_FLAG_OFF;
//            break;
//        case Mode::IDLE:
//            if (pilot_desired_runstate == RunState::RUN) {
//                status |= MAV_GENERATOR_STATUS_FLAG_WARMING_UP;
//            } else {
//                status |= MAV_GENERATOR_STATUS_FLAG_IDLE;
//            }
//            break;
//        case Mode::RUN:
//            status |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
//            break;
//        case Mode::CHARGE:
//            status |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
//            status |= MAV_GENERATOR_STATUS_FLAG_CHARGING;
//            break;
//        case Mode::BALANCE:
//            status |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
//            status |= MAV_GENERATOR_STATUS_FLAG_CHARGING;
//            break;
//        }
//    }
//
//    if (last_reading.errors & (uint8_t)Errors::Overload) {
//        status |= MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT;
//    }
//    if (last_reading.errors & (uint8_t)Errors::LowVoltageOutput) {
//        status |= MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER;
//    }
//
//    if (last_reading.errors & (uint8_t)Errors::MaintenanceRequired) {
//        status |= MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED;
//    }
//    if (last_reading.errors & (uint8_t)Errors::StartDisabled) {
//        status |= MAV_GENERATOR_STATUS_FLAG_START_INHIBITED;
//    }
//    if (last_reading.errors & (uint8_t)Errors::LowBatteryVoltage) {
//        status |= MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT;
//    }


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
    if (RxBuf[36] != FOOTER_MAGIC1) {
        move_header_in_buffer(1);
        return false;
    }
    if (RxBuf[37] != FOOTER_MAGIC2) {
        move_header_in_buffer(1);
        return false;
    }

    // calculate checksum....
//    uint16_t checksum = 0;
//    const uint8_t *checksum_buffer = &u.parse_buffer[2];
//    for (uint8_t i=0; i<5; i++) {
//        checksum += be16toh_ptr(&checksum_buffer[2*i]);
//    }

//    if (checksum != be16toh(u.packet.checksum)) {
//        move_header_in_buffer(1);
//        return false;
//    }

    // check the version:
//    const uint16_t version = be16toh(u.packet.version);
//    const uint8_t major = version / 100;
//    const uint8_t minor = (version % 100) / 10;
//    const uint8_t point = version % 10;
//    if (!protocol_information_anounced) {
//        gcs().send_text(MAV_SEVERITY_INFO, "RichenPower: protocol %u.%u.%u", major, minor, point);
//        protocol_information_anounced = true;
//    }

    // define some temporary variables for unpacking the data
    uint32_t tempUint32 = 0;
    int32_t tempInt32 = 0;
    uint16_t tempUint16 = 0;
    int16_t tempInt16 = 0;

    tempUint32 = (RxBuf[23] << 24) | (RxBuf[22] << 16) | (RxBuf[21] << 8) | (RxBuf[20]);
    last_reading.runtime =  tempUint32;
    tempInt32 = (RxBuf[27] << 24) | (RxBuf[26] << 16) | (RxBuf[25] << 8) | (RxBuf[24]);
    last_reading.seconds_until_maintenance = tempInt32;
    last_reading.errors = 0;
    // get RPM
    tempUint16 = (RxBuf[29]<<8) | RxBuf[28];
    last_reading.rpm = tempUint16;
    // get voltage
    tempUint16 = (RxBuf[17]<<8) | RxBuf[16];
    last_reading.output_voltage = ((float)(tempUint16)) * 0.01;
    // get current
    tempUint16 = (RxBuf[13]<<8) | RxBuf[12];
    last_reading.output_current = ((float)(tempUint16)) * 0.01;
    // get power generated
    tempUint16 = (RxBuf[15]<<8) | RxBuf[14];
    last_reading.pwrGenerated = ((float)(tempUint16)) * 0.1;
    // get batt current setpoint
    tempUint16 = (RxBuf[19]<<8) | RxBuf[18];
    last_reading.batt_current_setpoint = ((float)(tempUint16)) * 0.01;
    // get batt current
    //tempUint16 = (RxBuf[19]<<8) | RxBuf[18];
    //last_reading.batt_current_setpoint = ((float)(tempUint16)) * 0.01;
    // get temperatures
    tempInt16 = (RxBuf[31]<<8) | RxBuf[30];
    last_reading.rectTemp = tempInt16 * 0.01;
    tempInt16 = (RxBuf[33]<<8) | RxBuf[32];
    last_reading.genTemp = tempInt16 * 0.01;

    // get servo ctrl value
    tempUint16 = (RxBuf[35]<<8) | RxBuf[34];
    last_reading.servoCmd = tempUint16;

    //last_reading_ms = AP_HAL::millis();

    body_length = 0;

    // update the time we started idling at:
//    if (last_reading.mode == Mode::IDLE) {
//        if (idle_state_start_ms == 0) {
//            idle_state_start_ms = last_reading_ms;
//        }
//    } else {
//        idle_state_start_ms = 0;
//    }

    return true;
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
