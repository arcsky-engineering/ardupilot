/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *
 * original code by:
 *  BlueMark Innovations BV, Roel Schiphorst
 *  Contributors: Tom Pittenger, Josh Henderson, Andrew Tridgell
 *  Parts of this code are based on/copied from the Open Drone ID project https://github.com/opendroneid/opendroneid-core-c
 *
 * The code has been tested with the BlueMark DroneBeacon MAVLink transponder running this command in the ArduPlane folder:
 * sim_vehicle.py --console --map -A --serial1=uart:/dev/ttyUSB1:9600
 * (and a DroneBeacon MAVLink transponder connected to ttyUSB1)
 *
 * See https://github.com/ArduPilot/ArduRemoteID for an open implementation of a transmitter module on serial
 * and DroneCAN
 */

#include "AP_OpenDroneID.h"

#if AP_OPENDRONEID_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Parachute/AP_Parachute.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <stdio.h>
#include <string.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_OpenDroneID::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable ODID subsystem
    // @Description: Enable ODID subsystem
    // @Values: 0:Disabled,1:Enabled
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_OpenDroneID, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: MAVPORT
    // @DisplayName: MAVLink serial port
    // @Description: Serial port number to send OpenDroneID MAVLink messages to. Can be -1 if using DroneCAN.
    // @Values: -1:Disabled,0:Serial0,1:Serial1,2:Serial2,3:Serial3,4:Serial4,5:Serial5,6:Serial6
    AP_GROUPINFO("MAVPORT", 2, AP_OpenDroneID, _mav_port, -1),

    // @Param: CANDRIVER
    // @DisplayName: DroneCAN driver number
    // @Description: DroneCAN driver index, 0 to disable DroneCAN
    // @Values: 0:Disabled,1:Driver1,2:Driver2
    AP_GROUPINFO("CANDRIVER", 3, AP_OpenDroneID, _can_driver, 0),
    
    // @Param: OPTIONS
    // @DisplayName: OpenDroneID options
    // @Description: Options for OpenDroneID subsystem
    // @Bitmask: 0:EnforceArming, 1:AllowNonGPSPosition, 2:LockUASIDOnFirstBasicIDRx
    AP_GROUPINFO("OPTIONS", 4, AP_OpenDroneID, _options, 0),

    // @Param: BARO_ACC
    // @DisplayName: Barometer vertical accuraacy
    // @Description: Barometer Vertical Accuracy when installed in the vehicle. Note this is dependent upon installation conditions and thus disabled by default
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("BARO_ACC", 5, AP_OpenDroneID, _baro_accuracy, -1.0),

    AP_GROUPEND
};

#if defined(OPENDRONEID_UA_TYPE)
// ensure the type is within the allowed range
#if OPENDRONEID_UA_TYPE < 0 || OPENDRONEID_UA_TYPE > 15
#error "OPENDRONEID_UA_TYPE must be between 0 and 15"
#endif
#endif

// constructor
AP_OpenDroneID::AP_OpenDroneID()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("OpenDroneID must be singleton");
    }
#endif
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_OpenDroneID::init()
{
    if (_enable == 0) {
        return;
    }

    load_UAS_ID_from_persistent_memory();
    _chan = mavlink_channel_t(gcs().get_channel_from_port_number(_mav_port));
    _init_time_ms = AP_HAL::millis();
    _initialised = true;
}

void AP_OpenDroneID::load_UAS_ID_from_persistent_memory()
{
    id_len = sizeof(id_str);
    size_t id_type_len = sizeof(id_type);
    size_t ua_type_len = sizeof(ua_type);
    if (hal.util->get_persistent_param_by_name("DID_UAS_ID", id_str, id_len) &&
        hal.util->get_persistent_param_by_name("DID_UAS_ID_TYPE", id_type, id_type_len) &&
        hal.util->get_persistent_param_by_name("DID_UA_TYPE", ua_type, ua_type_len)) {
        if (id_len && id_type_len && ua_type_len) {
            _options.set_and_save(_options.get() & ~LockUASIDOnFirstBasicIDRx);
            _options.notify();
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OpenDroneID: Locked UAS_ID: %s", id_str);
        }
    } else {
        id_len = 0;
    }
}

void AP_OpenDroneID::set_basic_id() {
    if (pkt_basic_id.id_type != MAV_ODID_ID_TYPE_NONE) {
        return;
    }
    if (id_len > 0) {
        // prepare basic id pkt
        uint8_t val = gcs().sysid_this_mav();
        pkt_basic_id.target_system = val;
        pkt_basic_id.target_component = MAV_COMP_ID_ODID_TXRX_1;
        pkt_basic_id.id_type = atoi(id_type);
        pkt_basic_id.ua_type = atoi(ua_type);
        char buffer[21];
        snprintf(buffer, sizeof(buffer), "%s", id_str);
        memcpy(pkt_basic_id.uas_id, buffer, sizeof(pkt_basic_id.uas_id));
    }
}

// Generic placeholder SelfID/OperatorID values originated by ArduPilot. These
// are intended to be overridden by the GCS (see handle_msg). Build-overridable.
#ifndef OPENDRONEID_SELF_ID_DEFAULT
#define OPENDRONEID_SELF_ID_DEFAULT "X55"
#endif
#ifndef OPENDRONEID_OPERATOR_ID_DEFAULT
#define OPENDRONEID_OPERATOR_ID_DEFAULT "001"
#endif

// Some Remote ID modules (e.g. Cube ID) refuse to operate until they have
// received both a SelfID and an OperatorID. ArduPilot originates generic
// placeholders so such modules work out of the box. A GCS can override these at
// any time by sending OPEN_DRONE_ID_SELF_ID / OPEN_DRONE_ID_OPERATOR_ID, which
// handle_msg() decodes straight into these same structs - so once a real value
// arrives the (non-empty) field is left untouched here.
void AP_OpenDroneID::set_self_id_operator_id_defaults()
{
    WITH_SEMAPHORE(_sem);
    const uint8_t sysid = gcs().sysid_this_mav();

    if (pkt_self_id.description[0] == '\0') {
        pkt_self_id.target_system = sysid;
        pkt_self_id.target_component = MAV_COMP_ID_ODID_TXRX_1;
        pkt_self_id.description_type = MAV_ODID_DESC_TYPE_TEXT;
        strncpy(pkt_self_id.description, OPENDRONEID_SELF_ID_DEFAULT, sizeof(pkt_self_id.description) - 1);
    }

    if (pkt_operator_id.operator_id[0] == '\0') {
        pkt_operator_id.target_system = sysid;
        pkt_operator_id.target_component = MAV_COMP_ID_ODID_TXRX_1;
        pkt_operator_id.operator_id_type = MAV_ODID_OPERATOR_ID_TYPE_CAA;
        strncpy(pkt_operator_id.operator_id, OPENDRONEID_OPERATOR_ID_DEFAULT, sizeof(pkt_operator_id.operator_id) - 1);
    }
}

void AP_OpenDroneID::get_persistent_params(ExpandingString &str) const
{
    if (id_len > 0) {
        str.printf("DID_UAS_ID=%s\nDID_UAS_ID_TYPE=%s\nDID_UA_TYPE=%s\n", id_str, id_type, ua_type);
    }
}

// Translate Remote ID module error strings into user-friendly messages.
// The DB201 sends text tokens like "LOC ID SYS OP_LOC". The DB300 (and other
// ArduRemoteID-style modules) instead report a numeric bitmask in the error
// field, e.g. "err 8". Both are cryptic to the operator, so decode either form.
static void translate_arm_status_error(const char* error, char* failmsg, uint8_t failmsg_len)
{
    // DB300 numeric bitmask path: pull the first integer out of the error
    // string (robust to "err 8", "8", "ERR 8", etc.). See the db300 manual
    // v1.3 section 1.8 "Status codes":
    //   1  = transmission disabled        2  = no BasicID received/configured
    //   4  = no OperatorID received       8  = no drone Location msg / no GPS fix
    //   16 = no System msg / no GPS info
    // Codes combine (e.g. 10 = 2 | 8). Surface configuration faults first as
    // they won't clear on their own, then the GPS/timing-dependent ones.
    long code = -1;
    for (const char *p = error; *p != '\0'; p++) {
        if (*p >= '0' && *p <= '9') {
            code = 0;
            while (*p >= '0' && *p <= '9') {
                code = (code * 10) + (*p - '0');
                p++;
            }
            break;
        }
    }
    if (code > 0) {
        if (code & 2) {
            strncpy(failmsg, "Basic I D not Configured", failmsg_len);
        } else if (code & 4) {
            strncpy(failmsg, "No Operator Location", failmsg_len);
        } else if (code & 8) {
            strncpy(failmsg, "No Drone Location or GPS fix", failmsg_len);
        } else if (code & 16) {
            strncpy(failmsg, "No System Message or GPS info", failmsg_len);
        } else if (code & 1) {
            strncpy(failmsg, "Remote I D Tx Disabled", failmsg_len);
        } else {
            strncpy(failmsg, "Remote ID module not ready", failmsg_len);
        }
        failmsg[failmsg_len - 1] = '\0';
        return;
    }

    // DB201 text-token path
    // Check for specific error codes from DB201 and translate them
    if (strstr(error, "OP_LOC") != nullptr) {
        strncpy(failmsg, "No Operator Location", failmsg_len);
    } else if (strstr(error, "LOC") != nullptr) {
        strncpy(failmsg, "No Drone Location", failmsg_len);
    } else if (strstr(error, "SYS") != nullptr) {
        strncpy(failmsg, "Remote I D System Message", failmsg_len);
    } else if (strstr(error, "ID") != nullptr) {
        strncpy(failmsg, "Basic I D not Configured", failmsg_len);
    } else if (strlen(error) > 0) {
        // Unknown error, pass through as-is
        strncpy(failmsg, error, failmsg_len);
    } else {
        strncpy(failmsg, "Remote ID module not ready", failmsg_len);
    }
    failmsg[failmsg_len - 1] = '\0';
}

// Perform the pre-arm checks and prevent arming if they are not satisifed
// Except in the case of an in-flight reboot
bool AP_OpenDroneID::pre_arm_check(char* failmsg, uint8_t failmsg_len)
{
    WITH_SEMAPHORE(_sem);

    if (!option_enabled(Options::EnforceArming)) {
        return true;
    }

    if (_enable == 0) {
        strncpy(failmsg, "DID_ENABLE must be 1", failmsg_len);
        return false;
    }

    // Check if we're receiving ARM_STATUS from the Remote ID module (DB201)
    const uint32_t max_age_ms = 3000;
    const uint32_t now_ms = AP_HAL::millis();

    if (last_arm_status_ms == 0 || now_ms - last_arm_status_ms > max_age_ms) {
        strncpy(failmsg, "no response from Remote ID module", failmsg_len);
        return false;
    }

    // Trust the DB201's arm status - it checks everything internally
    // (BasicID, Location, System, Operator Location)
    if (arm_status.status != MAV_ODID_ARM_STATUS_GOOD_TO_ARM) {
        translate_arm_status_error(arm_status.error, failmsg, failmsg_len);
        return false;
    }

    return true;
}

void AP_OpenDroneID::update()
{
    if (_enable == 0) {
        return;
    }

    set_basic_id();
    set_self_id_operator_id_defaults();

    // Auto-clear pilot emergency if the GCS stops sending SELF_ID (e.g. QGC closed
    // mid-emergency). Without this we'd broadcast EMERGENCY indefinitely until reboot.
    if (_pilot_emergency && (AP_HAL::millis() - _last_self_id_ms) > PILOT_EMERGENCY_TIMEOUT_MS) {
        _pilot_emergency = false;
        gcs().send_text(MAV_SEVERITY_INFO, "ODID: pilot emergency auto-cleared (GCS link stale)");
    }

    const bool armed = hal.util->get_soft_armed();
    if (armed && !_was_armed) {
        // use arm location as takeoff location
        AP::ahrs().get_location(_takeoff_location);
    }
    _was_armed = armed;

    send_dynamic_out();
    send_static_out();
}

// local payload space check which treats invalid channel as having space
// needed to populate the message structures for the DroneCAN backend
#define ODID_HAVE_PAYLOAD_SPACE(id) (_chan == MAV_CHAN_INVALID || HAVE_PAYLOAD_SPACE(_chan, id))

void AP_OpenDroneID::send_dynamic_out()
{

    // should we alternate the order of these in case there is not enough payload space
    // and the operator location isn't being sent out?

    const uint32_t now = AP_HAL::millis();
    if (now - _last_send_location_ms >= _mavlink_dynamic_period_ms &&
        ODID_HAVE_PAYLOAD_SPACE(OPEN_DRONE_ID_LOCATION)) {
        _last_send_location_ms = now;
        send_location_message();
    }

    // operator location needs to be sent at the same rate as location for FAA compliance
    if (now - _last_send_system_update_ms >= _mavlink_dynamic_period_ms &&
        ODID_HAVE_PAYLOAD_SPACE(OPEN_DRONE_ID_SYSTEM_UPDATE)) {
        _last_send_system_update_ms = now;
        send_system_update_message();
    }
}

void AP_OpenDroneID::send_static_out()
{
    const uint32_t now_ms = AP_HAL::millis();

    // Suppress warnings during boot grace period (30 seconds) to allow
    // GCS and transmitter time to establish communication
    const uint32_t boot_grace_period_ms = 30000;
    const bool in_grace_period = (now_ms - _init_time_ms) < boot_grace_period_ms;

    // we need to notify user if we lost the transmitter (but not during boot)
    if (!in_grace_period && now_ms - last_arm_status_ms > 5000) {
        if (now_ms - last_lost_tx_ms > 5000) {
            last_lost_tx_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Remote ID: lost connection");
        }
    } else if (last_lost_tx_ms != 0) {
        // we're OK again
        last_lost_tx_ms = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Remote ID: regained connection");
    }

    // Note: "lost operator location" warning removed - DB201 handles this check
    // and reports via ARM_STATUS. Pre-arm will fail with user-friendly message
    // "operator location not set in GCS" if operator location is missing.

    const uint32_t msg_spacing_ms = _mavlink_static_period_ms / 4;
    if (now_ms - last_msg_send_ms >= msg_spacing_ms) {
        // allow update of channel during setup, this makes it easy to debug with a GCS
        _chan = mavlink_channel_t(gcs().get_channel_from_port_number(_mav_port));
        bool sent_ok = false;
        switch (next_msg_to_send) {
        case NEXT_MSG_BASIC_ID:
            // Send BasicID over DroneCAN so DB201 can auto-save UAS ID
            send_basic_id_message();
            sent_ok = true;
            break;
        case NEXT_MSG_SYSTEM:
            if (ODID_HAVE_PAYLOAD_SPACE(OPEN_DRONE_ID_SYSTEM)) {
                send_system_message();
                sent_ok = true;
            }
            break;
        case NEXT_MSG_SELF_ID:
            // forwarded so modules that require a SelfID (e.g. Cube ID) are happy
            send_self_id_message();
            sent_ok = true;
            break;
        case NEXT_MSG_OPERATOR_ID:
            // forwarded so modules that require an OperatorID (e.g. Cube ID) are happy
            send_operator_id_message();
            sent_ok = true;
            break;
        case NEXT_MSG_ENUM_END:
            break;
        }
        if (sent_ok) {
            last_msg_send_ms = now_ms;
            next_msg_to_send = next_msg((uint8_t(next_msg_to_send) + 1) % uint8_t(NEXT_MSG_ENUM_END));
        }
    }
}

// The send_location_message
// all open_drone_id send functions use data stored in the open drone id class.
// This location send function is an exception. It uses live location data from the ArduPilot system.
void AP_OpenDroneID::send_location_message()
{
    auto &ahrs = AP::ahrs();
    const auto &barometer = AP::baro();
    const auto &gps = AP::gps();

    const AP_GPS::GPS_Status gps_status = gps.status();
    const bool got_bad_gps_fix = (gps_status < AP_GPS::GPS_Status::GPS_OK_FIX_3D);
    const bool armed = hal.util->get_soft_armed();

    Location current_location;
    if (!ahrs.get_location(current_location)) {
        return;
    }
    uint8_t uav_status = hal.util->get_soft_armed()? MAV_ODID_STATUS_AIRBORNE : MAV_ODID_STATUS_GROUND;
#if HAL_PARACHUTE_ENABLED
    // set emergency status if chute is released
    const auto *parachute = AP::parachute();
    if (parachute != nullptr && parachute->released()) {
        uav_status = MAV_ODID_STATUS_EMERGENCY;
    }
#endif
    if (AP::vehicle()->is_crashed()) {
        // if in crashed state also declare an emergency
        uav_status = MAV_ODID_STATUS_EMERGENCY;
    }

    // if we are armed with no GPS fix and we haven't specifically
    // allowed for non-GPS operation then declare an emergency
    if (got_bad_gps_fix && armed && !option_enabled(Options::AllowNonGPSPosition)) {
        uav_status = MAV_ODID_STATUS_EMERGENCY;
    }

    // if we are disarmed and falling at over 3m/s then declare an
    // emergency. This covers cases such as deliberate crash with
    // advanced failsafe and an unintended reboot or in-flight disarm
    if (!got_bad_gps_fix && !armed && gps.velocity().z > 3.0) {
        uav_status = MAV_ODID_STATUS_EMERGENCY;
    }

    // if we have watchdogged while armed then declare an emergency
    if (hal.util->was_watchdog_armed()) {
        uav_status = MAV_ODID_STATUS_EMERGENCY;
    }

    // pilot-declared emergency from the GCS (latched from incoming
    // OPEN_DRONE_ID_SELF_ID with description_type == MAV_ODID_DESC_TYPE_EMERGENCY).
    // The SelfID forward path to the RemoteID module is disabled in this build, so
    // we mirror the emergency state into Location.status which is forwarded and
    // broadcast.
    if (_pilot_emergency) {
        uav_status = MAV_ODID_STATUS_EMERGENCY;
    }

    float direction = ODID_INV_DIR;
    if (!got_bad_gps_fix) {
        direction = wrap_360(degrees(ahrs.groundspeed_vector().angle())); // heading (degrees)
    }

    const float speed_horizontal = create_speed_horizontal(ahrs.groundspeed());

    Vector3f velNED;
    UNUSED_RESULT(ahrs.get_velocity_NED(velNED));
    const float climb_rate = create_speed_vertical(-velNED.z); //make sure climb_rate is within Remote ID limit

    int32_t latitude = 0;
    int32_t longitude = 0;
    if (current_location.check_latlng()) { //set location if they are valid
        latitude = current_location.lat;
        longitude = current_location.lng;
    }

    // altitude referenced against 1013.2mb
    const float base_press_mbar = 1013.2;
    const float altitude_barometric = create_altitude(barometer.get_altitude_difference(base_press_mbar*100, barometer.get_pressure()));

    float altitude_geodetic = -1000;
    int32_t alt_amsl_cm;
    float undulation;
    if (current_location.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_amsl_cm)) {
        altitude_geodetic = alt_amsl_cm * 0.01;
    }
    if (gps.get_undulation(undulation)) {
        altitude_geodetic -= undulation;
    }

    // Compute the current height above the takeoff location
    float height_above_takeoff = 0;     // height above takeoff (meters)
    if (hal.util->get_soft_armed()) {
        int32_t curr_alt_asml_cm;
        int32_t takeoff_alt_asml_cm;
        if (current_location.get_alt_cm(Location::AltFrame::ABSOLUTE, curr_alt_asml_cm) &&
            _takeoff_location.get_alt_cm(Location::AltFrame::ABSOLUTE, takeoff_alt_asml_cm)) {
            height_above_takeoff = (curr_alt_asml_cm - takeoff_alt_asml_cm) * 0.01;
        }
    }

    // Accuracy

    // If we have GPS 3D lock we presume that the accuracies of the system will track the GPS's reported accuracy
    MAV_ODID_HOR_ACC horizontal_accuracy_mav = MAV_ODID_HOR_ACC_UNKNOWN;
    MAV_ODID_VER_ACC vertical_accuracy_mav = MAV_ODID_VER_ACC_UNKNOWN;
    MAV_ODID_SPEED_ACC speed_accuracy_mav = MAV_ODID_SPEED_ACC_UNKNOWN;
    MAV_ODID_TIME_ACC timestamp_accuracy_mav = MAV_ODID_TIME_ACC_UNKNOWN;

    float horizontal_accuracy;
    if (gps.horizontal_accuracy(horizontal_accuracy)) {
        horizontal_accuracy_mav = create_enum_horizontal_accuracy(horizontal_accuracy);
    }

    float vertical_accuracy;
    if (gps.vertical_accuracy(vertical_accuracy)) {
        vertical_accuracy_mav = create_enum_vertical_accuracy(vertical_accuracy);
    }

    float speed_accuracy;
    if (gps.speed_accuracy(speed_accuracy)) {
        speed_accuracy_mav = create_enum_speed_accuracy(speed_accuracy);
    }

    // if we have ever had GPS lock then we will have better than 1s
    // accuracy, as we use system timer to propogate time
    timestamp_accuracy_mav =  create_enum_timestamp_accuracy(1.0);

    // Barometer altitude accuraacy will be highly dependent on the airframe and installation of the barometer in use
    // thus ArduPilot cannot reasonably fill this in.
    // Instead allow a manufacturer to use a parameter to fill this in
    uint8_t barometer_accuracy = MAV_ODID_VER_ACC_UNKNOWN; //ahrs class does not provide accuracy readings
    if (!is_equal(_baro_accuracy.get(), -1.0f)) {
        barometer_accuracy = create_enum_vertical_accuracy(_baro_accuracy);
    }

    // Timestamp here is the number of seconds after into the current hour referenced to UTC time (up to one hour)

    // FIX we need to only set this if w have a GPS lock is 2D good enough for that?
    float timestamp = ODID_INV_TIMESTAMP;
    if (!got_bad_gps_fix) {
        uint32_t time_week_ms = gps.time_week_ms();
        timestamp = float(time_week_ms % (3600 * 1000)) * 0.001;
        timestamp = create_location_timestamp(timestamp);   //make sure timestamp is within Remote ID limit
    }


    {
        WITH_SEMAPHORE(_sem);
        // take semaphore so CAN gets a consistent packet
        pkt_location = mavlink_open_drone_id_location_t{
        latitude : latitude,
        longitude : longitude,
        altitude_barometric : altitude_barometric,
        altitude_geodetic : altitude_geodetic,
        height : height_above_takeoff,
        timestamp : timestamp,
        direction : uint16_t(direction * 100.0), // Heading (centi-degrees)
        speed_horizontal : uint16_t(speed_horizontal * 100.0), // Ground speed (cm/s)
        speed_vertical : int16_t(climb_rate * 100.0), // Climb rate (cm/s)
        target_system : 0,
        target_component : 0,
        id_or_mac : {},
        status : uint8_t(uav_status),
        height_reference : MAV_ODID_HEIGHT_REF_OVER_TAKEOFF,           // height reference enum: Above takeoff location or above ground
        horizontal_accuracy : uint8_t(horizontal_accuracy_mav),
        vertical_accuracy : uint8_t(vertical_accuracy_mav),
        barometer_accuracy : barometer_accuracy,
        speed_accuracy : uint8_t(speed_accuracy_mav),
        timestamp_accuracy : uint8_t(timestamp_accuracy_mav)
        };
        need_send_location = dronecan_send_all;
    }

    if (_chan != MAV_CHAN_INVALID) {
        mavlink_msg_open_drone_id_location_send_struct(_chan, &pkt_location);
    }
}

void AP_OpenDroneID::send_basic_id_message()
{
    // note that packet is filled in by the GCS
    need_send_basic_id |= dronecan_send_all;
    if (_chan != MAV_CHAN_INVALID) {
        mavlink_msg_open_drone_id_basic_id_send_struct(_chan, &pkt_basic_id);
    }
}

void AP_OpenDroneID::send_system_message()
{
    // note that packet is filled in by the GCS
    need_send_system |= dronecan_send_all;
    if (_chan != MAV_CHAN_INVALID) {
        mavlink_msg_open_drone_id_system_send_struct(_chan, &pkt_system);
    }
}

void AP_OpenDroneID::send_self_id_message()
{
    need_send_self_id |= dronecan_send_all;
    if (_chan != MAV_CHAN_INVALID) {
        mavlink_msg_open_drone_id_self_id_send_struct(_chan, &pkt_self_id);
    }
}

void AP_OpenDroneID::send_system_update_message()
{
    need_send_system |= dronecan_send_all;
    // note that packet is filled in by the GCS
    if (_chan != MAV_CHAN_INVALID) {
        const auto pkt_system_update = mavlink_open_drone_id_system_update_t {
        operator_latitude : pkt_system.operator_latitude,
        operator_longitude : pkt_system.operator_longitude,
        operator_altitude_geo : pkt_system.operator_altitude_geo,
        timestamp : pkt_system.timestamp,
        target_system : pkt_system.target_system,
        target_component : pkt_system.target_component,
        };
        mavlink_msg_open_drone_id_system_update_send_struct(_chan, &pkt_system_update);
    }
}

void AP_OpenDroneID::send_operator_id_message()
{
    need_send_operator_id |= dronecan_send_all;
    // note that packet is filled in by the GCS
    if (_chan != MAV_CHAN_INVALID) {
        mavlink_msg_open_drone_id_operator_id_send_struct(_chan, &pkt_operator_id);
    }
}

/*
* This converts a horizontal accuracy float value to the corresponding enum
*
* @param Accuracy The horizontal accuracy in meters
* @return Enum value representing the accuracy
*/
MAV_ODID_HOR_ACC AP_OpenDroneID::create_enum_horizontal_accuracy(float accuracy) const
{
    // Out of bounds return UNKNOWN flag
    if (accuracy < 0.0 || accuracy >= 18520.0) {
        return MAV_ODID_HOR_ACC_UNKNOWN;
    }

    static const struct {
        float accuracy;                 // Accuracy bound in meters
        MAV_ODID_HOR_ACC mavoutput;     // mavlink enum output
    } horiz_accuracy_table[] = {
        { 1.0,    MAV_ODID_HOR_ACC_1_METER},
        { 3.0,    MAV_ODID_HOR_ACC_3_METER},
        {10.0,    MAV_ODID_HOR_ACC_10_METER},
        {30.0,    MAV_ODID_HOR_ACC_30_METER},
        {92.6,    MAV_ODID_HOR_ACC_0_05NM},
        {185.2,   MAV_ODID_HOR_ACC_0_1NM},
        {555.6,   MAV_ODID_HOR_ACC_0_3NM},
        {926.0,   MAV_ODID_HOR_ACC_0_5NM},
        {1852.0,  MAV_ODID_HOR_ACC_1NM},
        {3704.0,  MAV_ODID_HOR_ACC_2NM},
        {7408.0,  MAV_ODID_HOR_ACC_4NM},
        {18520.0, MAV_ODID_HOR_ACC_10NM},
    };

    for (auto elem : horiz_accuracy_table) {
        if (accuracy < elem.accuracy) {
            return elem.mavoutput;
        }
    }

    // Should not reach this
    return MAV_ODID_HOR_ACC_UNKNOWN;
}

/**
* This converts a vertical accuracy float value to the corresponding enum
*
* @param Accuracy The vertical accuracy in meters
* @return Enum value representing the accuracy
*/
MAV_ODID_VER_ACC AP_OpenDroneID::create_enum_vertical_accuracy(float accuracy) const
{
    // Out of bounds return UNKNOWN flag
    if (accuracy < 0.0 || accuracy >= 150.0) {
        return MAV_ODID_VER_ACC_UNKNOWN;
    }

    static const struct {
        float accuracy;                 // Accuracy bound in meters
        MAV_ODID_VER_ACC mavoutput;     // mavlink enum output
    } vertical_accuracy_table[] = {
        { 1.0,  MAV_ODID_VER_ACC_1_METER},
        { 3.0,  MAV_ODID_VER_ACC_3_METER},
        {10.0,  MAV_ODID_VER_ACC_10_METER},
        {25.0,  MAV_ODID_VER_ACC_25_METER},
        {45.0,  MAV_ODID_VER_ACC_45_METER},
        {150.0, MAV_ODID_VER_ACC_150_METER},
    };

    for (auto elem : vertical_accuracy_table) {
        if (accuracy < elem.accuracy) {
            return elem.mavoutput;
        }
    }

    // Should not reach this
    return MAV_ODID_VER_ACC_UNKNOWN;
}

/**
* This converts a speed accuracy float value to the corresponding enum
*
* @param Accuracy The speed accuracy in m/s
* @return Enum value representing the accuracy
*/
MAV_ODID_SPEED_ACC AP_OpenDroneID::create_enum_speed_accuracy(float accuracy) const
{
    // Out of bounds return UNKNOWN flag
    if (accuracy < 0.0 || accuracy >= 10.0) {
        return MAV_ODID_SPEED_ACC_UNKNOWN;
    }

    if (accuracy < 0.3) {
        return MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND;
    } else if (accuracy < 1.0) {
        return MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND;
    } else if (accuracy < 3.0) {
        return MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND;
    } else if (accuracy < 10.0) {
        return MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND;
    }

    // Should not reach this
    return MAV_ODID_SPEED_ACC_UNKNOWN;
}

/**
* This converts a timestamp accuracy float value to the corresponding enum
*
* @param Accuracy The timestamp accuracy in seconds
* @return Enum value representing the accuracy
*/
MAV_ODID_TIME_ACC AP_OpenDroneID::create_enum_timestamp_accuracy(float accuracy) const
{
    // Out of bounds return UNKNOWN flag
    if (accuracy < 0.0 || accuracy >= 1.5) {
        return MAV_ODID_TIME_ACC_UNKNOWN;
    }

    static const MAV_ODID_TIME_ACC mavoutput [15] = {
        MAV_ODID_TIME_ACC_0_1_SECOND,
        MAV_ODID_TIME_ACC_0_2_SECOND,
        MAV_ODID_TIME_ACC_0_3_SECOND,
        MAV_ODID_TIME_ACC_0_4_SECOND,
        MAV_ODID_TIME_ACC_0_5_SECOND,
        MAV_ODID_TIME_ACC_0_6_SECOND,
        MAV_ODID_TIME_ACC_0_7_SECOND,
        MAV_ODID_TIME_ACC_0_8_SECOND,
        MAV_ODID_TIME_ACC_0_9_SECOND,
        MAV_ODID_TIME_ACC_1_0_SECOND,
        MAV_ODID_TIME_ACC_1_1_SECOND,
        MAV_ODID_TIME_ACC_1_2_SECOND,
        MAV_ODID_TIME_ACC_1_3_SECOND,
        MAV_ODID_TIME_ACC_1_4_SECOND,
        MAV_ODID_TIME_ACC_1_5_SECOND,
    };

    for (int8_t i = 1; i <= 15; i++) {
        if (accuracy <= 0.1 * i) {
            return mavoutput[i-1];
        }
    }

    // Should not reach this
    return MAV_ODID_TIME_ACC_UNKNOWN;
}

// make sure value is within limits of remote ID standard
uint16_t AP_OpenDroneID::create_speed_horizontal(uint16_t speed) const
{
    if (speed > ODID_MAX_SPEED_H) { // constraint function can't be used, because out of range value is invalid
        speed = ODID_INV_SPEED_H;
    }

    return speed;
}

// make sure value is within limits of remote ID standard
int16_t AP_OpenDroneID::create_speed_vertical(int16_t speed) const
{
    if (speed > ODID_MAX_SPEED_V) { // constraint function can't be used, because out of range value is invalid
        speed = ODID_INV_SPEED_V;
    } else if (speed < ODID_MIN_SPEED_V) {
        speed = ODID_INV_SPEED_V;
    }

    return speed;
}

// make sure value is within limits of remote ID standard
float AP_OpenDroneID::create_altitude(float altitude) const
{
    if (altitude > ODID_MAX_ALT) { // constraint function can't be used, because out of range value is invalid
        altitude = ODID_INV_ALT;
    } else if (altitude < ODID_MIN_ALT) {
        altitude = ODID_INV_ALT;
    }

    return altitude;
}

// make sure value is within limits of remote ID standard
float AP_OpenDroneID::create_location_timestamp(float timestamp) const
{
    if (timestamp > ODID_MAX_TIMESTAMP) { // constraint function can't be used, because out of range value is invalid
        timestamp = ODID_INV_TIMESTAMP;
    } else if (timestamp < 0) {
        timestamp = ODID_INV_TIMESTAMP;
    }

    return timestamp;
}

// handle a message from the GCS
void AP_OpenDroneID::handle_msg(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    if (!_initialised) {
        return;
    }
    WITH_SEMAPHORE(_sem);

    switch (msg.msgid) {
    // only accept ARM_STATUS from the transmitter
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS: {

        if (chan == _chan) {
            //gcs().send_text(MAV_SEVERITY_INFO, "DETECTED GCS ARM STATUS");
            mavlink_msg_open_drone_id_arm_status_decode(&msg, &arm_status);
            last_arm_status_ms = AP_HAL::millis();
        }
        break;
    }
    // accept other messages from the GCS
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID:
        mavlink_msg_open_drone_id_operator_id_decode(&msg, &pkt_operator_id);
        //gcs().send_text(MAV_SEVERITY_INFO, "DETECTED GCS OP ID");
        break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID:
        mavlink_msg_open_drone_id_self_id_decode(&msg, &pkt_self_id);
        // A GCS-sent SelfID overrides the generic default and is forwarded on to
        // the module. We additionally mirror a pilot emergency into Location.status
        // (latch on description_type==1 EMERGENCY, clear on any other type) so the
        // emergency is also carried for modules that key off operational status.
        _pilot_emergency = (pkt_self_id.description_type == MAV_ODID_DESC_TYPE_EMERGENCY);
        _last_self_id_ms = AP_HAL::millis();
        break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID: {
        mavlink_open_drone_id_basic_id_t tmp;
        mavlink_msg_open_drone_id_basic_id_decode(&msg, &tmp);
        if (tmp.id_type != MAV_ODID_ID_TYPE_NONE && strnlen((const char*)tmp.uas_id, 20) > 0) {
            pkt_basic_id = tmp;
            // Check if this is actually a new/different ID
            char new_id_str[21];
            snprintf(new_id_str, sizeof(new_id_str), "%s", (const char*)tmp.uas_id);
            const bool id_changed = (strncmp(new_id_str, id_str, sizeof(id_str)) != 0) ||
                                    (tmp.id_type != atoi(id_type)) ||
                                    (tmp.ua_type != atoi(ua_type));
            // Update persistent storage fields
            memcpy(id_str, new_id_str, sizeof(id_str));
            id_len = strlen(id_str);
            snprintf(id_type, sizeof(id_type), "%u", tmp.id_type);
            snprintf(ua_type, sizeof(ua_type), "%u", tmp.ua_type);
            // Only flash to bootloader if the ID actually changed
            if (id_changed) {
                _options.set_and_save(_options.get() | LockUASIDOnFirstBasicIDRx);
                hal.util->flash_bootloader();
                bootloader_flashed = true;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OpenDroneID: Set UAS_ID: %s", id_str);
            }
        }
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM:
        mavlink_msg_open_drone_id_system_decode(&msg, &pkt_system);
        //gcs().send_text(MAV_SEVERITY_INFO, "DETECTED GCS SYSTEM");
        last_system_ms = AP_HAL::millis();
        break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE: {
        //gcs().send_text(MAV_SEVERITY_INFO, "DETECTED GCS SYS UPDATE");
        mavlink_open_drone_id_system_update_t pkt_system_update;
        mavlink_msg_open_drone_id_system_update_decode(&msg, &pkt_system_update);
        pkt_system.operator_latitude = pkt_system_update.operator_latitude;
        pkt_system.operator_longitude = pkt_system_update.operator_longitude;
        pkt_system.operator_altitude_geo = pkt_system_update.operator_altitude_geo;
        pkt_system.timestamp = pkt_system_update.timestamp;
        last_system_update_ms = AP_HAL::millis();
        if (last_system_ms != 0) {
            // we can only mark system as updated if we have the other
            // information already
            last_system_ms = last_system_update_ms;
        }
        break;
    }
    }
}

// singleton instance
AP_OpenDroneID *AP_OpenDroneID::_singleton;

namespace AP
{

AP_OpenDroneID &opendroneid()
{
    return *AP_OpenDroneID::get_singleton();
}

}
#endif //AP_OPENDRONEID_ENABLED
