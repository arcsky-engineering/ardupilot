#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor_Generic_Fuel.h"
#include "../../ArduCopter/Copter.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_Generic_Fuel::var_info[] = {

    // @Param: VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Sets the analog input pin that should be used for voltage monitoring.
    // @Values: -1:Disabled, 2:Pixhawk/Pixracer/Navio2/Pixhawk2_PM1, 5:Navigator, 13:Pixhawk2_PM2/CubeOrange_PM2, 14:CubeOrange, 16:Durandal, 100:PX4-v1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("VOLT_PIN", 1, AP_BattMonitor_Generic_Fuel, _volt_pin, AP_BATT_VOLT_PIN),

    // @Param: CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Sets the analog input pin that should be used for current monitoring.
    // @Values: -1:Disabled, 3:Pixhawk/Pixracer/Navio2/Pixhawk2_PM1, 4:CubeOrange_PM2/Navigator, 14:Pixhawk2_PM2, 15:CubeOrange, 17:Durandal, 101:PX4-v1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("CURR_PIN", 2, AP_BattMonitor_Generic_Fuel, _curr_pin, AP_BATT_CURR_PIN),

    // @Param: VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage of the voltage sensing pin (@PREFIX@VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.
    // @User: Advanced
    AP_GROUPINFO("VOLT_MULT", 3, AP_BattMonitor_Generic_Fuel, _volt_multiplier, AP_BATT_VOLTDIVIDER_DEFAULT),

    // @Param: AMP_PERVLT
    // @DisplayName: Amps per volt
    // @Description: Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.
    // @Units: A/V
    // @User: Standard
    AP_GROUPINFO("AMP_PERVLT", 4, AP_BattMonitor_Generic_Fuel, _curr_amp_per_volt, AP_BATT_CURR_AMP_PERVOLT_DEFAULT),

    // @Param: AMP_OFFSET
    // @DisplayName: AMP offset
    // @Description: Voltage offset at zero current on current sensor
    // @Units: V
    // @User: Standard
    AP_GROUPINFO("AMP_OFFSET", 5, AP_BattMonitor_Generic_Fuel, _curr_amp_offset, AP_BATT_CURR_AMP_OFFSET_DEFAULT),

    // @Param: VLT_OFFSET
    // @DisplayName: Volage offset
    // @Description: Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("VLT_OFFSET", 6, AP_BattMonitor_Generic_Fuel, _volt_offset, 0),
    
    // Param indexes must be less than 10 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

/// Constructor
AP_BattMonitor_Generic_Fuel::AP_BattMonitor_Generic_Fuel(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
    
    _volt_pin_analog_source = hal.analogin->channel(_volt_pin);
    _curr_pin_analog_source = hal.analogin->channel(_curr_pin);

}

// read - read the voltage and current
void
AP_BattMonitor_Generic_Fuel::read()
{
    // this copes with changing the pin at runtime
    _state.healthy = _volt_pin_analog_source->set_pin(_volt_pin);

    // get voltage
    _state.voltage = (_volt_pin_analog_source->voltage_average() - _volt_offset) * _volt_multiplier;

    // read current
    if (has_current()) {
        // calculate time since last current read
        const uint32_t tnow = AP_HAL::micros();
        const uint32_t dt_us = tnow - _state.last_time_micros;

        // this copes with changing the pin at runtime
        _state.healthy &= _curr_pin_analog_source->set_pin(_curr_pin);

        // read current
        _state.current_amps = (_curr_pin_analog_source->voltage_average() - _curr_amp_offset) * _curr_amp_per_volt;

        update_consumed(_state, dt_us);

        // record time
        _state.last_time_micros = tnow;
    }
}

/// return true if battery provides current info
bool AP_BattMonitor_Generic_Fuel::has_current() const
{
    //return ((AP_BattMonitor::Type)_params._type.get() == AP_BattMonitor::Type::ANALOG_VOLTAGE_AND_CURRENT);
    return true;
}


// write custom function for updating the battery capacity if the generator is active in the system
bool AP_BattMonitor_Generic_Fuel::capacity_remaining_pct(uint8_t &percentage) const
{
    uint8_t pctVal;
    if (copter.get_gen_detected())
    {
        // handle capacity remaining by just providing fuel percentage
        pctVal = copter.get_fuel_pct();
        percentage = pctVal;
        return true;
    }
    else
    {
        // handle it the other way that it's already handled
        // we consider anything under 10 mAh as being an invalid capacity and so will be our measurement of remaining capacity
        if ( _params._pack_capacity <= 10) {
            return false;
        }

        // the monitor must have current readings in order to estimate consumed_mah and be healthy
        if (!has_current() || !_state.healthy) {
            return false;
        }

        const float mah_remaining = _params._pack_capacity - _state.consumed_mah;
        percentage = constrain_float(100 * mah_remaining / _params._pack_capacity, 0, UINT8_MAX);
        return true;
    }
    return pctVal;
}
