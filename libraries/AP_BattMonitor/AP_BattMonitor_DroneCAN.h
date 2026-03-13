#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"
#if HAL_ENABLE_DRONECAN_DRIVERS
#include <AP_DroneCAN/AP_DroneCAN.h>

#define AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS         1500000 // sensor becomes unhealthy if no successful readings for 1.5 seconds
#define AP_BATTMONITOR_UAVCAN_ERROR_DEBOUNCE_MS     1000    // error flags must persist for 1 second before reporting UNHEALTHY

#ifndef AP_BATTMONITOR_UAVCAN_MPPT_DEBUG
#define AP_BATTMONITOR_UAVCAN_MPPT_DEBUG 0
#endif

// BMS error flag definitions (must match BMS interface firmware)
// Upper 16 bits are errors (block arming), lower 16 bits are warnings (allow arming)
enum BMS_ErrorFlags : uint32_t {
    BMS_ERR_CELL_OVERTEMP        = 1UL << 31,
    BMS_ERR_CHIP_OVERTEMP        = 1UL << 30,
    BMS_ERR_CELL_OUT_OF_BALANCE  = 1UL << 29,
    BMS_ERR_COMM_BQ76942         = 1UL << 28,
    BMS_ERR_COMM_AIRCRAFT        = 1UL << 27,
    BMS_ERR_COMM_OTHER_BATT      = 1UL << 26,
    BMS_ERR_COV                  = 1UL << 25,  // cell overvoltage
    BMS_ERR_CUV                  = 1UL << 24,  // cell undervoltage
    BMS_ERR_FET_OVERTEMP         = 1UL << 23,
    BMS_ERR_OVERCURRENT          = 1UL << 22,
    BMS_ERR_CELL_UNDERTEMP       = 1UL << 21,
    BMS_ERR_GENERIC              = 1UL << 20,
    BMS_ERR_BAY_CHG_ID_LOSS      = 1UL << 19,
    // Warnings (lower 16 bits) - do not block arming
    BMS_WARN_CELL_TEMP           = 1UL << 15,
    BMS_WARN_CHIP_TEMP           = 1UL << 14,
    BMS_WARN_CELL_OUT_OF_BALANCE = 1UL << 13,
    BMS_WARN_OVERCURRENT         = 1UL << 12,
};

// Mask for errors only (upper 16 bits) - these block arming and report UNHEALTHY
#define BMS_ERROR_MASK  0xFFFF0000UL
// Mask for warnings only (lower 16 bits) - these allow arming but may be reported
#define BMS_WARNING_MASK  0x0000FFFFUL

// Battery operating modes (must match BMS interface firmware)
enum BMS_OperatingMode : uint8_t {
    BMS_MODE_INIT             = 0,
    BMS_MODE_PRE_ACTIVE       = 1,
    BMS_MODE_ACTIVE_ON_NORMAL = 2,
    BMS_MODE_ACTIVE_ON_FLYING = 3,
    BMS_MODE_CHARGE           = 4,
    BMS_MODE_ERROR            = 5,
    BMS_MODE_SHUTDOWN         = 6,
    BMS_MODE_BALANCING_ONLY   = 7,
    BMS_MODE_DEEPSLEEP        = 8,
    BMS_MODE_PASSIVE_ON       = 9,
    BMS_MODE_HOTSWAP          = 10,
    BMS_MODE_TEST             = 11,
};

class AP_BattMonitor_DroneCAN : public AP_BattMonitor_Backend
{
public:
    enum BattMonitor_DroneCAN_Type {
        UAVCAN_BATTERY_INFO = 0
    };

    /// Constructor
    AP_BattMonitor_DroneCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_DroneCAN_Type type, AP_BattMonitor_Params &params);

    static const struct AP_Param::GroupInfo var_info[];

    void init() override {}

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
    bool capacity_remaining_pct(uint8_t &percentage) const override;

    bool has_temperature() const override { return _has_temperature; }

    // Additional temperature accessors for BMS components
    bool get_temp_chip(float &temperature) const override;
    bool get_temp_cpu(float &temperature) const override;
    bool get_temp_fet(float &temperature) const override;

    bool has_current() const override { return true; }

    // Always have consumed energy, either directly from BatteryInfoAux msg or by cumulative current draw
    bool has_consumed_energy() const override { return true; }

    bool has_time_remaining() const override { return _has_time_remaining; }

    bool has_cell_voltages() const override { return _has_cell_voltages; }

    bool get_cycle_count(uint16_t &cycles) const override;

    // return mavlink fault bitmask (see MAV_BATTERY_FAULT enum)
    uint32_t get_mavlink_fault_bitmask() const override;

    // return battery operating mode
    uint8_t get_operating_mode() const override { return _operating_mode; }

    // return battery error flags
    uint32_t get_error_flags() const override { return _errorFlags; }

    // return model name string from DroneCAN BatteryInfo message
    const char *get_model_name() const override { return _has_model_name ? _model_name : nullptr; }

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);
    static AP_BattMonitor_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, uint8_t battery_id);
    static void handle_battery_info_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_power_BatteryInfo &msg);
    static void handle_battery_info_aux_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_power_BatteryInfoAux &msg);
    static void handle_mppt_stream_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const mppt_Stream &msg);

    void mppt_set_powered_state(bool power_on) override;

    // reset remaining percentage to given value
    bool reset_remaining(float percentage) override;

    // return true if this DroneCAN battery backend has the RESERVED_A "ready for arm" flag set
    bool is_ready_for_arm() const;

    // return true if we've received at least one BatteryInfo message from this battery
    bool has_received_data() const { return _interim_state.last_time_micros != 0; }

    // return true if battery data has timed out (received data before but not recently)
    bool has_timed_out() const;

    // return true if error flags have persisted long enough to report UNHEALTHY
    // (debounces transient errors during hot-swap)
    bool has_persistent_error_flags() const;

    // get the specific reason why battery is not ready for arm
    // returns true if battery is not ready and writes reason to buffer
    // returns false if battery is ready (no error)
    bool get_not_ready_reason(char *buffer, size_t buflen) const;

    // return true if at least two UAVCAN BatteryInfo backends are present and both are ready for arm
    static bool all_batteries_ready_for_arm();

private:
    void handle_battery_info(const uavcan_equipment_power_BatteryInfo &msg);
    void handle_battery_info_aux(const ardupilot_equipment_power_BatteryInfoAux &msg);
    void update_interim_state(const float voltage, const float current, const float temperature_K, const uint8_t soc, uint8_t soh_pct);

    static bool match_battery_id(uint8_t instance, uint8_t battery_id);

    // MPPT related enums and methods
    enum class MPPT_FaultFlags : uint8_t {
        OVER_VOLTAGE        = (1U<<0),
        UNDER_VOLTAGE       = (1U<<1),
        OVER_CURRENT        = (1U<<2),
        OVER_TEMPERATURE    = (1U<<3),
    };
    void handle_mppt_stream(const mppt_Stream &msg);
    void mppt_check_powered_state();

#if AP_BATTMONITOR_UAVCAN_MPPT_DEBUG
    static void mppt_report_faults(const uint8_t instance, const uint8_t fault_flags);
    static const char* mppt_fault_string(const MPPT_FaultFlags fault);
#endif

    // Return true if the DroneCAN state of charge should be used.
    // Return false if state of charge should be calculated locally by counting mah.
    bool use_CAN_SoC() const;

    AP_BattMonitor::BattMonitor_State _interim_state;

    HAL_Semaphore _sem_battmon;

    AP_DroneCAN* _ap_dronecan;
    uint8_t _soc;
    uint8_t _node_id;
    uint16_t _cycle_count;
    float _remaining_capacity_wh;
    float _full_charge_capacity_wh;
    bool _has_temperature;
    bool _has_cell_voltages;
    bool _has_time_remaining;
    bool _has_battery_info_aux;

    // Additional BMS temperatures (stored in Celsius)
    float _temp_chip;           // BQ chip temperature
    float _temp_cpu;            // STM32 processor temperature
    float _temp_fet;            // FET temperature
    bool _has_temp_chip;
    bool _has_temp_cpu;
    bool _has_temp_fet;
    uint8_t _instance;                  // instance of this battery monitor

 // set when incoming UAVCAN BatteryInfo.status_flags has RESERVED_A bit set
    bool _ready_for_arm = false;
    uint32_t _errorFlags = 0;
    uint32_t _error_flags_first_seen_ms = 0;  // timestamp when error flags first became non-zero (0 = no errors)
    uint8_t _operating_mode = 0;  // battery operating mode from state_of_charge_pct_stdev field

    char _model_name[32];           // model name from DroneCAN BatteryInfo (e.g. "Arcsky-2603001")
    bool _has_model_name = false;   // true once model_name received from BatteryInfo
    bool _model_name_logged = false; // true once model_name has been written to the log

    AP_Float _curr_mult;                 // scaling multiplier applied to current reports for adjustment
    // MPPT variables
    struct {
        bool is_detected;               // true if this UAVCAN device is a Packet Digital MPPT
        bool powered_state;             // true if the mppt is powered on, false if powered off
        bool vehicle_armed_last;        // latest vehicle armed state. used to detect changes and power on/off MPPT board
        uint8_t fault_flags;            // bits holding fault flags
        uint32_t powered_state_remote_ms; // timestamp of when request was sent, zeroed on response. Used to retry
    } _mppt;

    void handle_outputEnable_response(const CanardRxTransfer&, const mppt_OutputEnableResponse&);

    Canard::ObjCallback<AP_BattMonitor_DroneCAN, mppt_OutputEnableResponse> mppt_outputenable_res_cb{this, &AP_BattMonitor_DroneCAN::handle_outputEnable_response};
    Canard::Client<mppt_OutputEnableResponse> *mppt_outputenable_client;
};
#endif
