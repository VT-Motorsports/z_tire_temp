// vehicle_state.h
#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>

enum Corner : uint8_t
{
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    REAR_LEFT = 2,
    REAR_RIGHT = 3,
    NUM_CORNERS = 4
};

struct DTI_Inverter
{
    uint8_t node_id;

    // Packet 0x1F: General Data 6
    uint8_t control_mode;    // 1=Speed,2=Current,3=CurrBrake,4=Pos,7=None
    int16_t target_iq;       // [A_pk * 10]
    uint16_t motor_position; // [deg * 10]
    uint8_t is_motor_still;  // 1=still, 0=rotating

    // Packet 0x20: General Data 1
    int32_t erpm;          // [ERPM]
    int16_t duty_cycle;    // [% * 10]
    int16_t input_voltage; // [V]

    // Packet 0x21: General Data 2
    int16_t ac_current; // [A_pk * 10]
    int16_t dc_current; // [A_dc * 10]

    // Packet 0x22: General Data 3
    int16_t controller_temp; // [°C * 10]
    int16_t motor_temp;      // [°C * 10]
    uint8_t fault_code;      // 0x00=None ... 0x0A=AnalogErr

    // Packet 0x23: General Data 4
    int32_t id; // [A_pk * 100]
    int32_t iq; // [A_pk * 100]

    // Packet 0x24: General Data 5
    int8_t throttle_signal; // [%]
    int8_t brake_signal;    // [%]

    bool digital_in1;
    bool digital_in2;
    bool digital_in3;
    bool digital_in4;
    bool digital_out1;
    bool digital_out2;
    bool digital_out3;
    bool digital_out4;

    bool drive_enable;

    bool limit_cap_temp;
    bool limit_dc_current;
    bool limit_drive_enable;
    bool limit_igbt_accel_temp;
    bool limit_igbt_temp;
    bool limit_input_voltage;
    bool limit_motor_accel_temp;
    bool limit_motor_temp;
    bool limit_rpm_min;
    bool limit_rpm_max;
    bool limit_power;

    uint8_t can_map_version;

    // Packet 0x25: Configured and Available AC Currents
    int16_t max_ac_current;       // [A_pk * 10]
    int16_t avail_max_ac_current; // [A_pk * 10]
    int16_t min_ac_current;       // [A_pk * 10]
    int16_t avail_min_ac_current; // [A_pk * 10]

    // Packet 0x26: Configured and Available DC Currents
    int16_t max_dc_current;       // [A_dc * 10]
    int16_t avail_max_dc_current; // [A_dc * 10]
    int16_t min_dc_current;       // [A_dc * 10]
    int16_t avail_min_dc_current; // [A_dc * 10]

    uint64_t last_rx_time_ms;
};

struct Analog
{
    uint16_t channels[8];
};

enum APPS_ERRORS
{
    PEDAL_AGREEMENT = 0,
    SHORT_CIRCUIT_P1 = 1,
    SHORT_CIRCUIT_P2 = 2,
    OPEN_CIRCUIT_P1 = 3,
    OPEN_CIRCUIT_P2 = 4,
    BRAKE_OVERLAP = 5,
    NUM_ERRORS = 8,
};

enum PEDAL_SLOPE_DIRECTION
{
    POSITIVE,
    NEGATIVE
};

namespace APPS_CONSTEXPRS
{
static constexpr uint16_t calculateRange(uint16_t highThreshold, uint16_t lowThreshold)
{
    return (highThreshold > lowThreshold) ? (highThreshold - lowThreshold) : (lowThreshold - highThreshold);
}

static constexpr PEDAL_SLOPE_DIRECTION PEDAL_SLOPE_DIRECTION(uint16_t highThreshold, uint16_t lowThreshold)
{
    return (highThreshold > lowThreshold) ? POSITIVE : NEGATIVE;
}

} // namespace APPS_CONSTEXPRS

struct APPS_data
{
    bool errors[APPS_ERRORS::NUM_ERRORS];
    bool faulted;

    // calibration values, constexpr set at compile time
    static constexpr int pedal1_adc_channel_num = 0;
    static constexpr int pedal2_adc_channel_num = 1;
    static constexpr uint16_t pedal1_low_threshold = 10000;
    static constexpr uint16_t pedal2_low_threshold = 10000;
    static constexpr uint16_t pedal1_high_threshold = 20000;
    static constexpr uint16_t pedal2_high_threshold = 20000;

    static constexpr int brake_main_adc_channel_num = 0;

    // other calibration values
    static constexpr float agreement_threshold = 0.10f; // 10% disagreement
    static constexpr int agreement_timeout_ms = 100;
    static constexpr float brake_on_threshold = 0.25f;  // throttle % to latch brake fault
    static constexpr float brake_off_threshold = 0.05f; // throttle % to clear brake fault

    // constexpr values that are calculated at compiletime
    static constexpr uint16_t pedal1_range_width =
        APPS_CONSTEXPRS::calculateRange(pedal1_high_threshold, pedal1_low_threshold);
    static constexpr uint16_t pedal2_range_width =
        APPS_CONSTEXPRS::calculateRange(pedal2_high_threshold, pedal2_low_threshold);
    static constexpr PEDAL_SLOPE_DIRECTION pedal1_slope_direction =
        APPS_CONSTEXPRS::PEDAL_SLOPE_DIRECTION(pedal1_high_threshold, pedal1_low_threshold);
    static constexpr PEDAL_SLOPE_DIRECTION pedal2_slope_direction =
        APPS_CONSTEXPRS::PEDAL_SLOPE_DIRECTION(pedal2_high_threshold, pedal2_low_threshold);

    // IN PERCENTAGE, translation of input voltages to output command AFTER APPS processing
    float commandedTorquePercentage;
    float pedal1_percent;
    float pedal2_percent;
};

// struct that provides access to sub  Interface structs that house publicly accessible data to whole program.
// classes that interact with vehicle state should refer to this struct as source of truth
class VehicleState
{
  public:
    DTI_Inverter INVERTERS[Corner::NUM_CORNERS];
    Analog analogIf;
    APPS_data APPSIf;
};
