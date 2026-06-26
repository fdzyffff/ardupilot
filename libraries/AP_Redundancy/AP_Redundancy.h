// libraries/AP_Redundancy/AP_Redundancy.h
#pragma once

#include "AP_Redundancy_config.h"

#ifndef ENABLE_REDUNDANCY_CONTROL
#define ENABLE_REDUNDANCY_CONTROL 0
#endif

#if ENABLE_REDUNDANCY_CONTROL

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Common/Location.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

// ---- Frame Structures ----

// FPGA → FMU frame (v2: 36 bytes total = header(2)+len(1)+data(32)+chk(1))
struct RedundancyDataFrame_from_FPGA {
    uint8_t  header[2];           // 0xA5 0x5A
    uint8_t  frame_length;        // v1=26, v2=32
    uint32_t timestamp;
    uint8_t  redundancy_status;   // bits6-7: this FMU num, bits3-4: ctrl FMU num, bits0-2: health
    uint8_t  adc_data[21];        // 7 channels x 3 bytes (24-bit)
    // v2 extended fields
    uint8_t  switch_reason;       // 0x00=none, 0x01=timeout, 0x02=vote_fail, 0x03=startup
    uint32_t switch_timestamp;    // FPGA time when switch occurred
    uint8_t  fpga_version;        // FPGA program version (was reserved)
    uint8_t  checksum;
} __attribute__((packed));

// FMU ↔ FMU / FMU → FPGA comm frame (v3: 85 bytes total)
struct RedundancyCommFrame {
    uint8_t  header[2];           // 0xA5 0x5A
    uint8_t  frame_length;        // v1=71, v2=78, v3=80
    uint32_t timestamp;
    uint8_t  redundancy_health;   // bits6-7: sender num, bits0-2: health flags
    uint8_t  work_mode;
    uint8_t  unlock_status;
    float    roll_deg;
    float    pitch_deg;
    float    yaw_deg;
    float    vel_n;
    float    vel_e;
    float    vel_d;
    int32_t  lat;                 // 1e7 degrees
    int32_t  lng;                 // 1e7 degrees
    int32_t  alt;                 // cm
    uint16_t pwm[REDUNDANCY_MAX_PWM_CHANNELS];
    // v2+ extended fields
    uint32_t param_hash;          // CRC32 of critical parameters
    uint8_t  frame_version;       // = REDUNDANCY_FRAME_VERSION
    // v3: mode & state split into independent 14-bit masks (low byte = ch1-8, high byte low 6 bits = ch9-14)
    uint8_t  relay_mode_low;      // bits0-7: relay-mode flag for ch1-8  (1=digital/relay, 0=PWM)
    uint8_t  relay_mode_high;     // bits0-5: relay-mode flag for ch9-14
    uint8_t  relay_state_low;     // bits0-7: relay target state for ch1-8  (1=high, 0=low)
    uint8_t  relay_state_high;    // bits0-5: relay target state for ch9-14
    uint8_t  checksum;
} __attribute__((packed));

// ---- Parse State Machine ----

enum ParseState {
    PARSE_WAIT_HEADER1 = 0,
    PARSE_WAIT_HEADER2,
    PARSE_WAIT_FRAME_LENGTH,
    PARSE_WAIT_DATA,
};

struct ParseContext {
    ParseState state;
    uint8_t    buffer[256];
    uint16_t   buf_index;
    uint16_t   expected_length;
    uint32_t   frame_count;
    uint32_t   last_frame_time_ms;

    ParseContext() : state(PARSE_WAIT_HEADER1), buf_index(0),
                     expected_length(0), frame_count(0), last_frame_time_ms(0) {}
};

// ---- Main Class ----

class AP_Redundancy {
public:
    AP_Redundancy();

    // Lifecycle
    void init(AP_SerialManager &serial_manager);
    void update();  // Called at 400Hz from scheduler

    static const struct AP_Param::GroupInfo var_info[];

    // External query interface (used by AP_CANopen, AP_Vehicle, etc.)
    bool is_in_control() const;
    uint8_t get_this_redundancy_num() const { return _this_redundancy_num; }
    uint8_t get_ctrl_redundancy_num() const { return _last_ctrl_redundancy_num; }
    bool is_param_check_passed() const { return _param_check_passed; }
    uint8_t get_last_switch_reason() const { return _last_switch_reason; }

    // Singleton
    static AP_Redundancy *get_singleton() { return _singleton; }

protected:
    // Vehicle-specific interface (subclass must implement)
    virtual uint8_t vehicle_mode_number() const = 0;
    virtual void vehicle_set_failover_mode(uint8_t prev_ctrl_mode) = 0;
    virtual void vehicle_set_target_location(const Location &loc) = 0;
    virtual bool vehicle_arm(AP_Arming::Method method) = 0;
    virtual void vehicle_disarm(AP_Arming::Method method) = 0;
    virtual bool vehicle_is_armed() const = 0;
    virtual bool vehicle_is_flying() const = 0;
    virtual Location vehicle_current_location() const = 0;

private:
    static AP_Redundancy *_singleton;

    // UART drivers
    AP_HAL::UARTDriver *_uart_fpga;     // Serial6
    AP_HAL::UARTDriver *_uart_fmu_a;    // Serial7
    AP_HAL::UARTDriver *_uart_fmu_b;    // Serial8
    bool _initialized;

    // Parse contexts
    ParseContext _ctx_fpga;
    ParseContext _ctx_fmu_a;
    ParseContext _ctx_fmu_b;

    // Redundancy state
    uint8_t  _this_redundancy_num;       // 0=unassigned, 1-3
    uint8_t  _last_ctrl_redundancy_num;  // 0=unassigned, 1-3
    uint8_t  _redundancy_status;
    uint8_t  _last_switch_reason;
    uint32_t _last_switch_timestamp;
    bool     _failover_ctrl_initialized; // true after first FPGA ctrl assignment seen
    uint8_t  _prev_ctrl_work_mode;      // work_mode of previous controller before switchover

    // ADC
    float _adc_values[7];

    // Timing
    uint32_t _last_heartbeat_from_FPGA_ms;
    uint32_t _last_update_ms;
    uint32_t _last_diag_ms;
    uint32_t _fmu_a_last_fpga_timestamp;
    uint32_t _fmu_b_last_fpga_timestamp;

    // Param hash
    uint32_t _local_param_hash;
    uint32_t _fmu_param_hash[3];         // index 0=FMU1, 1=FMU2, 2=FMU3
    bool     _param_check_passed;
    uint32_t _last_param_hash_compute_ms;

    // Follow-arm state
    uint8_t  _follow_arm_retry_count;
    uint32_t _last_follow_arm_attempt_ms;

    // Relay state
    uint16_t _relay_mode_mask;           // 14-bit: which channels are relay mode
    uint16_t _relay_state;               // 14-bit: relay target state
    uint16_t _relay_conflict_reported = 0; // per-channel latch, persistent across calls
                                            // (not reset by update_relay_state like the two above)

    // UART crossbar configuration
    AP_Int8 _uart_map[REDUNDANCY_UMAP_COUNT];
    bool    _uart_config_sent;
    uint8_t _uart_config_send_count;
    uint32_t _uart_config_last_send_ms;

    // Internal methods
    bool parse_frame_byte(ParseContext &ctx, uint8_t byte);
    void read_uart_data(AP_HAL::UARTDriver *uart, ParseContext &ctx);
    void process_fpga_frame();
    void process_fmu_frame(ParseContext &ctx, const char *source_name);
    void pack_and_send_comm_frame();
    void handle_failover();
    void handle_follow_arm();
    void handle_follow_disarm();
    uint8_t evaluate_health();
    uint32_t compute_param_hash();
    void update_relay_state();
    void report_relay_pwm_conflict(uint16_t conflict_now);
    void update_battery_from_adc();
    const RedundancyCommFrame *get_peer_frame(uint8_t fmu_num) const;
    bool validate_uart_map() const;
    void send_uart_config_frame();

    // ADC helpers
    static uint32_t get_adc_value_24bit(const uint8_t *adc_data, uint8_t channel);
    static float convert_adc_to_float(uint32_t raw);
};

#endif // ENABLE_REDUNDANCY_CONTROL
