// libraries/AP_Redundancy/AP_Redundancy.cpp
// Triple-redundancy flight controller shared library.
// Extracted and generalised from ArduPlane/redundancy_control.cpp.

#include "AP_Redundancy.h"

#if ENABLE_REDUNDANCY_CONTROL

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Relay/AP_Relay.h>

extern const AP_HAL::HAL &hal;

// Switch reason readable string (matches FPGA switch_reason encoding)
static const char *sw_reason_str(uint8_t reason)
{
    switch (reason) {
        case 0x01: return "timeout";
        case 0x02: return "vote";
        case 0x03: return "startup";
        default:   return "none";
    }
}

// File-local helper: is this SRV function a binary digital output
// that the FPGA should drive as a stable level?
// The whitelist mirrors the one used by update_relay_state()'s SRV-channel
// scan, and is also consumed by the AP_Relay conflict detector (below) to
// decide whether a non-zero SRV function on an AP_Relay-claimed channel
// constitutes a real override conflict.
static bool is_srv_relay_function(SRV_Channel::Aux_servo_function_t fn)
{
    switch (fn) {
        case SRV_Channel::k_egg_drop:
        case SRV_Channel::k_parachute_release:
        case SRV_Channel::k_gripper:
        case SRV_Channel::k_engine_run_enable:
        case SRV_Channel::k_ignition:
        case SRV_Channel::k_choke:
        case SRV_Channel::k_starter:
            return true;
        default:
            return false;
    }
}

// ---- Parameter Table ----

const AP_Param::GroupInfo AP_Redundancy::var_info[] = {
    // @Param: UMAP1
    // @DisplayName: FPGA UART1 mapping
    // @Description: Which FMU SERIAL (1-5) FPGA_UART1 maps to. 0=disconnected.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("UMAP1", 1, AP_Redundancy, _uart_map[0], REDUNDANCY_UMAP_DEFAULT_1),

    // @Param: UMAP2
    // @DisplayName: FPGA UART2 mapping
    // @Description: Which FMU SERIAL (1-5) FPGA_UART2 maps to. 0=disconnected.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("UMAP2", 2, AP_Redundancy, _uart_map[1], REDUNDANCY_UMAP_DEFAULT_2),

    // @Param: UMAP3
    // @DisplayName: FPGA UART3 mapping
    // @Description: Which FMU SERIAL (1-5) FPGA_UART3 maps to. 0=disconnected.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("UMAP3", 3, AP_Redundancy, _uart_map[2], REDUNDANCY_UMAP_DEFAULT_3),

    // @Param: UMAP4
    // @DisplayName: FPGA UART4 mapping
    // @Description: Which FMU SERIAL (1-5) FPGA_UART4 maps to. 0=disconnected.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("UMAP4", 4, AP_Redundancy, _uart_map[3], REDUNDANCY_UMAP_DEFAULT_4),

    // @Param: UMAP5
    // @DisplayName: FPGA UART5 mapping
    // @Description: Which FMU SERIAL (1-5) FPGA_UART5 maps to. 0=disconnected.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("UMAP5", 5, AP_Redundancy, _uart_map[4], REDUNDANCY_UMAP_DEFAULT_5),

    // @Param: UMAP6
    // @DisplayName: FPGA UART6 mapping
    // @Description: Which FMU SERIAL (1-5) FPGA_UART6 maps to. 0=disconnected.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("UMAP6", 6, AP_Redundancy, _uart_map[5], REDUNDANCY_UMAP_DEFAULT_6),

    // @Param: UMAP7
    // @DisplayName: FPGA UART7 mapping
    // @Description: Which FMU SERIAL (1-5) FPGA_UART7 maps to. 0=disconnected.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("UMAP7", 7, AP_Redundancy, _uart_map[6], REDUNDANCY_UMAP_DEFAULT_7),

    // @Param: UMAP8
    // @DisplayName: FPGA UART8 mapping
    // @Description: Which FMU SERIAL (1-5) FPGA_UART8 maps to. 0=disconnected.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("UMAP8", 8, AP_Redundancy, _uart_map[7], REDUNDANCY_UMAP_DEFAULT_8),

    AP_GROUPEND
};

// Singleton pointer
AP_Redundancy *AP_Redundancy::_singleton = nullptr;

// ---- Constructor ----

AP_Redundancy::AP_Redundancy()
    : _uart_fpga(nullptr)
    , _uart_fmu_a(nullptr)
    , _uart_fmu_b(nullptr)
    , _initialized(false)
    , _this_redundancy_num(0)
    , _last_ctrl_redundancy_num(0)
    , _redundancy_status(0)
    , _last_switch_reason(0)
    , _last_switch_timestamp(0)
    , _failover_ctrl_initialized(false)
    , _prev_ctrl_work_mode(0xFF)
    , _last_heartbeat_from_FPGA_ms(0)
    , _last_update_ms(0)
    , _last_diag_ms(0)
    , _fmu_a_last_fpga_timestamp(0)
    , _fmu_b_last_fpga_timestamp(0)
    , _local_param_hash(REDUNDANCY_PARAM_HASH_NONE)
    , _param_check_passed(false)
    , _last_param_hash_compute_ms(0)
    , _follow_arm_retry_count(0)
    , _last_follow_arm_attempt_ms(0)
    , _relay_mode_mask(0)
    , _relay_state(0)
    , _uart_config_sent(false)
    , _uart_config_send_count(0)
    , _uart_config_last_send_ms(0)
{
    memset(_adc_values, 0, sizeof(_adc_values));
    memset(_fmu_param_hash, 0, sizeof(_fmu_param_hash));

    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// ---- init() ----

void AP_Redundancy::init(AP_SerialManager &serial_manager)
{
    if (_initialized) {
        return;
    }

    _uart_fpga = serial_manager.find_serial(AP_SerialManager::SerialProtocol_redundancy_FPGA, 0);
    if (_uart_fpga == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Redundancy: Serial(FPGA) not available");
        return;
    }
    _uart_fpga->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

    _uart_fmu_a = serial_manager.find_serial(AP_SerialManager::SerialProtocol_redundancy_FMUa, 0);
    if (_uart_fmu_a == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Redundancy: Serial(FMUa) not available");
        return;
    }
    _uart_fmu_a->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

    _uart_fmu_b = serial_manager.find_serial(AP_SerialManager::SerialProtocol_redundancy_FMUb, 0);
    if (_uart_fmu_b == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Redundancy: Serial(FMUb) not available");
        return;
    }
    _uart_fmu_b->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

    _initialized = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Redundancy: initialized");
}

// ---- Frame parser (4-state state machine) ----

bool AP_Redundancy::parse_frame_byte(ParseContext &ctx, uint8_t byte)
{
    switch (ctx.state) {
        case PARSE_WAIT_HEADER1:
            if (byte == REDUNDANCY_FRAME_HEADER1) {
                ctx.buffer[0] = byte;
                ctx.buf_index = 1;
                ctx.state = PARSE_WAIT_HEADER2;
            }
            break;

        case PARSE_WAIT_HEADER2:
            if (byte == REDUNDANCY_FRAME_HEADER2) {
                ctx.buffer[1] = byte;
                ctx.buf_index = 2;
                ctx.state = PARSE_WAIT_FRAME_LENGTH;
            } else {
                ctx.state = PARSE_WAIT_HEADER1;
            }
            break;

        case PARSE_WAIT_FRAME_LENGTH:
            ctx.expected_length = byte;
            ctx.buffer[2] = byte;
            ctx.buf_index = 3;
            ctx.state = PARSE_WAIT_DATA;
            break;

        case PARSE_WAIT_DATA:
            if (ctx.buf_index < sizeof(ctx.buffer)) {
                ctx.buffer[ctx.buf_index++] = byte;
            }
            // Total frame = header(2) + frame_length_byte(1) + data(expected_length) + checksum(1)
            {
                uint16_t total_expected = 2U + 1U + ctx.expected_length + 1U;
                if (ctx.buf_index >= total_expected) {
                    // Verify checksum: sum bytes from index 2 to 2+expected_length (inclusive of length byte)
                    uint8_t checksum = 0;
                    for (uint16_t i = 2; i < 2U + 1U + ctx.expected_length; i++) {
                        checksum += ctx.buffer[i];
                    }
                    ctx.state = PARSE_WAIT_HEADER1;
                    if (checksum == ctx.buffer[total_expected - 1]) {
                        ctx.frame_count++;
                        ctx.last_frame_time_ms = AP_HAL::millis();
                        return true;
                    }
                    // bad checksum – silently discard
                }
            }
            break;
    }
    return false;
}

// ---- read_uart_data() ----

void AP_Redundancy::read_uart_data(AP_HAL::UARTDriver *uart, ParseContext &ctx)
{
    if (uart == nullptr) {
        return;
    }
    uint8_t byte;
    while (uart->read(&byte, 1) == 1) {
        if (parse_frame_byte(ctx, byte)) {
            // Dispatch strictly by current frame length (v3 only)
            if (ctx.expected_length == REDUNDANCY_FPGA_FRAME_LEN && uart == _uart_fpga) {
                process_fpga_frame();
            } else if (ctx.expected_length == REDUNDANCY_COMM_FRAME_LEN) {
                if (uart == _uart_fmu_a) {
                    process_fmu_frame(_ctx_fmu_a, "FMUa");
                } else if (uart == _uart_fmu_b) {
                    process_fmu_frame(_ctx_fmu_b, "FMUb");
                }
            }
        }
    }
}

// ---- process_fpga_frame() ----

void AP_Redundancy::process_fpga_frame()
{
    const RedundancyDataFrame_from_FPGA *frame =
        reinterpret_cast<const RedundancyDataFrame_from_FPGA *>(_ctx_fpga.buffer);

    // Extract this FMU number from bits 6-7 (01=FMU1, 10=FMU2, 11=FMU3)
    if (_this_redundancy_num == 0) {
        _this_redundancy_num = (frame->redundancy_status >> 6) & 0x03;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Redundancy: this=FMU%u", _this_redundancy_num);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "Redundancy: FPGA version=%u",
                      (unsigned)frame->fpga_version);
    }

    // Extract controlling FMU number from bits 3-4
    uint8_t ctrl_num = (frame->redundancy_status >> 3) & 0x03;

    // Lower 6 bits are status flags
    _redundancy_status = frame->redundancy_status & 0x3F;

    // Update FPGA timestamp (used for health evaluation)
    _last_heartbeat_from_FPGA_ms = frame->timestamp;

    // Parse ADC channels
    for (uint8_t i = 0; i < 7; i++) {
        uint32_t raw = get_adc_value_24bit(frame->adc_data, i);
        _adc_values[i] = convert_adc_to_float(raw);
    }

    // Switch reason / timestamp (always present in v3)
    _last_switch_reason    = frame->switch_reason;
    _last_switch_timestamp = frame->switch_timestamp;

    // Detect controller switch event — capture previous controller's work_mode
    if (ctrl_num != _last_ctrl_redundancy_num && _last_ctrl_redundancy_num != 0) {
        // Record mode of previous controller before updating ctrl num
        if (_last_ctrl_redundancy_num == _this_redundancy_num) {
            // We were the controller — use our own mode
            _prev_ctrl_work_mode = vehicle_mode_number();
        } else {
            // A peer was the controller — get mode from peer comm frame
            const RedundancyCommFrame *prev_frame = get_peer_frame(_last_ctrl_redundancy_num);
            _prev_ctrl_work_mode = (prev_frame != nullptr) ? prev_frame->work_mode : 0xFF;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "TMR SWITCH F%u->F%u reason=%s prev_mode=%u",
                      _last_ctrl_redundancy_num, ctrl_num,
                      sw_reason_str(_last_switch_reason),
                      _prev_ctrl_work_mode);
    }
    _last_ctrl_redundancy_num = ctrl_num;

    // Vehicle-specific ADC handling (battery scaling lives in subclass)
    update_battery_from_adc();
}

// ---- process_fmu_frame() ----

void AP_Redundancy::process_fmu_frame(ParseContext &ctx, const char *source_name)
{
    const RedundancyCommFrame *frame =
        reinterpret_cast<const RedundancyCommFrame *>(ctx.buffer);

    // Extract sender number (bits 6-7 of redundancy_health)
    uint8_t sender_num = (frame->redundancy_health >> 6) & 0x03;

    // Record the FPGA-sourced timestamp for health evaluation
    if (source_name[3] == 'a') {
        // FMUa
        _fmu_a_last_fpga_timestamp = frame->timestamp;
    } else {
        // FMUb
        _fmu_b_last_fpga_timestamp = frame->timestamp;
    }

    // Peer param hash (sender_num is 1-based, array is 0-based)
    if (sender_num >= 1 && sender_num <= 3) {
        _fmu_param_hash[sender_num - 1] = frame->param_hash;
    }
}

// ---- evaluate_health() ----
// Returns a 3-bit health field. bit0=FMU1 healthy, bit1=FMU2 healthy, bit2=FMU3 healthy.
// FMU timestamp is considered fresh if it equals last FPGA timestamp within tolerance.

uint8_t AP_Redundancy::evaluate_health()
{
    if (_this_redundancy_num == 0) {
        return 0x07;  // Unknown self – report all healthy
    }

    bool self_healthy  = true;
    bool fmu_a_healthy = (_fmu_a_last_fpga_timestamp + REDUNDANCY_HEALTH_TIMEOUT_MS >= _last_heartbeat_from_FPGA_ms);
    bool fmu_b_healthy = (_fmu_b_last_fpga_timestamp + REDUNDANCY_HEALTH_TIMEOUT_MS >= _last_heartbeat_from_FPGA_ms);

    uint8_t health = 0;
    switch (_this_redundancy_num) {
        case 1:
            if (self_healthy)  health |= 0x01;  // bit0 = FMU1
            if (fmu_a_healthy) health |= 0x02;  // bit1 = FMU2
            if (fmu_b_healthy) health |= 0x04;  // bit2 = FMU3
            break;
        case 2:
            if (fmu_a_healthy) health |= 0x01;  // bit0 = FMU1
            if (self_healthy)  health |= 0x02;  // bit1 = FMU2
            if (fmu_b_healthy) health |= 0x04;  // bit2 = FMU3
            break;
        case 3:
            if (fmu_a_healthy) health |= 0x01;  // bit0 = FMU1
            if (fmu_b_healthy) health |= 0x02;  // bit1 = FMU2
            if (self_healthy)  health |= 0x04;  // bit2 = FMU3
            break;
        default:
            health = 0x07;
            break;
    }
    return health;
}

// ---- compute_param_hash() ----
// Compute CRC32 over a fixed list of critical parameter names and their float values.
// This allows all three FMUs to detect parameter mismatches.

uint32_t AP_Redundancy::compute_param_hash()
{
    // Critical parameter names to hash
    static const char * const param_names[] = {
        "ARSPD_FBW_MIN",
        "ARSPD_FBW_MAX",
        "ARSPD_CRUISE",
        "FBWB_CLIMB_RATE",
        "TECS_CLMB_MAX",
        "TECS_SINK_MIN",
        "TECS_PITCH_MAX",
        "TECS_PITCH_MIN",
        "WP_RADIUS",
        "LIM_ROLL_CD",
        "LIM_PITCH_MAX",
        "LIM_PITCH_MIN",
        "TRIM_ARSPD_CM",
        "CRUISE_THROTTLE",
        "THR_MAX",
        "THR_MIN",
        "THR_FAILSAFE",
        "ATC_RAT_RLL_P",
        "ATC_RAT_RLL_I",
        "ATC_RAT_PIT_P",
        "ATC_RAT_PIT_I",
        nullptr  // sentinel
    };

    uint32_t crc = 0;
    for (uint8_t i = 0; param_names[i] != nullptr; i++) {
        const char *name = param_names[i];
        enum ap_var_type vtype;
        AP_Param *vp = AP_Param::find(name, &vtype);
        if (vp == nullptr) {
            continue;
        }
        // Hash the parameter name bytes
        crc = crc_crc32(crc, reinterpret_cast<const uint8_t *>(name), strlen(name));
        // Hash the float value
        float val = vp->cast_to_float(vtype);
        crc = crc_crc32(crc, reinterpret_cast<const uint8_t *>(&val), sizeof(val));
    }
    return crc;
}

// ---- update_relay_state() ----
// Scan SRV_Channels 1-14 and build two independent 14-bit bitmasks:
//   _relay_mode_mask : 1 = treat output as digital/relay (FPGA drives hard high/low)
//                       0 = normal PWM pulse train
//   _relay_state     : for relay-mode channels, 1 = drive high, 0 = drive low
//
// A channel is flagged as relay-mode when its SRV function is one of the known
// binary-output functions that ArduPilot drives between min/max (no intermediate values),
// OR when AP_Relay has a relay instance pointing at the corresponding MAIN_OUT pin.
//
// "State" is derived from the current output PWM vs. the midpoint of (min,max),
// which is robust regardless of whether the binary function drives from trim→max
// (gripper/parachute) or min→max (ignition/starter).
//
// When AP_Relay also claims a channel (see merge block at function end),
// its desired_state overrides the SRV-derived bit for that channel; the
// SRV-derived bit is preserved only for channels AP_Relay does NOT claim.

void AP_Redundancy::update_relay_state()
{
    _relay_mode_mask = 0;
    _relay_state     = 0;

    for (uint8_t ch = 0; ch < REDUNDANCY_MAX_PWM_CHANNELS; ch++) {
        const SRV_Channel *srv = SRV_Channels::srv_channel(ch);
        if (srv == nullptr) {
            continue;
        }

        SRV_Channel::Aux_servo_function_t fn = srv->get_function();
        if (!is_srv_relay_function(fn)) {
            continue;
        }

        _relay_mode_mask |= (uint16_t)(1U << ch);

        // State = (pwm_out > (min+max)/2) — works whether the function drives
        // from trim↔max (gripper/parachute) or min↔max (ignition/starter).
        const uint16_t pwm_out = srv->get_output_pwm();
        const uint16_t pwm_min = srv->get_output_min();
        const uint16_t pwm_max = srv->get_output_max();
        const uint16_t midpoint = (uint16_t)(((uint32_t)pwm_min + (uint32_t)pwm_max) / 2U);
        if (pwm_out > midpoint) {
            _relay_state |= (uint16_t)(1U << ch);
        }
    }

    // ---- AP_Relay path ----
    // Accept digital relays that AP_Relay manages (RELAY/IGNITION/PARACHUTE/
    // CAMERA/BRUSHED_REVERSE_*/ICE_STARTER) when their pin maps to a servo
    // channel 0..13. Merge semantics: mode_mask OR'd; state for channels the
    // AP_Relay claims is taken from AP_Relay's desired-state cache, overriding
    // any SRV-derived value on the same channel.

    uint16_t rly_mode = 0;
    uint16_t rly_state = 0;
    AP_Relay *relay = AP::relay();
    if (relay != nullptr) {
        relay->get_servo_channel_relay_masks(rly_mode, rly_state);
    }

    if (rly_mode != 0) {
        // Conflict detection: AP_Relay-claimed channel that ALSO has a non-zero
        // SRV function which is NOT itself a binary relay function.
        uint16_t srv_pwm_conflict = 0;
        for (uint8_t ch = 0; ch < REDUNDANCY_MAX_PWM_CHANNELS; ch++) {
            if (!(rly_mode & (uint16_t)(1U << ch))) {
                continue;
            }
            const SRV_Channel *srv = SRV_Channels::srv_channel(ch);
            if (srv == nullptr) {
                continue;
            }
            const SRV_Channel::Aux_servo_function_t fn = srv->get_function();
            if (fn == SRV_Channel::k_none) {
                continue;
            }
            if (is_srv_relay_function(fn)) {
                continue;
            }
            srv_pwm_conflict |= (uint16_t)(1U << ch);
        }
        report_relay_pwm_conflict(srv_pwm_conflict);

        // Merge: mode OR'd. For AP_Relay-claimed channels, state is
        // authoritatively taken from AP_Relay (clear the SRV-derived bit
        // first, then OR in AP_Relay's bit).
        _relay_state     &= (uint16_t)~rly_mode;
        _relay_state     |= (uint16_t)(rly_state & rly_mode);
        _relay_mode_mask |= rly_mode;
    }
}

// ---- report_relay_pwm_conflict() ----
// GCS WARNING when an AP_Relay-controlled channel is simultaneously bound
// to a non-zero SRV function that is not itself a binary relay function.
// Per-channel latch: each channel triggers at most one WARNING for its
// lifetime; a cleared-then-re-established conflict on the SAME channel
// does NOT re-fire. A fresh conflict on a previously-clean channel will.

void AP_Redundancy::report_relay_pwm_conflict(uint16_t conflict_now)
{
    const uint16_t fresh = conflict_now & ~_relay_conflict_reported;
    if (fresh == 0) {
        return;
    }
    for (uint8_t ch = 0; ch < REDUNDANCY_MAX_PWM_CHANNELS; ch++) {
        if (fresh & (uint16_t)(1U << ch)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "Redundancy: ch%u AP_Relay overrides SERVO%u_FUNCTION",
                          (unsigned)(ch + 1), (unsigned)(ch + 1));
        }
    }
    _relay_conflict_reported |= fresh;
}

// ---- update_battery_from_adc() ----
// Battery scaling is vehicle-specific (uses vehicle-specific multiplier params).
// Subclasses override this if they need ADC-based battery monitoring.

void AP_Redundancy::update_battery_from_adc()
{
    // Placeholder: vehicle adapter handles scaling.
    // _adc_values[3] and _adc_values[4] are the supply voltage channels.
    // See ArduPlane/redundancy_control.cpp for original AP_BATTERY_SCRIPTING usage.
}

// ---- validate_uart_map() ----

bool AP_Redundancy::validate_uart_map() const
{
    uint8_t serial_used[REDUNDANCY_UMAP_MAX_SERIAL + 1] = {};

    for (uint8_t i = 0; i < REDUNDANCY_UMAP_COUNT; i++) {
        uint8_t s = (uint8_t)_uart_map[i].get();
        if (s == 0) {
            continue;
        }
        if (s > REDUNDANCY_UMAP_MAX_SERIAL) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "RDN_UMAP%u=%u out of range", i + 1, s);
            return false;
        }
        if (serial_used[s] != 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "RDN_UMAP conflict: UART%u and UART%u both map to SERIAL%u",
                          serial_used[s], i + 1, s);
            return false;
        }
        serial_used[s] = i + 1;
    }
    return true;
}

// ---- send_uart_config_frame() ----

void AP_Redundancy::send_uart_config_frame()
{
    if (_uart_fpga == nullptr) {
        return;
    }

    uint8_t frame[REDUNDANCY_CFG_FRAME_TOTAL];
    frame[0] = REDUNDANCY_FRAME_HEADER1;       // 0xA5
    frame[1] = REDUNDANCY_CFG_FRAME_HEADER2;   // 0xA5
    frame[2] = REDUNDANCY_CFG_FRAME_LEN;       // 0x0A
    frame[3] = REDUNDANCY_CFG_CMD_UART_MAP;    // 0x01

    for (uint8_t i = 0; i < REDUNDANCY_UMAP_COUNT; i++) {
        frame[4 + i] = (uint8_t)_uart_map[i].get();
    }

    frame[12] = 0;  // reserved

    // XOR checksum over all preceding bytes
    uint8_t xor_sum = 0;
    for (uint8_t i = 0; i < REDUNDANCY_CFG_FRAME_TOTAL - 1; i++) {
        xor_sum ^= frame[i];
    }
    frame[REDUNDANCY_CFG_FRAME_TOTAL - 1] = xor_sum;

    _uart_fpga->write(frame, REDUNDANCY_CFG_FRAME_TOTAL);
}

// ---- pack_and_send_comm_frame() ----

void AP_Redundancy::pack_and_send_comm_frame()
{
    RedundancyCommFrame frame;
    memset(&frame, 0, sizeof(frame));

    frame.header[0] = REDUNDANCY_FRAME_HEADER1;
    frame.header[1] = REDUNDANCY_FRAME_HEADER2;

    // v3 frame: data length = total - header(2) - length_byte(1) - checksum(1)
    frame.frame_length = REDUNDANCY_COMM_FRAME_LEN;

    // Timestamp: reuse last known FPGA timestamp so peers can assess freshness
    frame.timestamp = _last_heartbeat_from_FPGA_ms;

    // redundancy_health: bits6-7 = this FMU num, bits0-2 = health flags
    uint8_t health = evaluate_health();
    frame.redundancy_health = (uint8_t)((_this_redundancy_num & 0x03) << 6) | (health & 0x07);

    // Flight mode and arm state come from vehicle subclass
    frame.work_mode     = vehicle_mode_number();
    frame.unlock_status = vehicle_is_armed() ? 1 : 0;

    // Attitude (radians → degrees)
    auto &ahrs = AP::ahrs();
    frame.roll_deg  = degrees(ahrs.get_roll());
    frame.pitch_deg = degrees(ahrs.get_pitch());
    frame.yaw_deg   = degrees(ahrs.get_yaw());

    // Velocity NED
    Vector3f vel_ned;
    if (!ahrs.get_velocity_NED(vel_ned)) {
        vel_ned.zero();
    }
    frame.vel_n = vel_ned.x;
    frame.vel_e = vel_ned.y;
    frame.vel_d = vel_ned.z;

    // Position
    Location loc = vehicle_current_location();
    frame.lat = loc.lat;
    frame.lng = loc.lng;
    frame.alt = loc.alt;

    // PWM outputs
    for (uint8_t i = 0; i < REDUNDANCY_MAX_PWM_CHANNELS; i++) {
        const SRV_Channel *srv = SRV_Channels::srv_channel(i);
        if (srv != nullptr) {
            frame.pwm[i] = srv->get_output_pwm();
        }
    }

    // v2+ extensions
    frame.param_hash      = _local_param_hash;
    frame.frame_version   = REDUNDANCY_FRAME_VERSION;

    // v3: independent relay-mode and relay-state 14-bit masks
    frame.relay_mode_low   = (uint8_t)(_relay_mode_mask & 0xFF);
    frame.relay_mode_high  = (uint8_t)((_relay_mode_mask >> 8) & 0x3F);
    frame.relay_state_low  = (uint8_t)(_relay_state & 0xFF);
    frame.relay_state_high = (uint8_t)((_relay_state >> 8) & 0x3F);

    // Checksum: sum of all bytes from frame_length byte onward (excluding checksum)
    // Checksum: sum frame_length byte + all data bytes (matching parser convention)
    {
        uint8_t checksum = 0;
        const uint8_t *p = reinterpret_cast<const uint8_t *>(&frame.frame_length);
        for (uint16_t i = 0; i <= frame.frame_length; i++) {
            checksum += p[i];
        }
        frame.checksum = checksum;
    }

    // Transmit to all three ports
    if (_uart_fpga  != nullptr) _uart_fpga ->write(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
    if (_uart_fmu_a != nullptr) _uart_fmu_a->write(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
    if (_uart_fmu_b != nullptr) _uart_fmu_b->write(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
}

// ---- get_peer_frame() ----
// Return the comm frame buffer for a given FMU number based on UART mapping.
// FMU_a / FMU_b serial assignment follows the convention:
//   FMU1: a=FMU2, b=FMU3
//   FMU2: a=FMU1, b=FMU3
//   FMU3: a=FMU1, b=FMU2

const RedundancyCommFrame *AP_Redundancy::get_peer_frame(uint8_t fmu_num) const
{
    if (fmu_num == 0 || fmu_num == _this_redundancy_num) {
        return nullptr;
    }

    // Determine which UART buffer contains the requested FMU's data
    switch (_this_redundancy_num) {
        case 1:
            if (fmu_num == 2) return reinterpret_cast<const RedundancyCommFrame *>(_ctx_fmu_a.buffer);
            if (fmu_num == 3) return reinterpret_cast<const RedundancyCommFrame *>(_ctx_fmu_b.buffer);
            break;
        case 2:
            if (fmu_num == 1) return reinterpret_cast<const RedundancyCommFrame *>(_ctx_fmu_a.buffer);
            if (fmu_num == 3) return reinterpret_cast<const RedundancyCommFrame *>(_ctx_fmu_b.buffer);
            break;
        case 3:
            if (fmu_num == 1) return reinterpret_cast<const RedundancyCommFrame *>(_ctx_fmu_a.buffer);
            if (fmu_num == 2) return reinterpret_cast<const RedundancyCommFrame *>(_ctx_fmu_b.buffer);
            break;
    }
    return nullptr;
}

// ---- handle_failover() ----
// Called the first time this FMU transitions to the controlling role.
// Arms the vehicle (if not already armed), switches to LOITER, and holds current position.

void AP_Redundancy::handle_failover()
{
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Redundancy: failover to FMU%u prev_mode=%u",
                  _this_redundancy_num, _prev_ctrl_work_mode);

    if (!vehicle_is_armed()) {
        vehicle_arm(AP_Arming::Method::SCRIPTING);
    }

    vehicle_set_failover_mode(_prev_ctrl_work_mode);

    Location current_loc = vehicle_current_location();
    vehicle_set_target_location(current_loc);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Redundancy: emergency arm + hold position");
}

// ---- handle_follow_arm() ----
// Backup FMU follows the controller's arm state, with pre-arm checks and retry limit.

void AP_Redundancy::handle_follow_arm()
{
    if (vehicle_is_armed()) {
        // Already armed – continuously hold current position as target
        Location current_loc = vehicle_current_location();
        vehicle_set_target_location(current_loc);
        return;
    }

    uint32_t now_ms = AP_HAL::millis();

    if (_follow_arm_retry_count >= REDUNDANCY_FOLLOW_ARM_MAX_RETRY) {
        // Exhausted retries
        return;
    }

    if (now_ms - _last_follow_arm_attempt_ms < REDUNDANCY_FOLLOW_ARM_RETRY_MS) {
        return;
    }

    _last_follow_arm_attempt_ms = now_ms;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Redundancy: follow-arm attempt %u/%u",
                  _follow_arm_retry_count + 1, REDUNDANCY_FOLLOW_ARM_MAX_RETRY);

    if (vehicle_arm(AP_Arming::Method::SCRIPTING)) {
        // Follow controller's current mode
        const RedundancyCommFrame *ctrl = get_peer_frame(_last_ctrl_redundancy_num);
        uint8_t ctrl_mode = (ctrl != nullptr) ? ctrl->work_mode : 0xFF;
        vehicle_set_failover_mode(ctrl_mode);
        Location current_loc = vehicle_current_location();
        vehicle_set_target_location(current_loc);
        _follow_arm_retry_count = 0;
    } else {
        _follow_arm_retry_count++;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Redundancy: follow-arm failed (attempt %u)",
                      _follow_arm_retry_count);
    }
}

// ---- handle_follow_disarm() ----
// Backup FMU follows controller disarm only if safely on the ground.

void AP_Redundancy::handle_follow_disarm()
{
    if (!vehicle_is_armed()) {
        return;
    }

    if (vehicle_is_flying()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Redundancy: follow-disarm refused (still flying)");
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Redundancy: follow-disarm");
    vehicle_disarm(AP_Arming::Method::SCRIPTING);
}

// ---- is_in_control() ----

bool AP_Redundancy::is_in_control() const
{
    return (_this_redundancy_num != 0) &&
           (_last_ctrl_redundancy_num != 0) &&
           (_this_redundancy_num == _last_ctrl_redundancy_num);
}

// ---- update() ----
// Main 10Hz control loop.

void AP_Redundancy::update()
{
    if (!_initialized ||
        _uart_fpga  == nullptr ||
        _uart_fmu_a == nullptr ||
        _uart_fmu_b == nullptr) {
        return;
    }

    // Read incoming bytes from all UARTs on every call
    read_uart_data(_uart_fpga,  _ctx_fpga);
    read_uart_data(_uart_fmu_a, _ctx_fmu_a);
    read_uart_data(_uart_fmu_b, _ctx_fmu_b);

    // 1Hz consolidated diagnostic
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_diag_ms >= 1000) {
        // Use local evaluate_health() — reflects actual FMU communication state
        uint8_t health = evaluate_health();
        char h1 = (health & 0x01) ? 'Y' : 'N';
        char h2 = (health & 0x02) ? 'Y' : 'N';
        char h3 = (health & 0x04) ? 'Y' : 'N';

        // Line 1: identity + frame rates
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "TMR FMU%u ctrl=F%u fps=%lu/%lu/%lu",
                      _this_redundancy_num,
                      _last_ctrl_redundancy_num,
                      (unsigned long)_ctx_fpga.frame_count,
                      (unsigned long)_ctx_fmu_a.frame_count,
                      (unsigned long)_ctx_fmu_b.frame_count);

        // Line 2: local-evaluated health + switch reason
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "Health: F1=%c F2=%c F3=%c sw=%s",
                      h1, h2, h3,
                      sw_reason_str(_last_switch_reason));

        // Line 3-4: peer FMU detail (only when frames received this window)
        if (_ctx_fmu_a.frame_count > 0) {
            const RedundancyCommFrame *fa =
                reinterpret_cast<const RedundancyCommFrame *>(_ctx_fmu_a.buffer);
            uint8_t na = (fa->redundancy_health >> 6) & 0x03;
            uint8_t ha = fa->redundancy_health & 0x07;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                "Peer F%u: mode=%u arm=%u h=%c%c%c",
                na, (unsigned)fa->work_mode,
                (unsigned)fa->unlock_status,
                (ha&1)?'Y':'N', (ha&2)?'Y':'N', (ha&4)?'Y':'N');
        }
        if (_ctx_fmu_b.frame_count > 0) {
            const RedundancyCommFrame *fb =
                reinterpret_cast<const RedundancyCommFrame *>(_ctx_fmu_b.buffer);
            uint8_t nb = (fb->redundancy_health >> 6) & 0x03;
            uint8_t hb = fb->redundancy_health & 0x07;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                "Peer F%u: mode=%u arm=%u h=%c%c%c",
                nb, (unsigned)fb->work_mode,
                (unsigned)fb->unlock_status,
                (hb&1)?'Y':'N', (hb&2)?'Y':'N', (hb&4)?'Y':'N');
        }

        _ctx_fpga.frame_count = 0;
        _ctx_fmu_a.frame_count = 0;
        _ctx_fmu_b.frame_count = 0;
        _last_diag_ms = now_ms;
    }

    // 10Hz timed logic
    if (now_ms - _last_update_ms < REDUNDANCY_UPDATE_INTERVAL_MS) {
        return;
    }
    _last_update_ms = now_ms;

    // Send UART crossbar config to FPGA (startup phase only, controlling FMU only)
    if (!_uart_config_sent && is_in_control()) {
        static const uint8_t defaults[REDUNDANCY_UMAP_COUNT] = {
            REDUNDANCY_UMAP_DEFAULT_1, REDUNDANCY_UMAP_DEFAULT_2,
            REDUNDANCY_UMAP_DEFAULT_3, REDUNDANCY_UMAP_DEFAULT_4,
            REDUNDANCY_UMAP_DEFAULT_5, REDUNDANCY_UMAP_DEFAULT_6,
            REDUNDANCY_UMAP_DEFAULT_7, REDUNDANCY_UMAP_DEFAULT_8,
        };
        bool differs = false;
        for (uint8_t i = 0; i < REDUNDANCY_UMAP_COUNT; i++) {
            if ((uint8_t)_uart_map[i].get() != defaults[i]) {
                differs = true;
                break;
            }
        }

        if (differs && validate_uart_map()) {
            if (_uart_config_send_count < REDUNDANCY_CFG_SEND_COUNT &&
                (now_ms - _uart_config_last_send_ms >= REDUNDANCY_CFG_SEND_INTERVAL_MS)) {
                send_uart_config_frame();
                _uart_config_send_count++;
                _uart_config_last_send_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Redundancy: UART map config sent (%u/%u)",
                              _uart_config_send_count, REDUNDANCY_CFG_SEND_COUNT);
            }
            if (_uart_config_send_count >= REDUNDANCY_CFG_SEND_COUNT) {
                _uart_config_sent = true;
            }
        } else {
            _uart_config_sent = true;
        }
    }

    // Recompute param hash at most once every 5 seconds
    if (now_ms - _last_param_hash_compute_ms >= 5000) {
        _local_param_hash = compute_param_hash();
        _last_param_hash_compute_ms = now_ms;
    }

    // Check param consistency with peers
    if (_local_param_hash != REDUNDANCY_PARAM_HASH_NONE) {
        bool consistent = true;
        for (uint8_t i = 0; i < 3; i++) {
            if (_fmu_param_hash[i] != REDUNDANCY_PARAM_HASH_NONE &&
                _fmu_param_hash[i] != _local_param_hash) {
                consistent = false;
                break;
            }
        }
        _param_check_passed = consistent;
    }

    // Update relay bitmasks
    update_relay_state();

    // Handle arm/disarm following when we are NOT the controller
    if (_this_redundancy_num != 0 && _last_ctrl_redundancy_num != 0 &&
        _last_ctrl_redundancy_num != _this_redundancy_num) {

        // Determine the controller's frame from our peer FMU buffers
        const RedundancyCommFrame *ctrl_frame = nullptr;
        if (_last_ctrl_redundancy_num == 1) {
            // Controller is FMU1; we are FMU2 or FMU3
            if (_this_redundancy_num == 2 || _this_redundancy_num == 3) {
                ctrl_frame = reinterpret_cast<const RedundancyCommFrame *>(_ctx_fmu_a.buffer);
            }
        } else if (_last_ctrl_redundancy_num == 2) {
            if (_this_redundancy_num == 1) {
                ctrl_frame = reinterpret_cast<const RedundancyCommFrame *>(_ctx_fmu_a.buffer);
            } else if (_this_redundancy_num == 3) {
                ctrl_frame = reinterpret_cast<const RedundancyCommFrame *>(_ctx_fmu_b.buffer);
            }
        } else if (_last_ctrl_redundancy_num == 3) {
            if (_this_redundancy_num == 1 || _this_redundancy_num == 2) {
                ctrl_frame = reinterpret_cast<const RedundancyCommFrame *>(_ctx_fmu_b.buffer);
            }
        }

        if (ctrl_frame != nullptr) {
            bool ctrl_armed = (ctrl_frame->unlock_status != 0);
            if (ctrl_armed) {
                handle_follow_arm();
            } else {
                handle_follow_disarm();
            }
        }

    } else if (is_in_control()) {
        // Perform failover setup once when controller actually switches.
        // _failover_ctrl_initialized tracks whether we have seen the first
        // FPGA assignment — that initial assignment is NOT a real switch,
        // so we must skip handle_failover() for it.
        static uint8_t last_handled_ctrl_num = 0;
        if (last_handled_ctrl_num != _last_ctrl_redundancy_num) {
            if (!_failover_ctrl_initialized) {
                // First time seeing controller assignment after boot — not a real switch
                _failover_ctrl_initialized = true;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Redundancy: initial ctrl=FMU%u (no failover)", _last_ctrl_redundancy_num);
            } else {
                // Real failover event
                handle_failover();
            }
            last_handled_ctrl_num = _last_ctrl_redundancy_num;
        }
    }

    // Pack and transmit our comm frame
    pack_and_send_comm_frame();
}

// ---- Static ADC helpers ----

uint32_t AP_Redundancy::get_adc_value_24bit(const uint8_t *adc_data, uint8_t channel)
{
    if (channel >= 7) {
        return 0;
    }
    const uint8_t *p = adc_data + channel * 3;
    return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | (uint32_t)p[2];
}

float AP_Redundancy::convert_adc_to_float(uint32_t raw)
{
    if (raw >= (1U << 23)) {
        // Negative: two's complement from 24-bit
        return -(float)(((1U << 24) - raw) / (float)(1U << 17));
    }
    return (float)(raw / (float)(1U << 17));
}

#endif // ENABLE_REDUNDANCY_CONTROL
