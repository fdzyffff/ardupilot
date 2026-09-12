#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <Filter/Filter.h>
#include <FD_Target/FD_Target_HY.h>
#include <FD_Target/FD_Target_Loc.h>

#include "UDelay.h"

class UAttack {
public:
    UAttack();

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();
    void do_print();
    void handle_attack_msg(const mavlink_message_t &msg);

    bool is_active() const { return _target_source != TargetSource::NONE; }
    bool is_active_loc() const { return _target_source == TargetSource::LOC; }
    bool is_active_cam() const { return _target_source == TargetSource::CAM; }

    const Vector2f &get_los_c_deg() const { return _los_c_deg; }
    const Vector2f &get_los_e_deg() const { return _los_e_deg; }
    const Vector3f &get_los_c_rate_dps() const { return _los_c_rate_dps; }
    const Vector3f &get_target_rate_c_dps() const { return _target_rate_c_dps; }
    const Vector3f &get_target_rate_b_dps() const { return _target_rate_b_dps; }
    float get_throttle() const { return throttle.get(); }
    float get_throttle_rate() const { return throttle_rate.get(); }
    Location get_target_loc();
    void set_target_loc(Location &loc_in);
    // 滞后补偿使能（锁架期由 ModeAttack 关闭；发射锁存后/退出模式时恢复）。
    void set_lag_offset_enabled(bool en);
    float get_ff_atk() const { return ff_atk.get(); }

private:
    enum class TargetSource : uint8_t {
        NONE = 0,
        LOC = 1,
        CAM = 2
    };

    enum class ControlStage : uint8_t {
        NONE = 0,
        PITCH_CAPTURE = 1,
        ANGLE_CAPTURE = 2,
        LOS_RATE = 3
    };

    void init_targets();
    void update_delay_history();
    bool get_delayed_state(Vector3f &attitude_b_deg, Vector3f &gyro_b_dps) const;

    void process_observation(const Vector2f &los_c_deg,
                             const Vector3f &attitude_b_deg,
                             const Vector3f &gyro_b_dps);
    void update_control_value(const Vector3f &attitude_b_deg, float observation_dt_s);
    void apply_target_visibility_limit();
    void clear_target_output();
    void update_log();

    AP_Int8 use_target_cam;
    AP_Int8 use_target_loc;
    AP_Int16 camera_delay_ms;
    AP_Float los_vector_filt_hz;
    AP_Float yaw_angle_gain;
    AP_Float pitch_angle_gain;
    AP_Float rate_limit_dps;
    AP_Float visibility_limit_deg;
    AC_PID los_yaw_rate_pid{3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f};
    AC_PID los_pitch_rate_pid{3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f};
    AP_Int16 debug_print;
    AP_Float throttle;
    AP_Float throttle_rate;
    AP_Float roll_level_gain;
    AP_Float forward_pitch_deg;
    AP_Int8 forward_pitch_en;
    AP_Float track_yaw_gain;
    AP_Float track_pitch_gain;
    AP_Float lag_pitch_half_life_s;
    AP_Float lag_pitch_max_deg;
    AP_Float lag_yaw_half_life_s;
    AP_Float lag_yaw_max_deg;
    AP_Int8  lag_bc_en;         // 滞后补偿反激励开关（1=扣除自身角速度分量，防自激震荡）
    AP_Float ff_atk;            // 拦截离架档 pitch 速率环 FF 前馈增益（平时恒 0，须保持 ATC_RAT_PIT_FF=0）

    FD_Target_HY target_cam;
    FD_Target_Loc target_loc;
    bool target_cam_initialized;
    bool target_loc_initialized;
    TargetSource _target_source;
    ControlStage _control_stage;

    UDelay attitude_delay;
    UDelay gyro_delay;

    Matrix3f rotation_body_to_camera;
    Matrix3f rotation_camera_to_body;
    Vector2f _los_c_deg;
    Vector2f _los_e_deg;
    Vector3f _los_e_unit;
    Vector3f _los_e_dot;
    Vector3f _los_e_rate_dps;
    Vector3f _camera_e_deg;
    Vector3f _camera_log_e_deg;
    Vector2f _velocity_e_deg;
    float _velocity_speed_ms;
    bool _velocity_valid;
    Vector3f _gyro_c_dps;
    Vector3f _los_c_rate_dps;
    Vector3f _target_rate_c_dps;
    Vector3f _target_rate_b_dps;

    DerivativeFilterFloat_Size7 los_e_x_derivative;
    DerivativeFilterFloat_Size7 los_e_y_derivative;
    DerivativeFilterFloat_Size7 los_e_z_derivative;
    LowPassFilterConstDtFloat los_e_x_filter;
    LowPassFilterConstDtFloat los_e_y_filter;
    LowPassFilterConstDtFloat los_e_z_filter;

    bool have_last_target;
    bool los_rate_control_active;
    bool _enable_lag_offset = true;   // 滞后补偿估计使能（锁架期禁用，见 set_lag_offset_enabled）
    float _lag_offset_pitch_deg;
    float _lag_offset_yaw_deg;
    Vector3f _guid_rate_c_dps;             // 制导基础指令（不含滞后补偿分量），滞后估计器积分输入
    uint32_t last_observation_ms;
    uint32_t last_filter_config_ms;
    uint32_t last_log_ms;
    uint32_t update_call_count;
    uint32_t update_rate_last_ms;
};


