#pragma once

#include <AP_HAL/AP_HAL.h>
#include <FD_Target/FD_Target.h>
#include "UDelay.h"

class UAttack {

public:

    friend class Plane;
    friend class ModeAttackCam;
    friend class ModeAttackLoc;
    friend class Uart;

    // constructor, destructor
    UAttack();

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    void init();
    bool is_active() const { return (current_idx>0); }
    bool is_active_loc() const { return (current_idx == 1); }
    bool is_active_cam() const { return (current_idx == 2); }
    void update_control_value();
    void update_vel_bf_info();
    void init_target();
    void update();
    const Vector2f& get_bf_info();
    const Vector2f& get_ef_info();
    const Vector2f& get_ef_rate_info();
    const Vector2f& get_bfe_info();

    float get_target_pitch_rate() {return _target_pitch_rate;}
    float get_target_roll_angle() {return _target_roll_angle;}
    float get_target_yaw_rate() {return _target_yaw_rate;}

    void handle_attack_msg(const mavlink_message_t &msg);
    void handle_info(float p1, float p2);

    void update_target_pitch_rate();
    void update_target_roll_angle();
    void update_target_yaw_rate();
    void update_log();
    void do_print();

    void update_target_loc();
    void set_target_loc(Location& loc_in); 
    bool have_target_loc();
    Location get_target_loc() {return _target_loc;}

    struct {
        float p1;
        float p2;
        float p3;
        float p4;
        float p11;
        float p12;
        float p13;
        float p14;
        float p21;
        float p22;
        float p23;
        float p24;
        bool new_data;
        uint16_t count;
        uint16_t count_log;
    } display_info;

    Vector2f bf_info;
    Vector2f bfe_info; // in body frame without pitch and roll
    Vector2f vel_bf_info;
    Vector2f ef_info;
    Vector2f ef_rate_info;
    bool _active;
    float _target_pitch_rate;
    float _target_roll_angle;
    float _target_yaw_rate;
    float _attack_angle_target;
    float _attack_angle_measure;
    float _attack_angle_rate_target;
    float _attack_angle_rate_measure;

    float _delta_course;

    UDelay udelay;

private:

    AP_Float        attack_k2_pitch;
    AP_Float        attack_k3_pitch;
    AP_Float        attack_k1_yaw;
    AP_Float        attack_k2_yaw;
    AP_Float        attack_k3_yaw;
    AP_Float        attack_k2_roll;
    AP_Float        attack_k_angle;
    AP_Float        attack_throttle;
    AP_Int32        atk_time_out;
    AP_Float        attack_angle;
    AP_Float        pitch_limit;
    AP_Float        pitch_rate_limit;
    AP_Float        attack_pitch_off;
    AP_Int16        print;
    AP_Int8         use_target_cam;
    AP_Int8         use_target_loc;
    AP_Int8         use_target_cam_type;
    AP_Float        filt_yaw_hz;
    AP_Float        filt_pithc_hz;
    AC_PID          attack_roll_pid{0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f, 5.0f, 5.0f, 0.0f};
    AC_PID          attack_pitch_pid{0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f, 5.0f, 5.0f, 0.0f};

    FD_Target_Base*       _Target_ptr_cam;
    FD_Target_Loc*        _Target_ptr_loc;
    FD_Target_DYT*        _Target_ptr_cam_DYT;

    uint32_t _last_ms;
    int8_t current_idx;

    DerivativeFilterFloat_Size7 _pitch_filter;
    DerivativeFilterFloat_Size7 _yaw_filter;
    LowPassFilterConstDtFloat _yaw_sample_filter;
    LowPassFilterConstDtFloat _pitch_sample_filter;
    float _last_yaw;
    float _last_yaw_sample;

    LowPassFilterVector3f _target_pos{1.0};
    uint32_t _last_target_update_ms;
    Location _target_loc;
};
