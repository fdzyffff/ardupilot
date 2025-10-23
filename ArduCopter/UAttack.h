#pragma once

#include <AP_HAL/AP_HAL.h>
#include <FD_Target/FD_Target.h>
#include "User_shiftaverage.h"

class UAttack {

public:

    friend class Copter;
    friend class ModeAttack;

    // constructor, destructor
    UAttack();

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    void init();
    bool is_active() const { return (current_idx>0); }
    void udpate_control_value();
    void init_target();
    void update();
    void update_cam();
    void update_control();
    void update_attack_angle_target();
    const Vector2f& get_ef_gun_info();
    const Vector2f& get_ef_cam_info();

    float get_target_vel_x() {return _target_vel_x;}
    float get_target_vel_y() {return _target_vel_y;}
    float get_target_vel_z() {return _target_vel_z;}
    float get_target_angle_yaw() {return _target_angle_yaw;}

    void handle_attack_msg(const mavlink_message_t &msg);
    void handle_info(float p1, float p2);

    void update_target_vel_x();
    void update_target_vel_y();
    void update_target_vel_z();
    void update_target_angle_yaw();
    void update_log();

    void start();
    void stop();
    void reset();

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

    Vector2f ef_cam_info;
    Vector2f ef_gun_info;
    bool _active;
    float _target_vel_x;
    float _target_vel_y;
    float _target_vel_z;
    float _target_angle_yaw;


private:

    AP_Int16        print;
    AP_Int8         use_target_cam;
    AP_Int8         use_target_cam_type;
    AP_Float        filt_yaw_hz;
    AP_Float        filt_pithc_hz;
    AP_Float        gun_pitch;
    AP_Float        aim_pitch;

    // AC_PID          attack_velx_pid{0.5f, 1.0f, 0.0f, 0.0f, 1.0f, 5.0f, 5.0f, 5.0f, 0.0f};
    AC_PID          attack_velz_pid{0.5f, 1.0f, 0.0f, 0.0f, 1.0f, 5.0f, 5.0f, 5.0f, 0.0f};

    FD_Target_Base*   _Target_ptr_cam;
    FD_Target_QD*   _Target_ptr_cam_QD;

    uint32_t _last_control_ms;
    uint32_t _last_reset_ms;
    uint32_t _last_log_ms;
    bool _reset;
    bool _running;
    int8_t current_idx;

    LowPassFilterFloat _yaw_sample_filter;
    LowPassFilterFloat _pitch_sample_filter;
    LowPassFilterFloat _ef_rate_x_filter;
    LowPassFilterFloat _ef_rate_y_filter;
    LowPassFilterFloat _delta_yaw_filter;

    User_shiftaverage _pitch_filter;
    User_shiftaverage _yaw_filter;
};
