#pragma once

#include <AP_HAL/AP_HAL.h>
#include "UTarget.h"

class UTarget_Base;

class UAttack {

public:

    // constructor, destructor
    UAttack();

    void init();
    bool is_active() const { return _active; }
    void udpate_control_value();
    void init_target();
    void delete_target();
    void update();
    const Vector2f& get_bf_info();
    const Vector2f& get_ef_info();
    const Vector2f& get_ef_rate_info();

    float get_target_pitch_rate() {return _target_pitch_rate;}
    float get_target_roll_angle() {return _target_roll_angle;}
    float get_target_yaw_rate() {return _target_yaw_rate;}

    void handle_attack_msg(const mavlink_message_t &msg);

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

    UTarget_Base* _UTarget_ptr;
    uint8_t _target_type;


private:
    void target_update();
    void time_out_check();
    void update_target_pitch_rate();
    void update_target_roll_angle();
    void update_target_yaw_rate();
    void update_log();

    uint32_t _last_ms;
};
