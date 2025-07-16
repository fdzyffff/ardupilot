#pragma once

#include <AP_HAL/AP_HAL.h>
#include <FD_Target/FD_Target.h>

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
    bool is_active_loc() const { return (current_idx == 2); }
    void udpate_control_value();
    void init_target();
    void update();
    const Vector2f& get_bf_info();
    const Vector2f& get_ef_info();
    const Vector2f& get_ef_rate_info();

    float get_target_pitch_rate() {return _target_pitch_rate;}
    float get_target_roll_angle() {return _target_roll_angle;}
    float get_target_yaw_rate() {return _target_yaw_rate;}

    void handle_attack_msg(const mavlink_message_t &msg);
    void handle_info(float p1, float p2);

    void update_target_pitch_rate();
    void update_target_roll_angle();
    void update_target_yaw_rate();
    void update_target_throttle();
    void update_log();

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
    float _attack_throttle;

    float _attack_throttle_pid;
    float _attack_throttle_p;
    float _attack_throttle_i;
    float _attack_throttle_d;

private:

    AP_Float        attack_k1_pitch;
    AP_Float        attack_k2_pitch;
    AP_Float        attack_k1_yaw;
    AP_Float        attack_k2_yaw;
    AP_Float        attack_k2_roll;
    AP_Float        attack_k_angle;
    AP_Float        attack_throttle;
    AP_Float        attack_throttle_rate;
    AP_Int16        attack_timeout;
    AP_Float        attack_angle;
    AP_Float        pitch_limit;
    AP_Float        pitch_rate_limit;
    AP_Float        attack_pitch_off;
    AP_Int16        print;
    AP_Int8         use_target_cam;
    AP_Int8         use_target_cam_type;
    AP_Float        filt_yaw_hz;
    AP_Float        filt_pithc_hz;

    AC_PID          attack_roll_pid{0.5f, 0.1f, 0.01f, 0.0f, 1.0f, 5.0f, 5.0f, 5.0f, 0.5f};
    AC_PID          attack_throttle_pid{0.5f, 0.03f, 0.01f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.5f};

    FD_Target_Base*   _Target_ptr_cam;
    FD_Target_Mav*    _Target_ptr_cam_mav;

    uint32_t _last_ms;
    uint32_t _last_reset_ms;
    bool _reset;
    int8_t current_idx;

    DerivativeFilterFloat_Size7 _pitch_filter;
    DerivativeFilterFloat_Size7 _yaw_filter;
    LowPassFilterFloat _yaw_sample_filter;
    LowPassFilterFloat _pitch_sample_filter;
    float _last_yaw;
    float _last_yaw_sample;


    #define UDELAY_BUFFER 100
    class UDelay {
    public:
        UDelay() {;};
        
        void init();
        void push();
        bool get_idx(uint16_t step, float &roll, float &pitch, float &yaw);

    private:
        struct {
            float roll;
            float pitch;
            float yaw;
            uint32_t time_ms;
        } _buffer[UDELAY_BUFFER];
        uint16_t _idx;
    };

    UDelay udelay;

};
