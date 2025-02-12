#pragma once

#include <AP_HAL/AP_HAL.h>
#include <FD_CAM/FD_CAM.h>
class UCam_base;

class UAttack {

public:

    // constructor, destructor
    UAttack();

    void init();
    bool is_active() const { return _active; }
    void udpate_control_value();
    void init_cam_port();
    void update();
    void do_cmd_on(bool on);
    const Vector2f& get_bf_info();
    const Vector2f& get_ef_info();
    const Vector2f& get_ef_rate_info();

    float get_target_pitch_rate() {return _target_pitch_rate;}
    float get_target_roll_angle() {return _target_roll_angle;}
    float get_target_yaw_rate() {return _target_yaw_rate;}

    void handle_info_test(float p1, float p2);

    AP_HAL::UARTDriver* get_port(void) {return _cam_port;}

    void update_log();

    bool display_info_new;
    float display_info_p1;
    float display_info_p2;
    float display_info_p3;
    float display_info_p4;
    float display_info_p11;
    float display_info_p12;
    float display_info_p13;
    float display_info_p14;
    uint8_t display_info_count;
    uint8_t display_info_count_log;

    Vector2f bf_info;
    Vector2f ef_info;
    Vector2f ef_rate_info;
    bool _active;
    AP_HAL::UARTDriver* _cam_port;
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

    UCam_base* _UCam_ptr;
    uint8_t _cam_port_type;

private:
    void cam_update();
    void time_out_check();
    void update_target_pitch_rate();
    void update_target_roll_angle();
    void update_target_yaw_rate();
    void update_target_throttle();
    void update_target_bf_to_ef();

    uint32_t _last_ms;
};

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class UCam_base {
public:
    friend class UAttack;

    UCam_base(UAttack &frotend_in): _frotend(frotend_in) {};
    virtual bool is_valid() = 0;
    virtual void update() = 0;
    virtual void do_cmd_on(bool on) = 0;
    virtual void handle_info(float p1, float p2) = 0;
    virtual void handle_msg(const mavlink_message_t &msg) = 0;
    virtual void handle_info_test(float p1, float p2) = 0;
    UAttack &_frotend;
    bool _valid;
};

class UCam: public UCam_base {
public:
    friend class UAttack;

    UCam(UAttack &frotend_in, AP_HAL::UARTDriver* port_in);
    bool is_valid() override;
    void update() override;
    void do_cmd_on(bool on) override;
    void do_cmd_pre_lock();
    void handle_info(float p1, float p2) override;
    void handle_msg(const mavlink_message_t &msg) override {return;};
    void handle_info_test(float p1, float p2) override;

    float cal_frame_angle(float pixel, float angle, float x_in);
private:
    FD_CAM* FD_CAM_ptr;
    uint32_t _last_ms;

    DerivativeFilterFloat_Size7 _pitch_filter;
    DerivativeFilterFloat_Size7 _yaw_filter;
    LowPassFilterFloat _yaw_earth_rate_filter;
    LowPassFilterFloat _yaw_rate_filter;
    LowPassFilterFloat _pitch_rate_filter;
};


