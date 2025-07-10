#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>

#include <FD_Gimbal_Uart/FD_HaoFu.h>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class FD_Gimbal_Base {
public:
    FD_Gimbal_Base() {};
    virtual ~FD_Gimbal_Base() {};
    virtual bool init();
    virtual void update() = 0;
    bool is_valid() {return _valid;}
    void handle_info(float p1, float p2);
    bool get_info(float &p1, float &p2);
    void handle_raw_info(float p1, float p2);
    bool get_raw_info(float &p1, float &p2);
    virtual void handle_msg(const mavlink_message_t &msg);
    virtual void do_rate_control(float pitch_rate, float yaw_rate);
    virtual void get_attitude_euler(float& gimbal_roll, float& gimbal_pitch, float& gimbal_yaw);
    virtual bool have_target();

    uint32_t _last_ms;
    bool _new_data;
    bool _valid;
    float _p1;
    float _p2;
    float _raw_p1;
    float _raw_p2;

    float _gimbal_roll;
    float _gimbal_pitch;
    float _gimbal_yaw;
};

class FD_Gimbal_Loc: public FD_Gimbal_Base {
public:
    FD_Gimbal_Loc();
    ~FD_Gimbal_Loc() {};

    static const struct AP_Param::GroupInfo var_info[];

    bool init() override;
    void update() override;
    void handle_msg(const mavlink_message_t &msg) override;
    void handle_info_test(float p1, float p2);

    Location current_loc;
    Location target_loc;

    AP_Int32 target_timeout;
    AP_Float nav_radius;

private:
    bool _have_target;
};

class FD_Gimbal_HaoFu: public FD_Gimbal_Base {
public:
    FD_Gimbal_HaoFu();
    ~FD_Gimbal_HaoFu() {};

    static const struct AP_Param::GroupInfo var_info[];

    bool init() override;
    void update() override;
    void handle_msg(const mavlink_message_t &msg) override;
    float cal_frame_angle(float pixel, float angle, float x_in);
    void handle_info_test(float p1, float p2);
    void update_uart();

private:
    AP_Int32 target_timeout;
    AP_Float cam_width;
    AP_Float cam_height;
    AP_Float cam_angle_x;
    AP_Float cam_angle_y;

    FD_HaoFu* FD_HaoFu_ptr;
};


using AP_HAL::millis;
