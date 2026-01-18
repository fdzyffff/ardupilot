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

#include <FD_Uart/FD_Uart.h>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class FD_Target_Base {
public:
    FD_Target_Base() {};
    virtual ~FD_Target_Base() {};
    virtual bool init() {return false;}
    virtual void update() = 0;
    bool is_valid() {return _valid;}
    void handle_info(float p1, float p2);
    bool get_info(float &p1, float &p2);
    uint8_t get_type();
    void set_type(uint8_t type_in);
    virtual void handle_msg(const mavlink_message_t &msg);
    uint32_t _last_ms;
    bool _new_data;
    bool _valid;
    float _p1;
    float _p2;
    uint8_t _type;
};

class FD_Target_Loc: public FD_Target_Base {
public:
    FD_Target_Loc();
    ~FD_Target_Loc() {};

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

class FD_Target_FP847: public FD_Target_Base {
public:
    FD_Target_FP847();
    ~FD_Target_FP847() {};

    static const struct AP_Param::GroupInfo var_info[];

    bool init() override;
    void update() override;
    void handle_msg(const mavlink_message_t &msg) override;
    float cal_frame_angle(float pixel, float angle, float x_in);
    void handle_info_test(float p1, float p2);

private:
    AP_Int32 target_timeout;
    AP_Float cam_width;
    AP_Float cam_height;
    AP_Float cam_angle_x;
    AP_Float cam_angle_y;
};

using AP_HAL::millis;
