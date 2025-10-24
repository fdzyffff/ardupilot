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

#include <FD_Target_Uart/FD_QD.h>

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
    virtual void handle_msg(const mavlink_message_t &msg);
    uint32_t _last_ms;
    bool _new_data;
    bool _valid;
    float _p1;
    float _p2;
};

class FD_Target_QD: public FD_Target_Base {
public:
    FD_Target_QD();
    ~FD_Target_QD() {};

    static const struct AP_Param::GroupInfo var_info[];

    bool init() override;
    void update() override;
    void handle_msg(const mavlink_message_t &msg) override;
    float cal_frame_angle(float pixel, float angle, float x_in);
    float cal_frame_angle_left_up(float pixel, float angle, float x_in);
    void handle_info_test(float p1, float p2);

private:
    AP_Int32 target_timeout;
    AP_Float cam_width;
    AP_Float cam_height;
    AP_Float cam_angle_x;
    AP_Float cam_angle_y;
    AP_Int8  cam_use_xy;
    AP_Int8  cam_debug;

    FD_QD* FD_QD_ptr;
};


using AP_HAL::millis;
