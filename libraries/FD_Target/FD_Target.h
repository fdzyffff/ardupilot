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

#include <FD_LRB/FD_LRB.h>
#include <FD_K230/FD_K230.h>
#include <FD_RK3588/FD_RK3588.h>

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

class FD_Target_K230: public FD_Target_Base {
public:
    FD_Target_K230();
    ~FD_Target_K230() {};

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

    FD_K230* FD_K230_ptr;
};

class FD_Target_LRB: public FD_Target_Base {
public:
    FD_Target_LRB();
    ~FD_Target_LRB() {};
    bool init() override;
    void update() override;
    void do_cmd_on(bool on);
    void do_cmd_pre_lock();
    void handle_info_test(float p1, float p2);
    float cal_frame_angle(float pixel, float angle, float x_in);

private:
    AP_Int32 target_timeout;
    AP_Float cam_width;
    AP_Float cam_height;
    AP_Float cam_angle_x;
    AP_Float cam_angle_y;
    AP_Float cam_pitch_offset;
    AP_Int16 lock_x;
    AP_Int16 lock_y;
    AP_Int8  lock_size;
    AP_Int16 cam_x_offset;
    AP_Int16 cam_y_offset;
    AP_Int16 lock_y_down;

    FD_LRB* FD_LRB_ptr;
};

class FD_Target_RK3588: public FD_Target_Base {
public:
    FD_Target_RK3588();
    ~FD_Target_RK3588() {};

    static const struct AP_Param::GroupInfo var_info[];

    bool init() override;
    void update() override;
    void handle_msg(const mavlink_message_t &msg) override;
    float cal_frame_angle(float angle, float x_in);
    void handle_info_test(float p1, float p2);

private:
    AP_Int32 target_timeout;
    AP_Float cam_angle_x;
    AP_Float cam_angle_y;

    FD_RK3588* FD_RK3588_ptr;
};

class FD_Target_Mav: public FD_Target_Base {
public:
    FD_Target_Mav();
    ~FD_Target_Mav() {};
    bool init() override;
    void update() override;
    void handle_msg(const mavlink_message_t &msg) override;
    void handle_info_test(float p1, float p2);

private:
    AP_Int32 target_timeout;

};

using AP_HAL::millis;
