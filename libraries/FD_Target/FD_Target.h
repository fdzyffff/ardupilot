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
    void recover_info();
    uint8_t get_type();
    void set_type(uint8_t type_in);
    void set_valid(bool valid_in);
    virtual void handle_msg(const mavlink_message_t &msg);
    virtual Location& get_target_loc();
    uint32_t _last_ms;
    bool _new_data;
    bool _valid;
    float _p1;
    float _p2;
    uint8_t _type;
    Location _target_loc;
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
    void set_target_loc(Location &loc_in);

    Location& get_target_loc() override;

    Location _current_loc;

    AP_Int32 target_timeout;
    AP_Float target_distout;
    AP_Float nav_radius;
    AP_Int8 use_external_loc;

private:
    uint32_t _last_target_ms;
    uint32_t last_update_ms;
};

class FD_Target_DYT: public FD_Target_Base {
public:
    FD_Target_DYT();
    ~FD_Target_DYT() {};

    static const struct AP_Param::GroupInfo var_info[];

    bool init() override;
    void update() override;
    void handle_msg(const mavlink_message_t &msg) override;
    float cal_frame_angle(float pixel, float angle, float x_in);
    void handle_info_test(float p1, float p2);
    AP_HAL::UARTDriver* get_port(void) {return _port;}

    FD1_msg_DYT_control uart_msg_DYT_control; 
    FD1_msg_DYT_telem uart_msg_DYT_telem; 

private:
    AP_Int32 target_timeout;
    AP_Float cam_width;
    AP_Float cam_height;
    AP_Float cam_angle_x;
    AP_Float cam_angle_y;

    AP_HAL::UARTDriver* _port;
    // FD1_msg_DYT_apminfo uart_msg_DYT_apminfo;

    uint32_t last_update_ms;
    uint32_t last_center_ms;
    uint32_t last_track_ms; 
    uint32_t last_cancel_ms;
};

class FD_Target_External: public FD_Target_Base {
public:
    FD_Target_External();
    ~FD_Target_External() {};

    static const struct AP_Param::GroupInfo var_info[];

    bool init() override;
    void update() override;
    void handle_msg(const mavlink_message_t &msg) override;
    void handle_info_test(float p1, float p2);
    AP_HAL::UARTDriver* get_port(void) {return _port;}

    float get_target_speed() {return _target_speed;}
    float get_target_pitch() {return _target_pitch;}
    float get_target_roll() {return _target_roll;}
    
    void set_target_angle(float gimbal_yaw, float gimbal_pitch);
    void set_target_loc(Location& loc_in);
    void pack_status();

    FD1_msg_LS_control uart_msg_LS_control;
    FD1_msg_LS_status uart_msg_LS_status;

private:
    AP_Int32 target_timeout;

    AP_HAL::UARTDriver* _port;
    // FD1_msg_DYT_apminfo uart_msg_DYT_apminfo;

    uint32_t last_update_ms;
    float _gimbal_yaw;
    float _gimbal_pitch;
    float _target_speed;
    float _target_pitch;
    float _target_roll;
};

using AP_HAL::millis;
