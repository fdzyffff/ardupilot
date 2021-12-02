#pragma once

#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>

class UserParameters {

public:
    UserParameters() :
        Ucam_pid(1.0f, 0.0f, 0.0f, 0.0f, 0.2f, 3.0f, 3.0f, 3.0f, 0.02f),
        Thr_pid(1.0f, 0.0f, 0.0f, 0.0f, 0.2f, 3.0f, 3.0f, 3.0f, 0.02f)
        {}
    static const struct AP_Param::GroupInfo var_info[];
    AP_Float fly_attack_angle;
    AC_PID   Ucam_pid;
    AC_PID   Thr_pid;

    AP_Float cam_angle_x;
    AP_Float cam_angle_y;
    AP_Float cam_pixel_x;
    AP_Float cam_pixel_y;
    AP_Float cam_pitch_offset;
    AP_Int32 cam_time_out;
    AP_Int32 cam_print;
    AP_Float cam_target_x;
    AP_Float cam_target_y;

    AP_Float fly_yaw_tc;
    AP_Float fly_pitch_limit;
    AP_Float fly_roll_limit;
    AP_Float fly_roll_factor;
    AP_Float fly_pitch_scalar;
    AP_Float fly_climb_factor;

    AP_Int8  uart_cam_test;
};
