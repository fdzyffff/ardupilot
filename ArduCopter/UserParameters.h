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
    
    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
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

    AP_Int32 gcs_time_out;
    AP_Int32 gcs_print;
    AP_Int16 gcs_num_cruise;
    AP_Int32 gcs_time_cruise;
    AP_Float gcs_group_yaw;
    AP_Int16 gcs_group_relative_yaw;
    AP_Float gcs_search_yangle_left;
    AP_Float gcs_search_yangle_right;
    AP_Float gcs_search_yrate;
    AP_Int16 gcs_target_alt;
    AP_Float gcs_group_delay;

    AP_Float fly_yaw_tc;
    AP_Float fly_pitch_limit;
    AP_Float fly_pitch_rate;
    AP_Float fly_roll_limit;
    AP_Float fly_roll_factor;
    AP_Float fly_pitch_scalar;
    AP_Float fly_climb_factor;
    AP_Float fly_climb_rate_offset;

    AP_Int16 tele_power;

    AP_Int32 ekf_origin_latitude;
    AP_Int32 ekf_origin_longitude;
    AP_Int32 ekf_origin_alt;

    AP_Float group_search_dist;

    AP_Float atk_thr_up_factor;

};
