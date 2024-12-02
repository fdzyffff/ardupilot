#pragma once

#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];
    
    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    AP_Float attack_k;
    AP_Float attack_k2;
    AP_Float attack_throttle;
    AP_Float attack_throttle_rate;
    AP_Int16 attack_timeout;
    AP_Float attack_angle; // degree
    AP_Float pitch_limit;
    AP_Float pitch_rate_limit;
    AP_Int16 print;
    AC_PID   attack_throttle_pid;
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
    AP_Float attack_angle_kp;
    AP_Float attack_roll_factor;
    AP_Int16 lock_y_down;


private:
    // Put your parameter variable definitions here
};
