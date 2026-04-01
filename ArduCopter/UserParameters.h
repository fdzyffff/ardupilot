#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int16 cam_time_out;
    AP_Int8  cam_print;
    AP_Float attack_k;
    AP_Float attack_k2;
    AP_Float tag_scale_f_big;
    AP_Float tag_scale_f_small;
    AP_Float tag_scale_u_big;
    AP_Float tag_scale_u_small;
    AP_Float cam_angle_x;
    AP_Float cam_angle_y;
    AP_Float assit_gain;
    AP_Float filt_hz;
    AP_Float cam_roll_off;
    AP_Float cam_pitch_off;
    AP_Float hook_mission_alt;
};
