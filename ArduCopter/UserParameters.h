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
    AP_Float cam_width;
    AP_Float cam_height;
    AP_Float cam_angle_x;
    AP_Float cam_angle_y;
    AP_Float angle_limit;
    AP_Float rate_limit;
    AP_Int32 loc_A_lat;
    AP_Int32 loc_A_lng;
    AP_Int32 loc_A_alt;
    AP_Float loc_A_yaw;
    AP_Int32 loc_B_lat;
    AP_Int32 loc_B_lng;
    AP_Int32 loc_B_alt;
    AP_Float loc_B_yaw;
};
