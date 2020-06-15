#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters() {}
    static const struct AP_Param::GroupInfo var_info[];
    
    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()

    
//private:
    // Put your parameter variable definitions here

    AP_Float cam1_angle;
    AP_Float cam1_xlength;
    AP_Float luoche_land_speed;
    AP_Float luoche_xy_speed;
    AP_Int8 luoche_info_level;
    AP_Int16 luoche_cam1_timeout;
    AP_Int8 vel_channel;
    AP_Float luoche_follow_alt;
    AP_Float luoche_final_alt;
    AP_Float luoche_vel_factor;
};
