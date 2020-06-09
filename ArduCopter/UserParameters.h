#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters() {}
    static const struct AP_Param::GroupInfo var_info[];
    
    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    AP_Int8 get_int8Param() const { return _int8; }
    AP_Int16 get_int16Param() const { return _int16; }
    AP_Float get_floatParam() const { return _float; }
    
//private:
    // Put your parameter variable definitions here
    AP_Int8 _int8;
    AP_Int16 _int16;
    AP_Float _float;
    AP_Float cam1_angle;
    AP_Float cam1_xlength;
    AP_Float cam2_angle;
    AP_Float cam2_xlength;
    AP_Float genren_follow_vel;
    AP_Float genren_follow_default_yawrate;
    AP_Int8 genren_info_level;
    AP_Int16 genren_cam1_timeout;
    AP_Int16 genren_cam2_timeout;
};
