#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters() {}
    static const struct AP_Param::GroupInfo var_info[];
    
    // Put your parameter variable definitions here
    AP_Int8 _print;
    AP_Float _spin_yaw;
    AP_Int16 _target_rpm1;
    AP_Int16 _target_rpm2;

};
