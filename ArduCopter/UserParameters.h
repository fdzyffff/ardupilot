#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    // Put your parameter variable definitions here
    AP_Int8 arm_mode;
    AP_Int8 fence_mode;
    AP_Float assit_gain;
};
