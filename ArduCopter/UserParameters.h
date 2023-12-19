#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters() {}
    static const struct AP_Param::GroupInfo var_info[];

    // Put your parameter variable definitions here
    AP_Float user_pitch_range;
    AP_Float user_pitch_rate;
};
