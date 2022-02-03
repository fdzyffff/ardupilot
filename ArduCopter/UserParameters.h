#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters() {}
    static const struct AP_Param::GroupInfo var_info[];
    
    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    AP_Float EF3_target_alt;
    AP_Float EF3_yaw_factor;
    AP_Float EF2_alt_max;
    AP_Float EF2_alt_min;
    AP_Int8  EF_use_uwb_port;
    AP_Float EF_football_dir;
    
private:
    // Put your parameter variable definitions here
};
