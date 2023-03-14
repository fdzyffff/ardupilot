#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters() {}
    static const struct AP_Param::GroupInfo var_info[];
    
    // Put your parameter variable definitions here
    AP_Int16 netgun_max; //look down is 9000
    AP_Int16 netgun_min; //look at skyline is 0
};
