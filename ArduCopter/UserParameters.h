#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int16 cam_time_out;
    AP_Int8  cam_print;
};
