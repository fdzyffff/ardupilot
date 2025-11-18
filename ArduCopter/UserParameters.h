#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int8 set_origin;
    AP_Float origin_pos_off_x;
    AP_Float origin_pos_off_y;
    AP_Float origin_pos_off_z;
};
