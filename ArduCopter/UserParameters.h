#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int8 role;
    AP_Float detection_R;
    AP_Float connection_R;
};
