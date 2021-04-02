#pragma once

#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_PI_2D.h>

class UserParameters {

public:
    UserParameters() {}
    static const struct AP_Param::GroupInfo var_info[];

    // Put your parameter variable definitions here
    AP_Int16 usr_hil_mode;
    AP_Int16 usr_hil_test;
    AP_Int16 usr_print;

    AC_PI_2D myvel_pi_xy{0.2f, 0.3f, 3000, 5, 0.0025f};
    AP_Float myvel_filter_hz;

private:
};
