#pragma once

#include <AP_Param/AP_Param.h>
#include <AC_AttitudeControl/AC_PosControl.h>                   // Position control library

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    AP_Float filt_gyro_hz;
    AP_Float filt_acc_hz;
    AC_PI_2D assit_pi_xy{0.2f, 0.3f, 3000, 5, 0.0025f};
};
