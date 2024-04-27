#pragma once

#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];
    
    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    AP_Int8 forward_type;
    AP_Int8 forward_print;
    AP_Int16 cam_time_out;
    AC_PID   Ucam_pid;
    AP_Int8  cam_print;
    AP_Float attack_pitch;
    AP_Float attack_roll_factor;

private:
    // Put your parameter variable definitions here
};
