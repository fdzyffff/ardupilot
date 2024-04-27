#include "UserParameters.h"

// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_FWDTYPE", 0, UserParameters, forward_type, 0),
    AP_GROUPINFO("_FWDPRINT", 1, UserParameters, forward_print, 0),
    AP_GROUPINFO("_TIMEOUT", 10, UserParameters, cam_time_out, 2000),
    AP_SUBGROUPINFO(Ucam_pid, "_PITCH_", 11, UserParameters, AC_PID),
    AP_GROUPINFO("_PRINT", 12, UserParameters, cam_print, 0),
    AP_GROUPINFO("_ANGLE", 13, UserParameters, cam_angle, 60.0f),

    AP_GROUPEND
};

UserParameters::UserParameters():
    Ucam_pid(0.1f, 0.0f, 0.0f, 0.0f, 0.2f, 3.0f, 3.0f, 3.0f, 0.02f)
{
    AP_Param::setup_object_defaults(this, var_info);
}
