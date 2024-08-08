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
    AP_GROUPINFO("_ATK_PITCH", 13, UserParameters, attack_pitch, -1000.0f),
    AP_GROUPINFO("_ATK_ROLLF", 14, UserParameters, attack_roll_factor, 1.0f),
    AP_GROUPINFO("_OFFYAW", 15, UserParameters, attack_yaw_offset, 1.0f),
    AP_GROUPINFO("_OFFPTH", 16, UserParameters, attack_pitch_offset, 1.0f),
    AP_GROUPINFO("_ATK_VEL", 17, UserParameters, attack_vel, 500.0f),
    AP_GROUPINFO("_MAX_VEL_XY", 18, UserParameters, max_vel_xy, 600.0f),
    AP_GROUPINFO("_MAX_VEL_Z", 19, UserParameters, max_vel_z, 300.0f),

    AP_GROUPEND
};

UserParameters::UserParameters():
    Ucam_pid(0.75f, 0.0f, 0.0f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f, 0.02f)
{
    AP_Param::setup_object_defaults(this, var_info);
}
