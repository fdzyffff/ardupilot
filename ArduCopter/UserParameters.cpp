#include "UserParameters.h"

// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_UATK_K",     0, UserParameters, attack_k,                2.0f),
    AP_GROUPINFO("_UATK_K2",    1, UserParameters, attack_k2,               2.0f),
    AP_GROUPINFO("_UATK_THR",   2, UserParameters, attack_throttle,        75.0f),
    AP_GROUPINFO("_UATK_TRATE", 3, UserParameters, attack_throttle_rate,    1.0f),
    AP_GROUPINFO("_UATK_OUTMS", 4, UserParameters, attack_timeout,       2000),
    AP_GROUPINFO("_UATK_PL",    5, UserParameters, pitch_limit,            30.f),
    AP_GROUPINFO("_UATK_PRL",   6, UserParameters, pitch_rate_limit,       30.f),
    AP_GROUPINFO("_UPRINT",     7, UserParameters, print,                   0),
    AP_SUBGROUPINFO(attack_throttle_pid, "_ATKTHR_", 8, UserParameters, AC_PID),
    AP_GROUPINFO("_CAM_W",      9, UserParameters, cam_width,             720),
    AP_GROUPINFO("_CAM_H",     10, UserParameters, cam_height,            720),
    AP_GROUPINFO("_CAM_ANG_X", 11, UserParameters, cam_angle_x,           45.0f),
    AP_GROUPINFO("_CAM_ANG_Y", 12, UserParameters, cam_angle_y,           45.0f),
    AP_GROUPINFO("_CAM_P_OFF", 13, UserParameters, cam_pitch_offset,      25.0f),

    AP_GROUPEND
};

UserParameters::UserParameters():
    attack_throttle_pid(0.75f, 0.0f, 0.0f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f, 0.02f)
{
    AP_Param::setup_object_defaults(this, var_info);
}
