#include "UserParameters.h"

// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_A_K1_PTH",      0, UserParameters, attack_k1_pitch,         2.0f),
    AP_GROUPINFO("_A_K2_PTH",      1, UserParameters, attack_k2_pitch,         2.0f),
    AP_GROUPINFO("_A_K1_YAW",      2, UserParameters, attack_k1_yaw,           2.0f),
    AP_GROUPINFO("_A_K2_YAW",      3, UserParameters, attack_k2_yaw,           2.0f),
    AP_GROUPINFO("_A_K_ANGLE",     4, UserParameters, attack_k_angle,         30.f),
    AP_GROUPINFO("_A_K_ROLL",      5, UserParameters, attack_roll_factor,      0.1f),
    AP_GROUPINFO("_A_THR",         6, UserParameters, attack_throttle,        75.0f),
    AP_GROUPINFO("_A_THR_RATE",    7, UserParameters, attack_throttle_rate,    1.0f),
    AP_GROUPINFO("_A_OUTMS",       8, UserParameters, attack_timeout,       2000),
    AP_GROUPINFO("_A_ANGLE",       9, UserParameters, attack_angle,           30.f),
    AP_GROUPINFO("_A_PTH_LIM",    10, UserParameters, pitch_limit,            30.f),
    AP_GROUPINFO("_A_PTH_RLIM",   11, UserParameters, pitch_rate_limit,       30.f),
    AP_GROUPINFO("_UPRINT",       12, UserParameters, print,                   0),
    AP_SUBGROUPINFO(attack_throttle_pid, "_ATKTHR_", 9, UserParameters, AC_PID),
    AP_GROUPINFO("_CAM_W",        13, UserParameters, cam_width,             720),
    AP_GROUPINFO("_CAM_H",        14, UserParameters, cam_height,            720),
    AP_GROUPINFO("_CAM_ANG_X",    15, UserParameters, cam_angle_x,            45.0f),
    AP_GROUPINFO("_CAM_ANG_Y",    16, UserParameters, cam_angle_y,            45.0f),
    AP_GROUPINFO("_CAM_P_OFF",    17, UserParameters, cam_pitch_offset,       25.0f),
    AP_GROUPINFO("_LOCK_X",       18, UserParameters, lock_x,                500),
    AP_GROUPINFO("_LOCK_Y",       19, UserParameters, lock_y,                250),
    AP_GROUPINFO("_LOCK_SIZE",    20, UserParameters, lock_size,               2),
    AP_GROUPINFO("_CAM_OFFX",     21, UserParameters, cam_x_offset,          690),
    AP_GROUPINFO("_CAM_OFFY",     22, UserParameters, cam_y_offset,          370),
    AP_GROUPINFO("_LOCK_Y_DOWN",  23, UserParameters, lock_y_down,             2),
 
    AP_GROUPEND
};

UserParameters::UserParameters():
    attack_throttle_pid(0.15f, 0.25f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.02f)
{
    AP_Param::setup_object_defaults(this, var_info);
}
