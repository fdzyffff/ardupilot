#include "UserParameters.h"

// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_SUBGROUPINFO(attack_throttle_pid, "_ATKTHR_", 0, UserParameters, AC_PID),
    AP_SUBGROUPINFO(attack_roll_pid    , "_ATKRLL_", 1, UserParameters, AC_PID),
    AP_GROUPINFO("_A_K1_PTH",      2, UserParameters, attack_k1_pitch,         2.0f),
    AP_GROUPINFO("_A_K2_PTH",      3, UserParameters, attack_k2_pitch,         2.0f),
    AP_GROUPINFO("_A_K1_YAW",      4, UserParameters, attack_k1_yaw,           2.0f),
    AP_GROUPINFO("_A_K2_YAW",      5, UserParameters, attack_k2_yaw,           2.0f),
    AP_GROUPINFO("_A_K_ANGLE",     6, UserParameters, attack_k_angle,          1.0f),
    AP_GROUPINFO("_A_THR",         7, UserParameters, attack_throttle,        75.0f),
    AP_GROUPINFO("_A_THR_RATE",    8, UserParameters, attack_throttle_rate,    1.0f),
    AP_GROUPINFO("_A_OUTMS",       9, UserParameters, attack_timeout,       2000),
    AP_GROUPINFO("_A_ANGLE",      10, UserParameters, attack_angle,           30.f),
    AP_GROUPINFO("_A_PTH_LIM",    11, UserParameters, pitch_limit,            30.f),
    AP_GROUPINFO("_A_PTH_RLIM",   12, UserParameters, pitch_rate_limit,       30.f),
    AP_GROUPINFO("_UPRINT",       13, UserParameters, print,                   0),
    AP_GROUPINFO("_CAM_W",        14, UserParameters, cam_width,             720),
    AP_GROUPINFO("_CAM_H",        15, UserParameters, cam_height,            720),
    AP_GROUPINFO("_CAM_ANG_X",    16, UserParameters, cam_angle_x,            45.0f),
    AP_GROUPINFO("_CAM_ANG_Y",    17, UserParameters, cam_angle_y,            45.0f),
    AP_GROUPINFO("_CAM_P_OFF",    18, UserParameters, cam_pitch_offset,       25.0f),
    AP_GROUPINFO("_LOCK_X",       19, UserParameters, lock_x,                500),
    AP_GROUPINFO("_LOCK_Y",       20, UserParameters, lock_y,                250),
    AP_GROUPINFO("_LOCK_SIZE",    21, UserParameters, lock_size,               2),
    AP_GROUPINFO("_CAM_OFFX",     22, UserParameters, cam_x_offset,          690),
    AP_GROUPINFO("_CAM_OFFY",     23, UserParameters, cam_y_offset,          370),
    AP_GROUPINFO("_LOCK_Y_DOWN",  24, UserParameters, lock_y_down,             2),
    AP_GROUPINFO("_A_OFF_PTH",    25, UserParameters, attack_pitch_off,       -5.0f),
 
    AP_GROUPEND
};

UserParameters::UserParameters():
    attack_throttle_pid(0.5f, 0.03f, 0.01f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.5f),
    attack_roll_pid(0.5f, 0.1f, 0.01f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.5f)
{
    AP_Param::setup_object_defaults(this, var_info);
}
