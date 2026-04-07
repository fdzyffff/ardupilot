#include "UserParameters.h"
#include "config.h"

#if USER_PARAMS_ENABLED == ENABLED
// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_PRINT",      0, UserParameters, cam_print,               0),
    AP_GROUPINFO("_TIMEOUT",    1, UserParameters, cam_time_out,         2000),
    AP_GROUPINFO("_AIM_K",      2, UserParameters, attack_k,                2.0f),
    AP_GROUPINFO("_AIM_K2",     3, UserParameters, attack_k2,               2.0f),
    AP_GROUPINFO("_SF_FBIG",    4, UserParameters, tag_scale_f_big,         1.0f),
    AP_GROUPINFO("_SF_FSML",    5, UserParameters, tag_scale_f_small,       0.205f),
    AP_GROUPINFO("_SF_UBIG",    6, UserParameters, tag_scale_u_big,         1.0f),
    AP_GROUPINFO("_SF_USML",    7, UserParameters, tag_scale_u_small,       0.201f),
    AP_GROUPINFO("_CAM_ANG_X",  8, UserParameters, cam_angle_x,            45.0f),
    AP_GROUPINFO("_CAM_ANG_Y",  9, UserParameters, cam_angle_y,            45.0f),
    AP_GROUPINFO("_ASSIT",     10, UserParameters, assit_gain,             3.0f),
    AP_GROUPINFO("_FILT_HZ",   11, UserParameters, filt_hz,               10.0f),
    AP_GROUPINFO("_CAM_ROFF",  12, UserParameters, cam_roll_off,           0.0f),
    AP_GROUPINFO("_CAM_POFF",  13, UserParameters, cam_pitch_off,          0.0f),
    AP_GROUPINFO("_UHK_ALT",   14, UserParameters, hook_mission_alt,       2.0f),
    AP_GROUPINFO("_APCH_CM",   15, UserParameters, approach_cm,            200.0f),
    AP_GROUPINFO("_TESTCAM",   16, UserParameters, test_cam,               0),
    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
