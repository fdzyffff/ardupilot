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
    AP_GROUPINFO("_CAM_W",      4, UserParameters, cam_width,             720),
    AP_GROUPINFO("_CAM_H",      5, UserParameters, cam_height,            720),
    AP_GROUPINFO("_CAM_ANG_X",  6, UserParameters, cam_angle_x,            45.0f),
    AP_GROUPINFO("_CAM_ANG_Y",  7, UserParameters, cam_angle_y,            45.0f),
    AP_GROUPINFO("_LIM_ANGLE" , 8, UserParameters, angle_limit,            30.f),
    AP_GROUPINFO("_LIM_RATE",   9, UserParameters, rate_limit,             20.f),
    AP_GROUPINFO("_LOCA_LAT",   10, UserParameters, loc_A_lat,              0),
    AP_GROUPINFO("_LOCA_LNG",   11, UserParameters, loc_A_lng,              0),
    AP_GROUPINFO("_LOCA_ALT",   12, UserParameters, loc_A_alt,              0),
    AP_GROUPINFO("_LOCA_YAW",   13, UserParameters, loc_A_yaw,              0.f),
    AP_GROUPINFO("_LOCB_LAT",   14, UserParameters, loc_B_lat,              0),
    AP_GROUPINFO("_LOCB_LNG",   15, UserParameters, loc_B_lng,              0),
    AP_GROUPINFO("_LOCB_ALT",   16, UserParameters, loc_B_alt,              0),
    AP_GROUPINFO("_LOCB_YAW",   17, UserParameters, loc_B_yaw,              0.f),

    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
