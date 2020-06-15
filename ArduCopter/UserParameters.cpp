#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars

    AP_GROUPINFO("_C1ANGLE", 1, UserParameters, cam1_angle, 135.0f),
    AP_GROUPINFO("_C1LEN", 2, UserParameters, cam1_xlength, 240.0f),
    AP_GROUPINFO("_V_LAND", 3, UserParameters, luoche_land_speed, 50.0f),
    AP_GROUPINFO("_V_XY", 4, UserParameters, luoche_xy_speed, 50.0f),
    AP_GROUPINFO("_INFOLV", 5, UserParameters, luoche_info_level, 2),
    AP_GROUPINFO("_C1TIMEOUT", 6, UserParameters, luoche_cam1_timeout, 1000),
    AP_GROUPINFO("_FLVELCH", 7, UserParameters, vel_channel, 8),
    AP_GROUPINFO("_FL_ALT", 8, UserParameters, luoche_follow_alt, 1000.0f),
    AP_GROUPINFO("_FINAL_ALT", 9, UserParameters, luoche_final_alt, 50.0f),
    AP_GROUPINFO("_V_FACTOR", 10, UserParameters, luoche_vel_factor, 1.4f),
    
    AP_GROUPEND
};
