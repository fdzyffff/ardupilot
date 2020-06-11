#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_INT8", 0, UserParameters, _int8, 0),
    AP_GROUPINFO("_INT16", 1, UserParameters, _int16, 0),
    AP_GROUPINFO("_FLOAT", 2, UserParameters, _float, 0),
    AP_GROUPINFO("_C1ANGLE", 3, UserParameters, cam1_angle, 135.0f),
    AP_GROUPINFO("_C1LEN", 4, UserParameters, cam1_xlength, 240.0f),
    AP_GROUPINFO("_C2ANGLE", 5, UserParameters, cam2_angle, 135.0f),
    AP_GROUPINFO("_C2LEN", 6, UserParameters, cam2_xlength, 240.0f),
    AP_GROUPINFO("_FLVEL", 7, UserParameters, genren_follow_vel, 400.0f),
    AP_GROUPINFO("_FLYAWRATE", 8, UserParameters, genren_follow_default_yawrate, 1000.0f),
    AP_GROUPINFO("_INFOLV", 9, UserParameters, genren_info_level, 2),
    AP_GROUPINFO("_C1TIMEOUT", 10, UserParameters, genren_cam1_timeout, 1000),
    AP_GROUPINFO("_C2TIMEOUT", 11, UserParameters, genren_cam2_timeout, 1000),
    AP_GROUPINFO("_FLYAWTC", 12, UserParameters, genren_follow_yawrate_max, 200.f),
    AP_GROUPINFO("_FLVELCH", 13, UserParameters, vel_channel, 8),
    AP_GROUPINFO("_FLVELCORR", 14, UserParameters, vel_corr_enable, 0),
    
    AP_GROUPEND
};
