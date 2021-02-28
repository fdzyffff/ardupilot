#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars

    AP_GROUPINFO("_CAM_ANGLE_X"     , 0 , UserParameters, cam_angle_x, 135.0f),
    AP_GROUPINFO("_CAM_ANGLE_Y"     , 1 , UserParameters, cam_angle_y, 135.0f),
    AP_GROUPINFO("_CAM_PIXEL_X"     , 2 , UserParameters, cam_pixel_x, 320.0f),
    AP_GROUPINFO("_CAM_PIXEL_Y"     , 3 , UserParameters, cam_pixel_y, 320.0f),
    AP_GROUPINFO("_CAM_PITCH"       , 4 , UserParameters, cam_pitch_offset, 320.0f),
    AP_GROUPINFO("_CAM_TIMEOUT"     , 5 , UserParameters, cam_time_out, 500),
    AP_GROUPINFO("_CAM_PRINT"       , 6 , UserParameters, cam_print, 0),

    AP_GROUPINFO("_GCS_TIMEOUT"     , 7 , UserParameters, gcs_time_out, 500),
    AP_GROUPINFO("_GCS_PRINT"       , 8 , UserParameters, gcs_print, 0),

    AP_GROUPINFO("_YAW_TC"          , 9 , UserParameters, fly_yaw_tc, 2.0f),
    //AP_GROUPINFO("_PITCH_P"         , 10, UserParameters, fly_pitch_p, 1.0f),
    //AP_GROUPINFO("_PITCH_D"         , 11, UserParameters, fly_pitch_d, 0.1f),
    AP_GROUPINFO("_P_LIMIT"         , 10, UserParameters, fly_pitch_limit, 3000.0f),
    AP_GROUPINFO("_HP_SCALAR"       , 11, UserParameters, fly_pitch_scalar, 2.0f),
    AP_GROUPINFO("_HCLB_FACTOR"     , 12, UserParameters, fly_climb_factor, 2.0f),

    AP_SUBGROUPINFO(Ucam_pid, "_PITCH_", 13, UserParameters, AC_PID),

    AP_GROUPEND
};
