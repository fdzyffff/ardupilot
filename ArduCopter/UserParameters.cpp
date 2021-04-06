#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars

    AP_GROUPINFO("_CAM_ANGLE_X"     , 0 , UserParameters, cam_angle_x, 135.0f),
    AP_GROUPINFO("_CAM_ANGLE_Y"     , 1 , UserParameters, cam_angle_y, 135.0f),
    AP_GROUPINFO("_CAM_PIXEL_X"     , 2 , UserParameters, cam_pixel_x, 320.0f),
    AP_GROUPINFO("_CAM_PIXEL_Y"     , 3 , UserParameters, cam_pixel_y, 320.0f),
    AP_GROUPINFO("_CAM_PITCH"       , 4 , UserParameters, cam_pitch_offset, -1000.0f),
    AP_GROUPINFO("_CAM_TIMEOUT"     , 5 , UserParameters, cam_time_out, 500),
    AP_GROUPINFO("_CAM_PRINT"       , 6 , UserParameters, cam_print, 0),
    AP_GROUPINFO("_CAM_TARGETX"     , 7 , UserParameters, cam_target_x, 0.0f),
    AP_GROUPINFO("_CAM_TARGETY"     , 8 , UserParameters, cam_target_y, 0.0f),

    AP_GROUPINFO("_GCS_TIMEOUT"     , 9 , UserParameters, gcs_time_out, 500),
    AP_GROUPINFO("_GCS_PRINT"       , 10, UserParameters, gcs_print, 0),
    AP_GROUPINFO("_GCS_NCRUISE"     , 11, UserParameters, gcs_num_cruise, 2),
    AP_GROUPINFO("_GCS_TCRUISE"     , 12, UserParameters, gcs_time_cruise, 0),
    AP_GROUPINFO("_GCS_GROUP_Y"     , 13, UserParameters, gcs_group_yaw, 90.f),
    AP_GROUPINFO("_GCS_S_YANG"      , 14, UserParameters, gcs_search_yangle, 45.f),
    AP_GROUPINFO("_GCS_S_YRATE"     , 15, UserParameters, gcs_search_yrate, 4.5f),

    AP_GROUPINFO("_YAW_TC"          , 16, UserParameters, fly_yaw_tc, 2.0f),
    AP_GROUPINFO("_P_LIMIT"         , 17, UserParameters, fly_pitch_limit, 3000.0f),
    AP_GROUPINFO("_R_LIMIT"         , 18, UserParameters, fly_roll_limit, 1500.0f),
    AP_GROUPINFO("_R_FACTOR"        , 19, UserParameters, fly_roll_factor, 1.0f),
    AP_GROUPINFO("_H_PSCALAR"       , 20, UserParameters, fly_pitch_scalar, 1.0f),
    AP_GROUPINFO("_H_CFACTOR"       , 21, UserParameters, fly_climb_factor, 1.0f),
    AP_GROUPINFO("_H_COFFSET"       , 22, UserParameters, fly_climb_rate_offset, 0.0f),

    AP_GROUPINFO("_ATK_ANGLE"       , 23, UserParameters, fly_attack_angle, 2000.0f),

    AP_SUBGROUPINFO(Ucam_pid, "_PITCH_", 24, UserParameters, AC_PID),

    AP_GROUPEND
};
