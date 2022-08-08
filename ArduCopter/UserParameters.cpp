#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_ATK_ANGLE"       , 0, UserParameters, fly_attack_angle, 2000.0f),

    AP_SUBGROUPINFO(Ucam_pid, "_PITCH_", 1, UserParameters, AC_PID),

    AP_GROUPINFO("_CAM_ANGLE_X"     , 2 , UserParameters, cam_angle_x, 135.0f),
    AP_GROUPINFO("_CAM_ANGLE_Y"     , 3 , UserParameters, cam_angle_y, 135.0f),
    AP_GROUPINFO("_CAM_PIXEL_X"     , 4 , UserParameters, cam_pixel_x, 320.0f),
    AP_GROUPINFO("_CAM_PIXEL_Y"     , 5 , UserParameters, cam_pixel_y, 320.0f),
    AP_GROUPINFO("_CAM_PITCH"       , 6 , UserParameters, cam_pitch_offset, -1000.0f),
    AP_GROUPINFO("_CAM_TIMEOUT"     , 7 , UserParameters, cam_time_out, 500),
    AP_GROUPINFO("_CAM_PRINT"       , 8 , UserParameters, cam_print, 0),
    AP_GROUPINFO("_CAM_TARGETX"     , 9 , UserParameters, cam_target_x, 0.0f),
    AP_GROUPINFO("_CAM_TARGETY"     , 10, UserParameters, cam_target_y, 0.0f),

    AP_GROUPINFO("_GCS_TIMEOUT"     , 11, UserParameters, gcs_time_out, 500),
    AP_GROUPINFO("_GCS_PRINT"       , 12, UserParameters, gcs_print, 0),
    AP_GROUPINFO("_GCS_NCRUISE"     , 13, UserParameters, gcs_num_cruise, 2),
    AP_GROUPINFO("_GCS_TCRUISE"     , 14, UserParameters, gcs_time_cruise, 0),
    AP_GROUPINFO("_GCS_GROUP_Y"     , 15, UserParameters, gcs_group_yaw, 90.f),
    AP_GROUPINFO("_GCS_REL_YAW"     , 16, UserParameters, gcs_group_relative_yaw, 0),
    AP_GROUPINFO("_GCS_S_YANGL"     , 17, UserParameters, gcs_search_yangle_left, -45.f),
    AP_GROUPINFO("_GCS_S_YANGR"     , 18, UserParameters, gcs_search_yangle_right, 45.f),
    AP_GROUPINFO("_GCS_S_YRATE"     , 19, UserParameters, gcs_search_yrate, 4.5f),
    AP_GROUPINFO("_GCS_ALT"         , 20, UserParameters, gcs_target_alt, 250),
    AP_GROUPINFO("_GCS_DELAY"       , 21, UserParameters, gcs_group_delay, 1000.f),

    AP_GROUPINFO("_YAW_TC"          , 22, UserParameters, fly_yaw_tc, 2.0f),
    AP_GROUPINFO("_P_LIMIT"         , 23, UserParameters, fly_pitch_limit, 3000.0f),
    AP_GROUPINFO("_R_LIMIT"         , 24, UserParameters, fly_roll_limit, 1500.0f),
    AP_GROUPINFO("_R_FACTOR"        , 25, UserParameters, fly_roll_factor, 1.0f),
    AP_GROUPINFO("_H_PSCALAR"       , 26, UserParameters, fly_pitch_scalar, 1.0f),
    AP_GROUPINFO("_H_CFACTOR"       , 27, UserParameters, fly_climb_factor, 1.0f),
    AP_GROUPINFO("_H_COFFSET"       , 28, UserParameters, fly_climb_rate_offset, 0.0f),

    AP_GROUPINFO("_TELE_POWER"      , 29, UserParameters, tele_power, 10),

    AP_GROUPINFO("_EKF_LAT"         , 30, UserParameters, ekf_origin_latitude, 0),
    AP_GROUPINFO("_EKF_LNG"         , 31, UserParameters, ekf_origin_longitude, 0),
    AP_GROUPINFO("_EKF_ALT"         , 32, UserParameters, ekf_origin_alt, 0),

    AP_GROUPINFO("_SEARCH_DIST"     , 33, UserParameters, group_search_dist, 1000.f),

    AP_SUBGROUPINFO(Thr_pid, "_Thr_", 34, UserParameters, AC_PID),
    AP_GROUPINFO("_P_UP_FACTOR"     , 35, UserParameters, atk_thr_up_factor, 1.f),
    AP_GROUPINFO("_P_RATE"          , 36, UserParameters, fly_pitch_rate, 3000.0f),
    AP_GROUPEND
};
