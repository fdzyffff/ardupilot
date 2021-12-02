#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_ATK_ANGLE"       , 0, UserParameters, fly_attack_angle, 2000.0f),

    AP_SUBGROUPINFO(Ucam_pid, "_PITCH_", 1, UserParameters, AC_PID),
    AP_SUBGROUPINFO(Thr_pid, "_Thr_", 2, UserParameters, AC_PID),

    AP_GROUPINFO("_CAM_ANGLE_X"     , 3 , UserParameters, cam_angle_x, 135.0f),
    AP_GROUPINFO("_CAM_ANGLE_Y"     , 4 , UserParameters, cam_angle_y, 135.0f),
    AP_GROUPINFO("_CAM_PIXEL_X"     , 5 , UserParameters, cam_pixel_x, 320.0f),
    AP_GROUPINFO("_CAM_PIXEL_Y"     , 6 , UserParameters, cam_pixel_y, 320.0f),
    AP_GROUPINFO("_CAM_PITCH"       , 7 , UserParameters, cam_pitch_offset, -1000.0f),
    AP_GROUPINFO("_CAM_TIMEOUT"     , 8 , UserParameters, cam_time_out, 500),
    AP_GROUPINFO("_CAM_PRINT"       , 9 , UserParameters, cam_print, 0),
    AP_GROUPINFO("_CAM_TARGETX"     , 10 , UserParameters, cam_target_x, 0.0f),
    AP_GROUPINFO("_CAM_TARGETY"     , 11, UserParameters, cam_target_y, 0.0f),

    AP_GROUPINFO("_YAW_TC"          , 12, UserParameters, fly_yaw_tc, 2.0f),
    AP_GROUPINFO("_P_LIMIT"         , 13, UserParameters, fly_pitch_limit, 3000.0f),
    AP_GROUPINFO("_R_LIMIT"         , 14, UserParameters, fly_roll_limit, 1500.0f),
    AP_GROUPINFO("_R_FACTOR"        , 15, UserParameters, fly_roll_factor, 1.0f),
    AP_GROUPINFO("_H_PSCALAR"       , 16, UserParameters, fly_pitch_scalar, 1.0f),
    AP_GROUPINFO("_H_CFACTOR"       , 17, UserParameters, fly_climb_factor, 1.0f),

    AP_GROUPINFO("_CAM_TEST"        , 18, UserParameters, uart_cam_test, 0),
    
    AP_GROUPEND
};
