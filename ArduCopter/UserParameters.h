#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters() {}
    static const struct AP_Param::GroupInfo var_info[];
    AP_Float cam_angle_x;
    AP_Float cam_angle_y;
    AP_Float cam_pixel_x;
    AP_Float cam_pixel_y;
    AP_Float cam_pitch_offset;
    AP_Int32 cam_time_out;
	AP_Int32 cam_print;

    AP_Int32 gcs_time_out;
    AP_Int32 gcs_print;

	AP_Float fly_yaw_tc;
	AP_Float fly_pitch_factor;
    AP_Float fly_pitch_limit;
	AP_Float fly_pitch_scalar;
	AP_Float fly_climb_factor;
};
