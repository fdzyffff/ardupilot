#include "Copter.h"

void Copter::genren_init(void)
{
    genren_state = genren_STANDBY;

    genren_msg_follow.last_update_ms = 0; 
    genren_msg_follow.valid = false;
    genren_msg_follow.raw_x = 0.0f;
    genren_msg_follow.raw_y = 0.0f;
    genren_msg_follow.corr_x = 0.0f;
    genren_msg_follow.corr_y = 0.0f;
    genren_msg_follow.out.x = 0.0f;
    genren_msg_follow.out.y = 0.0f;
    genren_msg_follow.out.z = 0.0f;

    genren_msg_avoid.last_update_ms = 0; 
    genren_msg_avoid.valid = false;
    genren_msg_avoid.raw_x = 0.0f;
    genren_msg_avoid.raw_y = 0.0f;
    genren_msg_avoid.corr_x = 0.0f;
    genren_msg_avoid.corr_y = 0.0f;
    genren_msg_avoid.out.x = 0.0f;
    genren_msg_avoid.out.y = 0.0f;
    genren_msg_avoid.out.z = 0.0f;
}

void Copter::genren_follow_handle(float input_x, float input_y, int16_t healthy)
{    
    if (g2.user_parameters.genren_info_level >= 2) {
        gcs().send_text(MAV_SEVERITY_INFO, "C1 in X:%0.2f Y:%0.2f", input_x, input_y);
    }
    if (healthy == 0) {
        return;
    }
    // if (is_zero(input_x) && is_zero(input_y)) {
    //     return;
    // }
    genren_msg_follow.raw_x = input_x;
    genren_msg_follow.raw_y = input_y;
    genren_msg_follow.last_update_ms = millis();
    genren_msg_follow.valid = true;
    Matrix3f tmp_m;

    float pixel_x = constrain_float(g2.user_parameters.cam1_xlength, 60.0f, 1080.f)*0.5f/tanf(radians( constrain_float(g2.user_parameters.cam1_angle, 20.0f, 80.0f) *0.5f));
    Vector3f tmp_input = Vector3f(pixel_x,input_x,-input_y);

    tmp_m.from_euler(ahrs_view->roll, ahrs_view->pitch, 0.0f);
    genren_msg_follow.out = tmp_m*tmp_input;
    genren_msg_follow.corr_x = genren_msg_follow.out.y;
    genren_msg_follow.corr_y = -genren_msg_follow.out.z;

    tmp_m.from_euler(ahrs_view->roll, ahrs_view->pitch, ahrs_view->yaw);
    genren_msg_follow.out = tmp_m*tmp_input;

    if (g2.user_parameters.genren_info_level >= 2) {
        gcs().send_text(MAV_SEVERITY_INFO, "C1 raw X:%0.2f Y:%0.2f", genren_msg_follow.raw_x, genren_msg_follow.raw_y);
        gcs().send_text(MAV_SEVERITY_INFO, "C1 cor X:%0.2f Y:%0.2f", genren_msg_follow.corr_x, genren_msg_follow.corr_y);
    }
    if (g2.user_parameters.genren_info_level >= 1) {
        gcs().send_text(MAV_SEVERITY_INFO, "C1 out X:%0.2f Y:%0.2f Z:%0.2f", genren_msg_follow.out.x, genren_msg_follow.out.y, genren_msg_follow.out.z);
    }
}

void Copter::genren_avoid_handle(float input_x, float input_y, int16_t healthy)
{
    if (g2.user_parameters.genren_info_level >= 2) {
        gcs().send_text(MAV_SEVERITY_INFO, "C2 in X:%0.2f Y:%0.2f", input_x, input_y);
    }
    if (healthy == 0) {
        return;
    }
    // if (is_zero(input_x) && is_zero(input_y)) {
    //     return;
    // }
    genren_msg_avoid.raw_x = input_x;
    genren_msg_avoid.raw_y = input_y;
    genren_msg_avoid.last_update_ms = millis();
    genren_msg_avoid.valid = true;
    Matrix3f tmp_m;

    float pixel_x = constrain_float(g2.user_parameters.cam2_xlength, 60.0f, 1080.f)*0.5f/tanf(radians( constrain_float(g2.user_parameters.cam2_angle, 20.0f, 80.0f) *0.5f));
    Vector3f tmp_input = Vector3f(pixel_x,input_x,-input_y);

    tmp_m.from_euler(ahrs_view->roll, ahrs_view->pitch, 0.0f);
    genren_msg_avoid.out = tmp_m*tmp_input;
    genren_msg_avoid.corr_x = genren_msg_avoid.out.y;
    genren_msg_avoid.corr_y = -genren_msg_avoid.out.z;
    if (g2.user_parameters.genren_info_level >= 2) {
        gcs().send_text(MAV_SEVERITY_INFO, "C2 raw X:%0.2f Y:%0.2f", genren_msg_avoid.raw_x, genren_msg_avoid.raw_y);
    }
    if (g2.user_parameters.genren_info_level >= 1) {
        gcs().send_text(MAV_SEVERITY_INFO, "C2 cor X:%0.2f Y:%0.2f", genren_msg_avoid.corr_x, genren_msg_avoid.corr_y);
    }
}

void Copter::genren_status_update()
{
    uint32_t tnow_ms = millis();
    if (g2.user_parameters.genren_cam1_timeout != 0 && (tnow_ms - genren_msg_follow.last_update_ms > (uint32_t)g2.user_parameters.genren_cam1_timeout)) {
        genren_msg_follow.valid = false;
    }
    if (g2.user_parameters.genren_cam2_timeout != 0 && (tnow_ms - genren_msg_avoid.last_update_ms > (uint32_t)g2.user_parameters.genren_cam2_timeout)) {
        genren_msg_avoid.valid = false;
    }
}
