#include "Copter.h"


void UCam::handle_current_pos_info(float x, float y, float z) {
    current_pos_SIM.x = x;
    current_pos_SIM.y = y;
    current_pos_SIM.z = -z;
}

void UCam::update_value_SIM(float dt){
    if (copter.g2.user_parameters.cam_pixel_x.get() < 50.f) {copter.g2.user_parameters.cam_pixel_x.set_and_save(50.f);}
    if (copter.g2.user_parameters.cam_pixel_y.get() < 50.f) {copter.g2.user_parameters.cam_pixel_y.set_and_save(50.f);}
    if (copter.g2.user_parameters.cam_angle_x.get() < 30.f) {copter.g2.user_parameters.cam_angle_x.set_and_save(30.f);}
    if (copter.g2.user_parameters.cam_angle_y.get() < 30.f) {copter.g2.user_parameters.cam_angle_y.set_and_save(30.f);}
    if (copter.g2.user_parameters.fly_yaw_tc.get() < 0.1f) {copter.g2.user_parameters.fly_yaw_tc.set_and_save(0.1f);}
    update_target_pitch_rate_SIM();
    update_target_roll_angle_SIM();
    update_target_yaw_rate_SIM();
    update_target_track_angle_SIM();
    update_q_rate_cds_SIM(dt);
}

void UCam::update_target_pitch_rate_SIM() {
    float measurement = degrees(copter.ahrs_view->pitch)/30.f; //-1 to +0.1
    float my_target_pitch_rate = copter.g2.user_parameters.Ucam_pid.update_all(-1.0f, measurement, false)*copter.g2.user_parameters.fly_pitch_rate.get();
    _target_pitch_rate = my_target_pitch_rate ;
    // gcs().send_text(MAV_SEVERITY_INFO, "%f", _target_pitch_rate);
}

void UCam::update_target_roll_angle_SIM() {
    float target_yaw = degrees(atan2f(target_pos_SIM.y - current_pos_SIM.y, target_pos_SIM.x - current_pos_SIM.x));
    float current_yaw = degrees(copter.ahrs_view->yaw);
    float info_x = wrap_180(target_yaw - current_yaw)/30.f;
    info_x = constrain_float(info_x*copter.g2.user_parameters.fly_roll_factor, -1.0f, 1.0f);
    float pitch_limit = MAX(copter.g2.user_parameters.fly_roll_limit, degrees(fabsf(copter.ahrs_view->pitch)));
    _target_roll_angle = pitch_limit*info_x;
}

void UCam::update_target_yaw_rate_SIM() {
    float target_yaw = degrees(atan2f(target_pos_SIM.y - current_pos_SIM.y, target_pos_SIM.x - current_pos_SIM.x));
    float current_yaw = degrees(copter.ahrs_view->yaw);
    float info_x = wrap_180(target_yaw - current_yaw)/30.f;
    info_x = constrain_float(info_x, -1.0f, 1.0f);
    float yaw_rate_tc = copter.g2.user_parameters.fly_yaw_tc;
    float yaw_rate_cds = constrain_float((3000.f * info_x * yaw_rate_tc), -6000.f, 6000.f);
    _target_yaw_rate_cds = yaw_rate_cds;
}

void UCam::update_target_track_angle_SIM() {
    Vector2f pos_xy = Vector2f(target_pos_SIM.x - current_pos_SIM.x, target_pos_SIM.y - current_pos_SIM.y);
    float pos_xy_length = MAX(0.1f, pos_xy.length());
    _current_angle_deg = degrees(atanf( (current_pos_SIM.z - target_pos_SIM.z) / pos_xy_length));
}

void UCam::update_q_rate_cds_SIM(float dt) {
    static float last_q_cd = 0.0f;
    float q_cd = 100.f*degrees(atan2f(target_pos_SIM.y - current_pos_SIM.y, target_pos_SIM.x - current_pos_SIM.x));
    q_cd = wrap_180_cd(q_cd);
    float q_cds = (q_cd - last_q_cd)/dt;
    last_q_cd = q_cd;
    _q_cds_filter.apply(q_cds, dt);
    _q_rate_cds = _q_cds_filter.get();
}
