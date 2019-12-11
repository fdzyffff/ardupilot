#include "Copter.h"

Copter::infoZQCC_class::infoZQCC_class(void) {};

bool Copter::infoZQCC_class::adjust_roll_pitch_yaw(float &roll, float &pitch, float angle_max, float &yaw_rate)
{
    if (angle_max <= 0.0f) {
        return false;
    }
    if (copter.g2.zqcc_sensor_length <= 0.0f) {
        return false;
    }
    if (copter.g2.zqcc_pitch <= 0.0f) {
        return false;
    }
    if (fabsf(copter.g2.zqcc_sensor_angle) > 165.0f) {
        return false;
    }

    float sensor_angle_rad = copter.g2.zqcc_sensor_angle*(M_PI/180.f);
    Vector2f rp_out(_pixel_x*2.0f/copter.g2.zqcc_sensor_length * tanf(sensor_angle_rad/2.0f) * copter.g2.zqcc_roll_factor * copter.g2.zqcc_pitch + copter.g2.zqcc_roll_comp, -copter.g2.zqcc_pitch);
    float rp_length = rp_out.length();
    if (rp_length > angle_max) {
        rp_out *= (angle_max / rp_length);
    }
    roll = rp_out.x;
    pitch = rp_out.y;
    yaw_rate = constrain_float(_pixel_x*copter.g2.zqcc_sensor_angle/copter.g2.zqcc_sensor_length * copter.g2.zqcc_roll_factor * 100.f, -400.f, 400.f);
    return true;
}

bool Copter::infoZQCC_class::adjust_climb_rate(float &target_climb_rate)
{
    float raw_climb_rate = target_climb_rate;
    if (copter.g2.zqcc_pitch <= 0.0f) {
        return false;
    }
    if (is_zero(copter.ahrs.cos_pitch())) {
        return false;
    }
    float sensor_angle_rad = copter.g2.zqcc_sensor_angle*(M_PI/180.f);
    float ratio = _pixel_y*2.0f/copter.g2.zqcc_sensor_length * tanf(sensor_angle_rad/2.0f);
    float vel_x_estimate;
    vel_x_estimate = 2000.f * fabsf(copter.ahrs.sin_pitch()/copter.ahrs.cos_pitch() );
    //vel_x_estimate = vel_x_estimate / sqrt(vel_x_estimate);
    target_climb_rate = ratio * vel_x_estimate * copter.g2.zqcc_climbrate_factor;
    if (copter.g2.zqcc_use_alt > 0) {
        target_climb_rate = 0.0f;
    }
    _delta_climb_rate = target_climb_rate - raw_climb_rate;
    return true;
}

void Copter::infoZQCC_class::update(float pixel_raw_x_in, float pixel_raw_y_in, float alt_cm_in)
{
    if (pixel_raw_x_in > copter.g2.zqcc_sensor_length || pixel_raw_y_in > copter.g2.zqcc_sensor_length) {
        copter.gcs().send_text(MAV_SEVERITY_CRITICAL,"vision data error!");
        return;
    }
    _pixel_raw_x = pixel_raw_x_in;
    _pixel_raw_y = pixel_raw_y_in;
    _pixel_x =  _pixel_raw_x * copter.ahrs.cos_roll() + _pixel_raw_y * copter.ahrs.sin_roll();
    _pixel_y = -_pixel_raw_x * copter.ahrs.sin_roll() + _pixel_raw_y * copter.ahrs.cos_roll();
    last_update = millis();
    if (copter.g2.zqcc_print & 1) {
        copter.gcs().send_text(MAV_SEVERITY_INFO, "Raw(%.2f, %.2f), Ret(%.2f, %.2f)", _pixel_raw_x, _pixel_raw_y, _pixel_x, _pixel_y);
    }
    copter.Log_Write_ZQCCINFO();

    _alt_cm_in = alt_cm_in;
    return;
}

void Copter::infoZQCC_class::update_sonar_alt(float &target_climb_rate)
{
    if (copter.g2.zqcc_use_alt < 0) {
        return;
    }
    if (_alt_cm_in > 100.f && fabsf(_alt_cm_in - _sonar_target_alt_cm) > 50.f) {
        _alt_avaliable = true;
        _sonar_target_alt_cm = _alt_cm_in;
        _sonar_target_alt_update_ms = millis();
        copter.gcs().send_text(MAV_SEVERITY_INFO, "new alt in : (%.2f %d)", _sonar_target_alt_cm, copter.g2.zqcc_alt_update_delay);
    }
    if (_alt_avaliable)
    {
        if (millis() - _sonar_target_alt_update_ms > (uint32_t)copter.g2.zqcc_alt_update_delay) {
            _alt_avaliable = false;       
            if (copter.g2.zqcc_use_alt > 0) {
                copter.target_rangefinder_alt = _sonar_target_alt_cm;
                copter.gcs().send_text(MAV_SEVERITY_INFO, "new alt updated : (%.2f)", copter.target_rangefinder_alt);
            }
        } else {
            target_climb_rate = 0.0f;
        }
    }
}

void Copter::infoZQCC_class::accumulate_lean(float roll, float pitch, float g_Dt)
{
    _acc_roll += roll * g_Dt * copter.g2.zqcc_brake_factor;
    _acc_pitch += pitch * g_Dt * copter.g2.zqcc_brake_factor;
    float lim_roll = MAX(1000.f , fabsf(2.0f * roll * copter.g2.zqcc_brake_factor) );
    float lim_pitch = MAX(1000.f , fabsf(2.0f * pitch * copter.g2.zqcc_brake_factor) );
    _acc_roll = constrain_float(_acc_roll, -lim_roll, lim_roll);
    _acc_pitch = constrain_float(_acc_pitch, -lim_pitch, lim_pitch);
    _lean_running = true;
}

void Copter::infoZQCC_class::release_lean(float &roll, float &pitch, float g_Dt)
{
    if( is_zero(roll) && is_zero(pitch) && _lean_running){
        roll = -constrain_float(_acc_roll, -2000.f, 2000.f);
        pitch = -constrain_float(_acc_pitch, -2000.f, 2000.f);
        _acc_roll += roll * g_Dt;
        _acc_pitch += pitch * g_Dt;
        if (fabsf(_acc_roll) < 1000.f && fabsf(_acc_pitch) < 1000.f) {
            reset_lean();
        }
    } else {
        reset_lean();
    }
}

void Copter::infoZQCC_class::reset_lean() {
    _lean_running = false;
    _acc_roll = 0.0f;
    _acc_pitch = 0.0f;
}

bool Copter::infoZQCC_class::running()
{
    if (last_update == 0) {
        return false;
    } else if ( (copter.g2.zqcc_timeout != 0) && (int32_t(millis() - last_update) > copter.g2.zqcc_timeout) ) {
        return false;
    }
    return true;
}

void Copter::infoZQCC_class::init()
{
    last_update = 0;
    _pixel_raw_x = 0.0f;
    _pixel_raw_y = 0.0f;
    _pixel_x = 0.0f;
    _pixel_y = 0.0f;
    _delta_climb_rate = 0.0f;
    _sonar_target_alt_cm = 0.0f;
    _alt_cm_in = 0.0f;
    _alt_avaliable = false;
    _sonar_target_alt_update_ms = millis();
    reset_lean();
}
