#include "Copter.h"

//Copter::infoZQCC_class Copter::infoZQCC;

Copter::infoZQCC_class::infoZQCC_class(void) {};

bool Copter::infoZQCC_class::adjust_roll_pitch(float &roll, float &pitch, float angle_max)
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
    Vector2f rp_out(_pixel_x*2.0f/copter.g2.zqcc_sensor_length * tanf(sensor_angle_rad/2.0f) * copter.g2.zqcc_roll_factor * copter.g2.zqcc_pitch, -copter.g2.zqcc_pitch);
    float rp_length = rp_out.length();
    if (rp_length > angle_max) {
        rp_out *= (angle_max / rp_length);
    }
    roll = rp_out.x;
    pitch = rp_out.y;
    return true;
}

void Copter::infoZQCC_class::update(float pixel_raw_x_in, float pixel_raw_y_in)
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
    if (copter.g2.zqcc_print) {
        copter.gcs().send_text(MAV_SEVERITY_INFO, "Raw(%.2f, %.2f), Ret(%.2f, %.2f)", _pixel_raw_x, _pixel_raw_y, _pixel_x, _pixel_y);
    }
    return;
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
}
