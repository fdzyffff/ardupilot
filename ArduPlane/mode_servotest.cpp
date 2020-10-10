#include "mode.h"
#include "Plane.h"

bool ModeServotest::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

    _tstart = millis();

    return true;
}

void ModeServotest::_exit()
{
    gcs().send_text(MAV_SEVERITY_INFO, "Test finish");
}

void ModeServotest::update()
{
    uint32_t timer =  millis() - _tstart;
    int16_t roll_in = 0;
    int16_t pitch_in = 0;
    int16_t yaw_in = 0;
    if (timer < 2000) {
        roll_in = -4500;
        pitch_in = 0;
        yaw_in = 0;
    } else if (timer < 4000) {
        roll_in = 4500;
        pitch_in = 0;
        yaw_in = 0;
    } else if (timer < 6000) {
        roll_in = 0;
        pitch_in = 0;
        yaw_in = 0;
    } else if (timer < 8000) {
        roll_in = 0;
        pitch_in = -4500;
        yaw_in = 0;
    } else if (timer < 10000) {
        roll_in = 0;
        pitch_in = 4500;
        yaw_in = 0;
    } else if (timer < 12000) {
        roll_in = 0;
        pitch_in = 0;
        yaw_in = 0;
    } else if (timer < 14000) {
        roll_in = 0;
        pitch_in = 0;
        yaw_in = -4500;
    } else if (timer < 16000) {
        roll_in = 0;
        pitch_in = 0;
        yaw_in = 4500;
    } else if (timer < 18000) {
        roll_in = 0;
        pitch_in = 0;
        yaw_in = 0;
    } else {
        if (plane.previous_mode != nullptr && plane.previous_mode != &plane.mode_servotest) {
            plane.set_mode(*plane.previous_mode, MODE_REASON_UNAVAILABLE);
        } else {
            plane.set_mode(plane.mode_fbwa, MODE_REASON_UNAVAILABLE);
        }
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll_in);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch_in);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, yaw_in);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
    plane.steering_control.steering = plane.steering_control.rudder = yaw_in;
}

