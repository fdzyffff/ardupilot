/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_MotorsSkateboard.h"

extern const AP_HAL::HAL& hal;

// init
void AP_MotorsSkateboard::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // gcs().send_text(MAV_SEVERITY_ERROR, "MotorsSKB INIT");
    // add_motor_num(AP_MOTORS_MOT_1);

    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    // motor_enabled[AP_MOTORS_MOT_1] = true;

    SRV_Channels::set_angle(SRV_Channel::k_engine_srv_0, 4500);
    SRV_Channels::set_angle(SRV_Channel::k_engine_srv_1, 4500);
    SRV_Channels::set_angle(SRV_Channel::k_engine_srv_2, 4500);
    SRV_Channels::set_angle(SRV_Channel::k_engine_srv_3, 4500);
    SRV_Channels::set_angle(SRV_Channel::k_engine_srv_4, 4500);
    SRV_Channels::set_angle(SRV_Channel::k_engine_srv_5, 4500);
    SRV_Channels::set_angle(SRV_Channel::k_engine_srv_6, 4500);
    SRV_Channels::set_angle(SRV_Channel::k_engine_srv_7, 4500);

    _mav_type = MAV_TYPE_COAXIAL;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_SKATEBOARD);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsSkateboard::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    set_initialised_ok((frame_class == MOTOR_FRAME_SKATEBOARD));
}

// set update rate to motors - a value in hertz
void AP_MotorsSkateboard::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 3 motors (but not the servo on channel 7)
    uint32_t mask = 0;
    rc_set_freq(mask, _speed_hz);
}

void AP_MotorsSkateboard::output_to_motors()
{
    // SRV_Channels::set_output_scaled(function, angle_cd);
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_0, 0.0f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_1, 0.0f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_2, 0.0f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_3, 0.0f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_4, 0.0f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_5, 0.0f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_6, 0.0f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_7, 0.0f);
            break;
        case SpoolState::GROUND_IDLE:
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_0, constrain_float(_srv_pitch*2.0f, -1.0f, 1.0f)*4500.f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_1, constrain_float(_srv_roll*2.0f, -1.0f, 1.0f)*4500.f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_2, constrain_float(_srv_roll+_srv_pitch, -1.0f, 1.0f)*4500.f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_3, constrain_float(_srv_roll-_srv_pitch, -1.0f, 1.0f)*4500.f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_4, constrain_float(_srv_roll+_srv_pitch, -1.0f, 1.0f)*4500.f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_5, constrain_float(_srv_roll-_srv_pitch, -1.0f, 1.0f)*4500.f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_6, constrain_float(_srv_yaw, -1.0f, 1.0f)*4500.f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_7, constrain_float(-_srv_yaw, -1.0f, 1.0f)*4500.f);
            break;
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsSkateboard::get_motor_mask()
{
    uint32_t motor_mask = 0;
    uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);

    // add parent's mask
    mask |= AP_MotorsMulticopter::get_motor_mask();

    return mask;
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsSkateboard::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   throttle_avg_max;           // throttle thrust average maximum value, 0.0 - 1.0

    // SRV_Channels::set_angle(SRV_Channel::k_engine_srv_0, 4500);
    // SRV_Channels::set_angle(SRV_Channel::k_engine_srv_1, 4500);
    // SRV_Channels::set_angle(SRV_Channel::k_engine_srv_2, 4500);
    // SRV_Channels::set_angle(SRV_Channel::k_engine_srv_3, 4500);
    // SRV_Channels::set_angle(SRV_Channel::k_engine_srv_4, 4500);
    // SRV_Channels::set_angle(SRV_Channel::k_engine_srv_5, 4500);
    // SRV_Channels::set_angle(SRV_Channel::k_engine_srv_6, 4500);
    // SRV_Channels::set_angle(SRV_Channel::k_engine_srv_7, 4500);

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain; // we scale this so a thrust request of 1.0f will ask for full servo deflection at full rear throttle
    throttle_thrust = get_throttle() * compensation_gain;
    throttle_avg_max = _throttle_avg_max * compensation_gain;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    throttle_avg_max = constrain_float(throttle_avg_max, throttle_thrust, _throttle_thrust_max);

    // The following mix may be offer less coupling between axis but needs testing
    //_thrust_right = roll_thrust * -0.5f + pitch_thrust * 1.0f;
    //_thrust_left = roll_thrust * 0.5f + pitch_thrust * 1.0f;
    //_thrust_rear = 0;

    _srv_roll = roll_thrust * 0.5f;
    _srv_pitch = pitch_thrust * 0.5f;
    _srv_yaw = yaw_thrust;

    // compensation_gain can never be zero
    _throttle_out = throttle_thrust / compensation_gain;

    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    _srv_roll = constrain_float(_srv_roll, -1.0f, 1.0f);
    _srv_pitch = constrain_float(_srv_pitch, -1.0f, 1.0f);
    _srv_yaw = constrain_float(_srv_yaw, -1.0f, 1.0f);
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsSkateboard::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
    switch (motor_seq) {
        case 1:
            SRV_Channels::set_output_pwm(SRV_Channel::k_engine_srv_0, pwm);
            break;
        case 2:
            SRV_Channels::set_output_pwm(SRV_Channel::k_engine_srv_1, pwm);
            break;
        case 3:
            SRV_Channels::set_output_pwm(SRV_Channel::k_engine_srv_2, pwm);
            break;
        case 4:
            SRV_Channels::set_output_pwm(SRV_Channel::k_engine_srv_3, pwm);
            break;
        case 5:
            SRV_Channels::set_output_pwm(SRV_Channel::k_engine_srv_4, pwm);
            break;
        case 6:
            SRV_Channels::set_output_pwm(SRV_Channel::k_engine_srv_5, pwm);
            break;
        case 7:
            SRV_Channels::set_output_pwm(SRV_Channel::k_engine_srv_6, pwm);
            break;
        case 8:
            SRV_Channels::set_output_pwm(SRV_Channel::k_engine_srv_7, pwm);
            break;
        default:
            // do nothing
            break;
    }
}

/*
  call vehicle supplied thrust compensation if set. This allows for
  vehicle specific thrust compensation for motor arrangements such as
  the forward motors tilting
*/
void AP_MotorsSkateboard::thrust_compensation(void)
{
    ;
}

/*
  override tricopter tail servo output in output_motor_mask
 */
void AP_MotorsSkateboard::output_motor_mask(float thrust, uint8_t mask, float rudder_dt)
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_0, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_1, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_2, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_3, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_4, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_5, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_6, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_engine_srv_7, 0.0f);
}

float AP_MotorsSkateboard::get_roll_factor(uint8_t i)
{
    float ret = 0.0f;

    return ret;
}
