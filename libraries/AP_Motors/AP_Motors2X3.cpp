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
#include <SRV_Channel/SRV_Channel.h>

#include "AP_Motors2X3.h"

extern const AP_HAL::HAL& hal;

// init
void AP_Motors2X3::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    add_motor_num(AP_MOTORS_MOT_1);
    add_motor_num(AP_MOTORS_MOT_2);

    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;

    // allow mapping of servo 1~4
    add_motor_num(AP_SERVO_1);
    add_motor_num(AP_SERVO_2);
    add_motor_num(AP_SERVO_3);

    SRV_Channels::set_angle(SRV_Channels::get_motor_function(AP_SERVO_1), 2000);//k_motor 4
    SRV_Channels::set_angle(SRV_Channels::get_motor_function(AP_SERVO_2), 2000);//k_motor 5
    SRV_Channels::set_angle(SRV_Channels::get_motor_function(AP_SERVO_3), 5500);//k_motor 6

    _mav_type = MAV_TYPE_QUADROTOR;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_2X3);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_Motors2X3::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    set_initialised_ok((frame_class == MOTOR_FRAME_2X3));
}

// set update rate to motors - a value in hertz
void AP_Motors2X3::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 3 motors (but not the servo on channel 7)
    uint32_t mask = 
	    1U << AP_MOTORS_MOT_1 |
	    1U << AP_MOTORS_MOT_2;
    rc_set_freq(mask, _speed_hz);
}

void AP_Motors2X3::output_to_motors()
{
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            // sends minimum values out to the motors
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(0));
            rc_write(AP_MOTORS_MOT_2, output_to_pwm(0));
            rc_write_angle(AP_SERVO_1,  0);
            rc_write_angle(AP_SERVO_2,  0);
            rc_write_angle(AP_SERVO_3, -3500);
            break;
        case SpoolState::GROUND_IDLE:
            // sends output to motors when armed but not flying
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[2], actuator_spin_up_to_ground_idle());
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[1]));
            rc_write(AP_MOTORS_MOT_2, output_to_pwm(_actuator[2]));
            rc_write_angle(AP_SERVO_1,  0);
            rc_write_angle(AP_SERVO_2,  0);
            rc_write_angle(AP_SERVO_3, -3500);
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // set motor output based on thrust requests
            set_actuator_with_slew(_actuator[1], thr_lin.thrust_to_actuator(_m1_out));
            set_actuator_with_slew(_actuator[2], thr_lin.thrust_to_actuator(_m2_out));
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[1]));
            rc_write(AP_MOTORS_MOT_2, output_to_pwm(_actuator[2]));
            rc_write_angle(AP_SERVO_1, degrees(_s1_out)*100);
            rc_write_angle(AP_SERVO_2, degrees(_s2_out)*100);
            rc_write_angle(AP_SERVO_3, degrees(_s3_out)*100);
            break;
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_Motors2X3::get_motor_mask()
{
    // tri copter uses channels 1,2,4 and 7
    uint32_t motor_mask = (1U << AP_MOTORS_MOT_1) |
                          (1U << AP_MOTORS_MOT_2);
    uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);

    // add parent's mask
    mask |= AP_MotorsMulticopter::get_motor_mask();

    return mask;
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_Motors2X3::output_armed_stabilizing()
{
    float SQ2 = 1.2f;
    float L_c = 0.02f;// dist from servo axis to mass center
    float L_arm = 0.10f;//dist from servo axis to small servo axis
    const float compensation_gain = thr_lin.get_compensation_gain();

    // throttle_avg_max = _throttle_avg_max * compensation_gain;

    float fx_in = _forward_in * compensation_gain;
    float fz_in = get_throttle() * compensation_gain;
    float mx_in = (_roll_in + _roll_in_ff) * compensation_gain;
    float my_in = (_pitch_in + _pitch_in_ff) * compensation_gain;
    float mz_in = (_yaw_in + _yaw_in_ff) * compensation_gain;

    float k_forward = 0.85f/SQ2;
    float k_up = 0.75f/SQ2;
    float k_roll = 0.25f/SQ2;
    float k_pitch = 1.0f;
    float k_yaw = 0.15f/SQ2;

    float t1_x = k_forward * fx_in + k_yaw * mz_in;
    float t1_y = k_up * fz_in      + k_roll * mx_in;

    float t2_x = k_forward * fx_in - k_yaw * mz_in;
    float t2_y = k_up * fz_in      - k_roll * mx_in;

    float phi_1 = atan2f(t1_x, t1_y);
    float phi_2 = atan2f(t2_x, t2_y);
    float phi = (phi_1 + phi_2) * 0.5f;

    _m1_out = constrain_float(safe_sqrt(t1_x*t1_x + t1_y*t1_y), 0.0f, 1.0f);//0~1
    _m2_out = constrain_float(safe_sqrt(t2_x*t2_x + t2_y*t2_y), 0.0f, 1.0f);//0~1

    _s1_out = phi_1 - phi - k_pitch * my_in;
    _s2_out = phi_2 - phi - k_pitch * my_in;
    _s3_out = safe_asin(L_c*sinf(phi)/L_arm) + phi - radians(35.f); //0 value mean 45 across horizon

    if (fz_in < 0.04f) {
        _s1_out = 0.0f;
        _s2_out = 0.0f;
        _s3_out = 0.0f;
    }
}

float AP_Motors2X3::slew_servo(float old_s, float raw_s) {
    float slew_max_rad = radians(200.f/400.f);
    float s = old_s + constrain_float(raw_s - old_s, -slew_max_rad, slew_max_rad);
    return s;
}

float AP_Motors2X3::slew_motor_with_servo(float new_s, float old_s, float raw_s, float old_m, float raw_m) {
    float m = raw_m;
    float delta_s_new = new_s - old_s;
    float delta_s_raw = raw_s - old_s;
    if (is_zero(delta_s_new) || is_zero(delta_s_raw) || (delta_s_new > delta_s_raw)) {
        m = raw_m;
    } else {
        m = old_m + delta_s_new/delta_s_raw*(raw_m - old_m);
    }
    return m;
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_Motors2X3::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // front right motor
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            // back right motor
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        case 3:
            // front right servo
            rc_write(AP_SERVO_1, pwm);
            break;
        case 4:
            // back right servo
            rc_write(AP_SERVO_2, pwm);
            break;
        case 5:
            // back left servo
            rc_write(AP_SERVO_3, pwm);
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
void AP_Motors2X3::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        ;
    }
}

/*
  override tricopter tail servo output in output_motor_mask
 */
void AP_Motors2X3::output_motor_mask(float thrust, uint16_t mask, float rudder_dt)
{
    // normal multicopter output
    AP_MotorsMulticopter::output_motor_mask(thrust, mask, rudder_dt);

    // and override yaw servo
    rc_write_angle(AP_SERVO_1,  0);
    rc_write_angle(AP_SERVO_2,  0);
    rc_write_angle(AP_SERVO_3,  5500);
}

