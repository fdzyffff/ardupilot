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

#include "AP_Motors4X4.h"

extern const AP_HAL::HAL& hal;

// init
void AP_Motors4X4::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    add_motor_num(AP_MOTORS_MOT_1);
    add_motor_num(AP_MOTORS_MOT_2);
    add_motor_num(AP_MOTORS_MOT_3);
    add_motor_num(AP_MOTORS_MOT_4);

    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;


    // allow mapping of servo 1~4
    add_motor_num(AP_SERVO_1);
    add_motor_num(AP_SERVO_2);
    add_motor_num(AP_SERVO_3);
    add_motor_num(AP_SERVO_4);

    SRV_Channels::set_angle(SRV_Channels::get_motor_function(AP_SERVO_1), 4500);//k_motor 5
    SRV_Channels::set_angle(SRV_Channels::get_motor_function(AP_SERVO_2), 4500);//k_motor 6
    SRV_Channels::set_angle(SRV_Channels::get_motor_function(AP_SERVO_3), 4500);//k_motor 7
    SRV_Channels::set_angle(SRV_Channels::get_motor_function(AP_SERVO_4), 4500);//k_motor 8

    _mav_type = MAV_TYPE_QUADROTOR;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_4X4);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_Motors4X4::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    set_initialised_ok((frame_class == MOTOR_FRAME_4X4));
}

// set update rate to motors - a value in hertz
void AP_Motors4X4::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 3 motors (but not the servo on channel 7)
    uint32_t mask = 
	    1U << AP_MOTORS_MOT_1 |
	    1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3 |
	    1U << AP_MOTORS_MOT_4;
    rc_set_freq(mask, _speed_hz);
}

void AP_Motors4X4::output_to_motors()
{
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            // sends minimum values out to the motors
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(0));
            rc_write(AP_MOTORS_MOT_2, output_to_pwm(0));
            rc_write(AP_MOTORS_MOT_3, output_to_pwm(0));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(0));
            rc_write_angle(AP_SERVO_1, 0);
            rc_write_angle(AP_SERVO_2, 0);
            rc_write_angle(AP_SERVO_3, 0);
            rc_write_angle(AP_SERVO_4, 0);
            break;
        case SpoolState::GROUND_IDLE:
            // sends output to motors when armed but not flying
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[2], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[3], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[4], actuator_spin_up_to_ground_idle());
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[1]));
            rc_write(AP_MOTORS_MOT_2, output_to_pwm(_actuator[2]));
            rc_write(AP_MOTORS_MOT_3, output_to_pwm(_actuator[3]));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[4]));
            rc_write_angle(AP_SERVO_1, 0);
            rc_write_angle(AP_SERVO_2, 0);
            rc_write_angle(AP_SERVO_3, 0);
            rc_write_angle(AP_SERVO_4, 0);
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // set motor output based on thrust requests
            set_actuator_with_slew(_actuator[1], thrust_to_actuator(_m1_out));
            set_actuator_with_slew(_actuator[2], thrust_to_actuator(_m2_out));
            set_actuator_with_slew(_actuator[3], thrust_to_actuator(_m3_out));
            set_actuator_with_slew(_actuator[4], thrust_to_actuator(_m4_out));
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(_m1_out));
            rc_write(AP_MOTORS_MOT_2, output_to_pwm(_m2_out));
            rc_write(AP_MOTORS_MOT_3, output_to_pwm(_m3_out));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(_m4_out));
            rc_write_angle(AP_SERVO_1, degrees(_s1_out)*100);
            rc_write_angle(AP_SERVO_2, degrees(_s2_out)*100);
            rc_write_angle(AP_SERVO_3, degrees(_s3_out)*100);
            rc_write_angle(AP_SERVO_4, degrees(_s4_out)*100);
            break;
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_Motors4X4::get_motor_mask()
{
    // tri copter uses channels 1,2,4 and 7
    uint32_t motor_mask = (1U << AP_MOTORS_MOT_1) |
                          (1U << AP_MOTORS_MOT_2) |
                          (1U << AP_MOTORS_MOT_3) |
                          (1U << AP_MOTORS_MOT_4);
    uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);

    // add parent's mask
    mask |= AP_MotorsMulticopter::get_motor_mask();

    return mask;
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_Motors4X4::output_armed_stabilizing()
{
    
    float SQ2 = 1.414f;
    float Length = 0.2f;
    float MASS = 1.0f;
    const float compensation_gain = get_compensation_gain();


    // throttle_avg_max = _throttle_avg_max * compensation_gain;

    float fx_in = _forward_in * compensation_gain;
    float fy_in = _lateral_in * compensation_gain;
    float fz_in = get_throttle() * compensation_gain;
    float mx_in = (_roll_in + _roll_in_ff) * compensation_gain;
    float my_in = (_pitch_in + _pitch_in_ff) * compensation_gain;
    float mz_in = (_yaw_in + _yaw_in_ff) * compensation_gain;
    
    float t1_y_out = 0.25f*(-SQ2*fx_in + SQ2*fy_in + 0.0f + 0.0f + 0.0f + 1.0f/Length*mz_in);
    float t2_y_out = 0.25f*(-SQ2*fx_in - SQ2*fy_in + 0.0f + 0.0f + 0.0f + 1.0f/Length*mz_in);
    float t3_y_out = 0.25f*( SQ2*fx_in - SQ2*fy_in + 0.0f + 0.0f + 0.0f + 1.0f/Length*mz_in);
    float t4_y_out = 0.25f*( SQ2*fx_in + SQ2*fy_in + 0.0f + 0.0f + 0.0f + 1.0f/Length*mz_in);
    float t1_x_out = 0.25f*( 0.0f + 0.0f - fz_in - SQ2/Length*mx_in + SQ2/Length*my_in + 0.0f);
    float t2_x_out = 0.25f*( 0.0f + 0.0f - fz_in - SQ2/Length*mx_in - SQ2/Length*my_in + 0.0f);
    float t3_x_out = 0.25f*( 0.0f + 0.0f - fz_in - SQ2/Length*mx_in - SQ2/Length*my_in + 0.0f);
    float t4_x_out = 0.25f*( 0.0f + 0.0f - fz_in + SQ2/Length*mx_in + SQ2/Length*my_in + 0.0f);

    _m1_out = safe_sqrt(t1_y_out*t1_y_out + t1_x_out*t1_x_out)/(2.0f*MASS*9.8f);//0~1
    _m2_out = safe_sqrt(t2_y_out*t2_y_out + t2_x_out*t2_x_out)/(2.0f*MASS*9.8f);;//0~1
    _m3_out = safe_sqrt(t3_y_out*t3_y_out + t3_x_out*t3_x_out)/(2.0f*MASS*9.8f);;//0~1
    _m4_out = safe_sqrt(t4_y_out*t4_y_out + t4_x_out*t4_x_out)/(2.0f*MASS*9.8f);;//0~1
    _s1_out = atan2(t1_y_out, t1_y_out);//0~1
    _s2_out = atan2(t2_y_out, t2_y_out);//0~1
    _s3_out = atan2(t3_y_out, t3_y_out);//0~1
    _s4_out = atan2(t4_y_out, t4_y_out);//0~1
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_Motors4X4::_output_test_seq(uint8_t motor_seq, int16_t pwm)
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
            // back left motor
            rc_write(AP_MOTORS_MOT_3, pwm);
            break;
        case 4:
            // front left motor
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        case 5:
            // front right servo
            rc_write(AP_SERVO_1, pwm);
            break;
        case 6:
            // back right servo
            rc_write(AP_SERVO_2, pwm);
            break;
        case 7:
            // back left servo
            rc_write(AP_SERVO_3, pwm);
            break;
        case 8:
            // front left servo
            rc_write(AP_SERVO_4, pwm);
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
void AP_Motors4X4::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        ;
    }
}

/*
  override tricopter tail servo output in output_motor_mask
 */
void AP_Motors4X4::output_motor_mask(float thrust, uint16_t mask, float rudder_dt)
{
    // normal multicopter output
    AP_MotorsMulticopter::output_motor_mask(thrust, mask, rudder_dt);

    // and override yaw servo
    rc_write_angle(AP_SERVO_1, 0);
    rc_write_angle(AP_SERVO_2, 0);
    rc_write_angle(AP_SERVO_3, 0);
    rc_write_angle(AP_SERVO_4, 0);
}

