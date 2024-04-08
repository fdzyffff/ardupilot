/// @file	AP_Motors4X4.h
/// @brief	Motor control class for Tricopters
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include "AP_MotorsMulticopter.h"

// tilt servos 1~4
#define AP_SERVO_1 CH_5
#define AP_SERVO_2 CH_6
#define AP_SERVO_3 CH_7
#define AP_SERVO_4 CH_8

/// @class      AP_Motors4X4
class AP_Motors4X4 : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_Motors4X4(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(speed_hz)
    {
    };

    // init
    void                init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set update rate to motors - a value in hertz
    void                set_update_rate( uint16_t speed_hz ) override;

    // output_to_motors - sends minimum values out to the motors
    virtual void        output_to_motors() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint32_t            get_motor_mask() override;

    // output a thrust to all motors that match a given motor
    // mask. This is used to control tiltrotor motors in forward
    // flight. Thrust is in the range 0 to 1
    // rudder_dt applys diffential thrust for yaw in the range 0 to 1
    void                output_motor_mask(float thrust, uint16_t mask, float rudder_dt) override;

    // return the roll factor of any motor, this is used for tilt rotors and tail sitters
    // using copter motors for forward flight
    float               get_roll_factor(uint8_t i) override;

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing() override;

    // call vehicle supplied thrust compensation if set
    void                thrust_compensation(void) override;

    const char* _get_frame_string() const override { return "4X4"; }

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    // parameters

    float           _m1_out;
    float           _m2_out;
    float           _m3_out;
    float           _m4_out;
    float           _s1_out;
    float           _s2_out;
    float           _s3_out;
    float           _s4_out;
};
