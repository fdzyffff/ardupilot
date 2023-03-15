#include "Plane.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/*
  main stabilization function for all 3 axes
 */
void Plane::sub_stabilize()
{
    static uint32_t sub_last_stabilize_ms = 0;

    float speed_scaler = 1.0f; //get_speed_scaler();

    uint32_t now = AP_HAL::millis();

    if (now - sub_last_stabilize_ms > 2000) {
        // if we haven't run the rate controllers for 2 seconds then
        // reset the integrators
        as_rollController.reset_I();
        as_pitchController.reset_I();
        as_yawController.reset_I();
    }
    sub_last_stabilize_ms = now;

    sub_stabilize_roll(speed_scaler);
    sub_stabilize_pitch(speed_scaler);
    sub_stabilize_yaw(speed_scaler);

    /*
      see if we should zero the attitude controller integrators. 
     */
    if (is_zero(get_throttle_input()))  {
        // we are low, with no climb rate, and zero throttle, and very
        // low ground speed. Zero the attitude controller
        // integrators. This prevents integrator buildup pre-takeoff.
        as_rollController.reset_I();
        as_pitchController.reset_I();
        as_yawController.reset_I();
    }
}

/*
  this is the main roll stabilization function. It takes the
  previously set nav_roll calculates roll servo_out to try to
  stabilize the plane at the given roll
 */
void Plane::sub_stabilize_roll(float speed_scaler)
{
    const float roll_out = sub_stabilize_roll_get_roll_out(speed_scaler);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll_out);
}

float Plane::sub_stabilize_roll_get_roll_out(float speed_scaler)
{
    bool disable_integrator = false;
    return as_rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor, speed_scaler, disable_integrator, false);
}

/*
  this is the main pitch stabilization function. It takes the
  previously set nav_pitch and calculates servo_out values to try to
  stabilize the plane at the given attitude.
 */
void Plane::sub_stabilize_pitch(float speed_scaler)
{
    const float pitch_out = sub_stabilize_pitch_get_pitch_out(speed_scaler);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch_out);
}

float Plane::sub_stabilize_pitch_get_pitch_out(float speed_scaler)
{
    bool disable_integrator = false;
    int32_t demanded_pitch = nav_pitch_cd + g.sub_pitch_trim_cd + SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * g.kff_throttle_to_pitch;
    return as_pitchController.get_servo_out(demanded_pitch - ahrs.pitch_sensor, speed_scaler, disable_integrator, false);
}

/*
  stabilize the yaw axis. There are 3 modes of operation:

    - yaw control for coordinated flight    
 */
void Plane::sub_stabilize_yaw(float speed_scaler)
{
    /*
      now calculate steering_control.rudder for the rudder
     */
    sub_calc_nav_yaw_coordinated(speed_scaler);
}

/*
  calculate yaw control for coordinated flight
 */
void Plane::sub_calc_nav_yaw_coordinated(float speed_scaler)
{
    bool disable_integrator = false;
    int16_t rudder_in = rudder_input();

    int16_t commanded_rudder;
    bool using_rate_controller = false;

    commanded_rudder = as_yawController.get_servo_out(speed_scaler, disable_integrator);

    // add in rudder mixing from roll
    commanded_rudder += SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) * g.kff_rudder_mix;
    commanded_rudder += rudder_in;

    steering_control.rudder = constrain_int16(commanded_rudder, -4500, 4500);

    if (!using_rate_controller) {
        /*
          When not running the yaw rate controller, we need to reset the rate
        */
        as_yawController.reset_rate_PID();
    }
}
