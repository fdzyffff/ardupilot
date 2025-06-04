#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAttack::init(bool ignore_checks)
{

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    if (is_disarmed_or_landed()) {
        set_stage(Stage::TAKEOFF);
    } else {
        if (copter.uattack.is_active()) {
            set_stage(Stage::ATTACK);
        } else {
            set_stage(Stage::LAND);
        }
    }

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAttack::run()
{
    if (_stage == Stage::TAKEOFF) {
        takeoff_run();
    } else if (_stage == Stage::ATTACK) {
        attack_run();
    } else {
        land_run();
    }
    // put at end because init will reset takeoff running flag
    update_stage();
}

void ModeAttack::takeoff_run()
{
    if (!takeoff.running()) {
        takeoff.start(constrain_float(g.pilot_takeoff_alt,50.0f,100.0f));
        gcs().send_text(MAV_SEVERITY_INFO, "Takeoff start %d", takeoff.running());
    }

    if (motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // get avoidance adjusted climb rate
    float target_climb_rate = 50.0f;

    // set position controller targets adjusted for pilot input
    takeoff.do_pilot_takeoff(target_climb_rate);

    float target_yaw_cd = 0.0f;
    target_yaw_cd = copter.uattack.get_ef_info().x*100.f;

    // call attitude controller with auto yaw
    attitude_control->input_euler_angle_roll_pitch_yaw(0.0f, 0.0f, target_yaw_cd, false);

    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
                // gcs().send_text(MAV_SEVERITY_INFO, "takeoff_run");
}

void ModeAttack::attack_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    float target_roll_cd = 0.0f;
    float target_pitch_cd = 0.0f;
    float target_yaw_cd = 0.0f;

    target_roll_cd = copter.uattack.get_target_roll_angle()*100.f;
    target_pitch_cd = -1500.f;
    target_yaw_cd = copter.uattack.get_ef_info().x*100.f;
    // call attitude controller with auto yaw
    attitude_control->input_euler_angle_roll_pitch_yaw(target_roll_cd, target_pitch_cd, target_yaw_cd, false);

    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

void ModeAttack::land_run() {

    // call attitude controller with auto yaw
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(-20.0f);
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

}

void ModeAttack::set_stage(Stage stage_in) {
    _stage = stage_in;
    _stage_time = millis();
    switch (_stage) {
        case Stage::TAKEOFF:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage Takeoff");
            break;
        case Stage::ATTACK:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage ATTACK");
            break;
        case Stage::LAND:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage LAND");
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "Stage UNKNOWN");
            break;
    }
}

void ModeAttack::update_stage()
{
    // uint32_t dt = millis() - _stage_time;
    switch (_stage) {
        case Stage::TAKEOFF:
            {
            // gcs().send_text(MAV_SEVERITY_INFO, "update_stage %d", takeoff.running());
                if (!takeoff.running()) {
                    if (copter.uattack.is_active()) {
                        set_stage(Stage::ATTACK);
                    } else {
                        set_stage(Stage::LAND);
                    }
                }
            }
            break;
        case Stage::ATTACK:
            break;
        case Stage::LAND:
            break;
        default:
            break;
    }
}

bool ModeAttack::is_taking_off() const
{
    return ((_stage == Stage::TAKEOFF) && takeoff.running());
}
