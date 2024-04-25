#include "Copter.h"


#if MODE_GUIDED_ENABLED == ENABLED

// initialise follow mode
bool ModeAttack::init(const bool ignore_checks)
{
    if (!copter.ugimbal.is_valid()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "No target");
    }
    // re-use guided mode
    return ModeGuided::init(ignore_checks);
}

// perform cleanup required when leaving follow mode
void ModeAttack::exit()
{
    ;
}

void ModeAttack::run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // re-use guided mode's velocity controller
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode

    // variables to be sent to velocity controller
    Vector3f desired_velocity_neu_cms;
    bool use_yaw = false;
    float yaw_cd = 0.0f;

    Vector3f dist_vec;  // vector to lead vehicle
    Vector3f dist_vec_offs;  // vector to lead vehicle + offset
    Vector3f vel_of_target;  // velocity of lead vehicle

    // log output at 10hz
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 100) || (last_log_ms == 0)) {
        log_request = true;
        last_log_ms = now;
    }
    // re-use guided mode's velocity controller (takes NEU)
    ModeGuided::set_velocity(desired_velocity_neu_cms, use_yaw, yaw_cd, false, 0.0f, false, log_request);

    ModeGuided::run();
}

uint32_t ModeAttack::wp_distance() const
{
    return copter.ugimbal.get_target_dist() * 100;
}

int32_t ModeAttack::wp_bearing() const
{
    return copter.ugimbal.get_target_yaw_cd();
}

/*
  get target position for mavlink reporting
 */
bool ModeAttack::get_wp(Location &loc) const
{
    float dist = copter.ugimbal.get_target_dist();
    float bearing = copter.ugimbal.get_target_yaw_cd()*0.01f;
    loc = copter.current_loc;
    loc.offset_bearing(bearing, dist);
    return true;
}

#endif // MODE_FOLLOW_ENABLED == ENABLED
