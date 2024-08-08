#include "Copter.h"

// attack_vel_init - initialise attack_vel controller
bool ModeAttackVel::init(bool ignore_checks)
{
    // initialise position and desired velocity
    if (copter.mode_guided.init(false)) {
        if (copter.ugimbal.is_valid()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "In Attack Vel");
            // initialise horizontal speed, acceleration
            pos_control->set_max_speed_accel_xy(copter.g2.user_parameters.max_vel_xy.get(), wp_nav->get_wp_acceleration());
            pos_control->set_correction_speed_accel_xy(copter.g2.user_parameters.max_vel_xy.get(), wp_nav->get_wp_acceleration());

            // initialize vertical speeds and acceleration
            pos_control->set_max_speed_accel_z(copter.g2.user_parameters.max_vel_z.get(), copter.g2.user_parameters.max_vel_z.get(), wp_nav->get_accel_z());
            pos_control->set_correction_speed_accel_z(copter.g2.user_parameters.max_vel_z.get(), copter.g2.user_parameters.max_vel_z.get(), wp_nav->get_accel_z());

            // initialise velocity controller
            pos_control->init_z_controller();
            pos_control->init_xy_controller();
            return true;
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING, "No Target");
            return false;
        }
    }

    return false;
}

// attack_vel_run - runs the attack_vel controller
// should be called at 100hz or more
void ModeAttackVel::run()
{
   // gcs().send_text(MAV_SEVERITY_WARNING, "ModeAttackVel run");
    static uint32_t _last_ms = 0;
    uint32_t delta_t = millis() - _last_ms;
    if (delta_t > 20 ) {
        bool use_yaw = true;
        float yaw_cd = 0.0f;
        bool use_yaw_rate = false;
        float yaw_rate_cds = 0.0f;
        bool relative_yaw = false;

        yaw_cd = copter.ugimbal.get_target_yaw_cd();
        float pitch_cd = copter.ugimbal.get_target_pitch_cd();
        Matrix3f tmp_m;
        tmp_m.from_euler(0.0f, radians(pitch_cd*0.01f), radians(yaw_cd*0.01f));

        // vel control
        Vector3f tmp_input = Vector3f(1.0f, 0.0f, 0.0f);
        Vector3f tmp_output = tmp_m*tmp_input;
        float target_vel_nolim = copter.g2.user_parameters.attack_vel;
        float vel_z_norm = 1.0f;
        float vel_z_nolim = tmp_output.z * target_vel_nolim;
        float vel_z_max = MAX(copter.g2.user_parameters.max_vel_z.get(), 100.0f);
        if (vel_z_nolim > vel_z_max) {
            vel_z_norm = vel_z_max/vel_z_nolim;
        }
        float target_vel = target_vel_nolim*vel_z_norm;
        Vector3f vel_output = tmp_output*target_vel;
        vel_output.z *= -1.0f;
 
        // copter.mode_guided.set_velocity(tmp_output, use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw, copter.ugimbal.new_data());

        // posvel control, vel ff is zero
        // Vector3f tmp_output = Vector3f(copter.ugimbal.get_target_vel().x*vel_yaw_factor, copter.ugimbal.get_target_vel().y*vel_yaw_factor,copter.ugimbal.get_target_vel().z*vel_yaw_factor);
        copter.mode_guided.set_velaccel(vel_output, Vector3f(0.0f, 0.0f, 0.0f), use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

        _last_ms = millis();
    }
    copter.mode_guided.run();
    if (!copter.ugimbal.is_valid()) {
         gcs().send_text(MAV_SEVERITY_WARNING, "Target status LOITER");
        if (!copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END)) {
            copter.set_mode(Mode::Number::POSHOLD, ModeReason::MISSION_END);
        }
    }
}

uint32_t ModeAttackVel::wp_distance() const
{
    return 0;
}

int32_t ModeAttackVel::wp_bearing() const
{
    return pos_control->get_bearing_to_target_cd();
}
