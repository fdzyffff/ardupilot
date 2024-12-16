#include "Plane.h"

void Plane::user_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    uk230.init();
}

void Plane::user_100Hz() {
    uk230.update();
}

void Plane::user_50Hz() {
    ubase.update();
    useruartfwd.update();
}

void Plane::user_1Hz() {
    // put your 1Hz code here
    if ((g2.user_cam_print.get() & (1<<0)) && uk230.display_info.new_data) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f,%0.0f,%0.0f,%0.0f", uk230.display_info.count, uk230.display_info.p1, uk230.display_info.p2, uk230.display_info.p3, uk230.display_info.p4);
        uk230.display_info.new_data = false;
        uk230.display_info.count = 0;
    }
    if (g2.user_cam_print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "Corr (%0.0f,%0.0f,%0.0f) on:%d", uk230.display_info.p11, uk230.display_info.p12, uk230.display_info.p13, uk230.is_valid());
    }
    if (g2.user_cam_print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_WARNING, "rpy (%0.1f,%0.1f,%0.1f)", uk230.get_target_roll_rate(), uk230.get_target_pitch_rate(), uk230.get_target_yaw_rate());
    }
    if (g2.user_cam_print.get() & (1<<3)) { // 8
        gcs().send_text(MAV_SEVERITY_WARNING, "xyd (%0.1f,%0.1f,%0.1f)", uk230.get_target_bf_vel_x(), uk230.get_target_bf_vel_y(), uk230.get_target_dist_cm());
    }
}

bool Plane::allow_to_land() {
    if (!rc().has_valid_input()) {
        return true;
    } else {
        const RC_Channel *tchan = rc().channel(plane.rcmap.throttle()-1);
        if (tchan == nullptr) {
            return true;
        }
        float tval = (tchan->norm_input_ignore_trim()+1.0f)*0.5f;
        if (tval >= 0.4f) {
            return false;
        } else {
            return true;
        }
    }
    return true;
}


// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Plane::position_ok() 
{
    // check ekf position estimate
    return (ekf_has_absolute_position() || ekf_has_relative_position());
}

// ekf_has_absolute_position - returns true if the EKF can provide an absolute WGS-84 position estimate
bool Plane::ekf_has_absolute_position() 
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status;
    ahrs.get_filter_status(filt_status);

    // if disarmed we accept a predicted horizontal position
    if (!arming.is_armed()) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    } else {
        // once armed we require a good absolute position and EKF must not be in const_pos_mode
        return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
    }
}

// ekf_has_relative_position - returns true if the EKF can provide a position estimate relative to it's starting position
bool Plane::ekf_has_relative_position() 
{
    // return immediately if EKF not used
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // get filter status from EKF
    nav_filter_status filt_status;
    ahrs.get_filter_status(filt_status);

    // if disarmed we accept a predicted horizontal relative position
    if (!arming.is_armed()) {
        return (filt_status.flags.pred_horiz_pos_rel);
    } else {
        return (filt_status.flags.horiz_pos_rel && !filt_status.flags.const_pos_mode);
    }
}
