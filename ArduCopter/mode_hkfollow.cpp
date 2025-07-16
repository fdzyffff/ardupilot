#include "Copter.h"

#if MODE_GUIDED_ENABLED

// init - initialise guided controller
bool ModeHKFollow::init(bool ignore_checks)
{
    if (copter.mode_guided.init(ignore_checks)) {
        if (copter.motors->armed() && !copter.ap.land_complete) {
            set_state(FOLLOW_State::Wait);
        } else {
            set_state(FOLLOW_State::Init);
        }
        gcs().send_text(MAV_SEVERITY_INFO, "In Flw MODE");
        follow_velocity.zero();
        follow_acceleration.zero();
        dist_cm_filt.set_cutoff_frequency(copter.g2.rangefinder_filt);
        return true;
    }
    return false;
}

// run - runs the guided controller
// should be called at 100hz or more
void ModeHKFollow::run()
{
    read_dist();
    update_state();

    switch (follow_State) {
        case FOLLOW_State::Init:
        {
            copter.mode_guided.run();
        }
        break;
        case FOLLOW_State::Takeoff:
        {
            copter.mode_guided.run();
        }
        break;
        case FOLLOW_State::Wait:
        {
            copter.mode_guided.run();
        }
        break;
        case FOLLOW_State::Search:
        {
            copter.mode_guided.run();
        }
        break;
        case FOLLOW_State::Track:
        {
            copter.mode_guided.run();
        }
        break;
        break;
    }
}

void ModeHKFollow::update_state()
{

    switch (follow_State) {
        case FOLLOW_State::Init:
        {
            if (copter.motors->armed() && copter.ap.land_complete) {
                set_state(FOLLOW_State::Takeoff);
            }
        }
        break;
        case FOLLOW_State::Takeoff:
        {
            if (copter.mode_guided.submode() != ModeGuided::SubMode::TakeOff) {
                set_state(FOLLOW_State::Wait);
            }
            if (copter.mode_guided.takeoff_complete) {
                set_state(FOLLOW_State::Wait);
            }
        }
        break;
        case FOLLOW_State::Wait:
        {
            if (copter.uattack.is_active() && dist_healthy) {
                set_state(FOLLOW_State::Track);
                cal_follow_info();
            } else {
                cal_wait_info();
            }
        }
        break;
        case FOLLOW_State::Search:
        {
            if (copter.uattack.is_active() && dist_healthy) {
                set_state(FOLLOW_State::Track);
                cal_follow_info();
            } else {
                cal_search_info();
            }
        }
        break;
        case FOLLOW_State::Track:
        {
            if (copter.uattack.is_active() && dist_healthy) {
                cal_follow_info();
            } else {
                set_state(FOLLOW_State::Search);
            }
        }
        break;
    }
}

void ModeHKFollow::set_state(FOLLOW_State state_in)
{
    if (follow_State == state_in) {
        return;
    }

    switch (state_in) {
        case FOLLOW_State::Init:
        {
            follow_State = state_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[FLW] State: Init");
        }
        break;
        case FOLLOW_State::Takeoff:
        {
            if (copter.mode_guided.do_user_takeoff_start(120.f)) {
                copter.set_auto_armed(true);
                follow_State = state_in;
                gcs().send_text(MAV_SEVERITY_INFO, "[FLW] State: Takeoff");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[FLW] State: Takeoff Fail");
            }
        }
        break;
        case FOLLOW_State::Wait:
        {
            copter.mode_guided.velaccel_control_start();
            follow_State = state_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[FLW] State: Wait");
        }
        break;
        case FOLLOW_State::Search:
        {
            if (copter.mode_circle.init(false)) {
                follow_State = state_in;
                gcs().send_text(MAV_SEVERITY_INFO, "[FLW] State: Search");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "[FLW] State: Search");
            }
        }
        break;
        case FOLLOW_State::Track:
        {
            copter.mode_guided.velaccel_control_start();
            follow_State = state_in;
            gcs().send_text(MAV_SEVERITY_INFO, "[FLW] State: Track");
        }
        break;
        default:
        break;
    }
}

bool ModeHKFollow::is_taking_off() const
{
    return follow_State == FOLLOW_State::Takeoff;
}

void ModeHKFollow::cal_follow_info()
{
    if (copter.uattack.is_active() && dist_healthy) {
        float dist = dist_cm_filt.get();
        float dist_error = constrain_float(dist - 250.f, -50.0f, 50.0f);
        float kp = 1.0f;
        Vector3f body_vel;
        body_vel.x = kp * dist_error;
        body_vel.y = 30.f;
        body_vel.z = 0.0f;

        Matrix3f tmp_body_earth_m;
        tmp_body_earth_m.from_euler(0.0f, 0.0f, AP::ahrs().get_yaw());
        follow_velocity = tmp_body_earth_m * body_vel;
        use_yaw = false;
        yaw_cd = degrees(AP::ahrs().get_yaw())*100.f;
        use_yaw_rate = true;
        yaw_rate_cds = copter.uattack.get_target_yaw_rate()*100.f;
        log_request = true;
        if (millis() - copter.mode_guided.my_update_time_ms > 100) {
            copter.mode_guided.set_velaccel(follow_velocity, follow_acceleration, use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw, log_request);
        }
    }
}

void ModeHKFollow::cal_wait_info()
{
    follow_velocity.zero();
    follow_velocity.zero();
    use_yaw = false;
    yaw_cd = degrees(AP::ahrs().get_yaw())*100.f;
    use_yaw_rate = true;
    yaw_rate_cds = 0.0f;
    log_request = true;
    if (millis() - copter.mode_guided.my_update_time_ms > 100) {
        copter.mode_guided.set_velaccel(follow_velocity, follow_acceleration, use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw, log_request);
    }
}


void ModeHKFollow::cal_search_info()
{
    follow_velocity.zero();
    follow_velocity.zero();
    use_yaw = false;
    yaw_cd = degrees(AP::ahrs().get_yaw())*100.f;
    use_yaw_rate = true;
    yaw_rate_cds = -1500.f;
    log_request = true;
    if (millis() - copter.mode_guided.my_update_time_ms > 100) {
        copter.mode_guided.set_velaccel(follow_velocity, follow_acceleration, use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw, log_request);
    }
}

void ModeHKFollow::read_dist()
{

    const uint32_t now = AP_HAL::millis();

    if (now - last_test_ms < 10000) {
        dist_healthy = true;
        return;
    }

    const RangeFinder *rangefinder = RangeFinder::get_singleton();

    if (rangefinder == nullptr) {
        dist_healthy = false;
        return;
    }

    // do not run too fast
    if (now - last_healthy_ms < 100) {
        return;
    }

    dist_healthy = (rangefinder->status_orient(ROTATION_NONE) == RangeFinder::Status::Good) &&
                            (rangefinder->range_valid_count_orient(ROTATION_NONE) >= 3);


    // tilt corrected but unfiltered, not glitch protected alt
    float tilt_correction = sinf(fabsf(AP::ahrs().get_pitch()));
    dist_cm = tilt_correction * rangefinder->distance_cm_orient(ROTATION_NONE);

    // filter rangefinder altitude
    const bool timed_out = now - last_healthy_ms > 1000;
    if (dist_healthy) {
        if (timed_out) {
            // reset filter if we haven't used it within the last second
            dist_cm_filt.reset(dist_cm);
        } else {
            // TODO: When we apply this library in plane we will need to be able to set the filter freq
            dist_cm_filt.apply(dist_cm, 0.05);
        }
        last_healthy_ms = now;
    }
}

void ModeHKFollow::handle_message(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        // decode packet
        // gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(&msg, &packet);
        switch(packet.command) {
            case MAV_CMD_USER_1:
                {
                    last_test_ms = millis();
                    dist_cm = packet.param1;
                    dist_cm_filt.apply(dist_cm, 0.05);
                }
                break;
            default:
                break;
        }
    }
}

uint32_t ModeHKFollow::wp_distance() const
{
    switch(follow_State) {
        case FOLLOW_State::Init:
        case FOLLOW_State::Takeoff:
        case FOLLOW_State::Wait:
        case FOLLOW_State::Search:
        case FOLLOW_State::Track:
            return copter.mode_guided.wp_distance();
            break;
        default:
            return 0;
            break;
    }
    return 0;
}

int32_t ModeHKFollow::wp_bearing() const
{
    switch(follow_State) {
        case FOLLOW_State::Init:
        case FOLLOW_State::Takeoff:
        case FOLLOW_State::Wait:
        case FOLLOW_State::Search:
        case FOLLOW_State::Track:
            return copter.mode_guided.wp_bearing();
            break;
        default:
            return 0;
            break;
    }
    return 0;
}

float ModeHKFollow::crosstrack_error() const
{
    switch(follow_State) {
        case FOLLOW_State::Init:
        case FOLLOW_State::Takeoff:
        case FOLLOW_State::Wait:
        case FOLLOW_State::Search:
        case FOLLOW_State::Track:
            return copter.mode_guided.crosstrack_error();
            break;
        default:
            return 0;
            break;
    }
    return 0;
}
#endif