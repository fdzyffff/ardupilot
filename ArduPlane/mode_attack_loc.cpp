#include "mode.h"
#include "Plane.h"

bool ModeAttackLoc::_enter()
{
    _dist_src = 0;
    recover_alt_cm = 0;
    if (plane.uattack.is_active_cam()) {
        set_stage(stage_class::AIM); 
        gcs().send_text(MAV_SEVERITY_INFO, "Attack CAM!");
        return true;
    } else if (plane.uattack.is_active_loc()) {
        target_loc = plane.uattack._Target_ptr_loc->target_loc;
        if (check_approach()) {
            set_stage(stage_class::AIM);                
        } else {
            set_stage(stage_class::APPROACH);
            build_path();
            gcs().send_text(MAV_SEVERITY_INFO, "Attack LOC!");
        }
        return true;
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "No target, Can NOT attack!");
    }
    return false;
}

void ModeAttackLoc::run()
{
    switch (stage) {
        case stage_class::APPROACH:

            // Direct stick mixing functionality has been removed, so as not to remove all stick mixing from the user completely
            // the old direct option is now used to enable fbw mixing, this is easier than doing a param conversion.
            if ((plane.g.stick_mixing == StickMixing::FBW) || (plane.g.stick_mixing == StickMixing::DIRECT_REMOVED)) {
                plane.stabilize_stick_mixing_fbw();
            }
            plane.stabilize_roll();
            plane.stabilize_pitch();
            plane.stabilize_yaw();
            break;
        case stage_class::AIM:
            plane.stabilize_attack();
            break;
        case stage_class::AWAY:
            plane.stabilize_roll();
            plane.stabilize_pitch();
            plane.stabilize_yaw();
            break;
        case stage_class::RECOVER:
            plane.stabilize_roll();
            plane.stabilize_pitch();
            plane.stabilize_yaw();
            break;
        default:
            break;
    }
}

void ModeAttackLoc::build_path()
{
    // copy the current location into the OldWP slot
    // -----------------------------------------------
    plane.prev_WP_loc = plane.current_loc;

    // Load the next_WP slot
    // ---------------------
    plane.next_WP_loc = target_loc;
    plane.next_WP_loc.alt = plane.current_loc.alt;
    // always over target for a distance
    float bearing_cd = plane.current_loc.get_bearing_to(target_loc);
    plane.next_WP_loc.offset_bearing(bearing_cd*0.01f, 200);

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    plane.set_target_altitude_current();

    plane.setup_glide_slope();
    plane.setup_turn_angle();

    // disable crosstrack, head directly to the point
    plane.auto_state.crosstrack = false;

    // reset loiter start time.
    plane.loiter.start_time_ms = 0;

    // start in non-VTOL mode
    plane.auto_state.vtol_loiter = false;
    
    plane.loiter_angle_reset();
}

void ModeAttackLoc::update()
{
    switch (stage) {
        case stage_class::APPROACH:
            update_approach();
            if (!plane.uattack.is_active()) {
                set_stage(stage_class::RECOVER);
            } else if (check_approach()) {
                set_stage(stage_class::AIM);
            }
            break;
        case stage_class::AIM:
            update_attack();
            if (check_attack()) {
                set_stage(stage_class::AWAY);
            } else if (!plane.uattack.is_active()) {
                set_stage(stage_class::RECOVER);
            }
            break;
        case stage_class::AWAY:
            update_away();
            if (check_away()) {
                set_stage(stage_class::RECOVER);
            }
            break;
        case stage_class::RECOVER:
            update_recover();
            if (check_target()) {
                if (plane.uattack._Target_ptr_loc != nullptr && plane.uattack._Target_ptr_loc->is_valid()) {
                    target_loc = plane.uattack._Target_ptr_loc->target_loc;
                    set_stage(stage_class::APPROACH);
                    build_path();
                } else if (plane.uattack.is_active_cam()) {
                    set_stage(stage_class::AIM);
                }
            }
            break;
        default:
            break;
    }
}

void ModeAttackLoc::update_approach()
{
    // 目标会移动，每周期把最新目标点刷新到 next_WP（否则 APPROACH 追的是进入时的旧点）
    if (plane.uattack._Target_ptr_loc != nullptr && plane.uattack._Target_ptr_loc->is_valid()) {
        target_loc = plane.uattack._Target_ptr_loc->target_loc;
        build_path();
    }
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeAttackLoc::update_attack()
{
    float throtle_rate = plane.uattack.attack_throttle_rate*plane.G_Dt;
    float target_throttle = plane.uattack.attack_throttle;
    _cmd_throttle = _cmd_throttle + constrain_float(target_throttle - _cmd_throttle, -throtle_rate, throtle_rate);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.mode_attack_loc.get_cmd_throttle());
}

void ModeAttackLoc::update_away()
{
    plane.nav_roll_cd = (int32_t)(plane.uattack.away_roll.get() * 100.0f);
    plane.nav_pitch_cd = (int32_t)(plane.uattack.away_pitch.get() * 100.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.aparm.throttle_cruise);
}

void ModeAttackLoc::update_recover()
{
    // 目标会移动，每周期刷新盘旋中心
    if (plane.uattack._Target_ptr_loc != nullptr && plane.uattack._Target_ptr_loc->is_valid()) {
        Location loc = plane.uattack._Target_ptr_loc->target_loc;
        recover_center.lat = loc.lat;
        recover_center.lng = loc.lng;
        int32_t alt_cm = 0;
        if (loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm) && alt_cm != 0) {
            recover_alt_cm = alt_cm;
        }
    }
    plane.next_WP_loc = recover_center;
    plane.next_WP_loc.alt = recover_alt_cm;
    plane.set_target_altitude_location(plane.next_WP_loc);

    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

bool ModeAttackLoc::check_approach()
{
    bool valid = false;
    float dist = get_target_distance(valid);
    if (!valid) {
        return false;
    }
    return dist < plane.uattack.approach_dist.get();
}

bool ModeAttackLoc::check_attack()
{
    bool valid = false;
    float dist = get_target_distance(valid);
    if (!valid) {
        // 没有距离数据时不退出 AIM
        return false;
    }
    return dist <= plane.uattack.aim_dist.get();
}

bool ModeAttackLoc::check_away()
{
    bool valid = false;
    float dist = get_target_distance(valid);
    if (!valid) {
        return false;
    }
    return dist >= plane.uattack.away_dist.get();
}

bool ModeAttackLoc::check_target()
{
    bool valid = false;
    float dist = get_target_distance(valid);
    if (!valid) {
        // 没有距离时，相机目标重新出现则回到 AIM
        return plane.uattack.is_active_cam();
    }
    return dist > plane.uattack.recover_dist.get();
}

float ModeAttackLoc::get_target_distance(bool &valid)
{
    valid = false;
    float dist = 0.0f;

    // 优先使用外部给出的 target_dist
    if (plane.uattack.get_target_dist(dist, 2000)) {
        update_dist_source(1);
        valid = true;
        return dist;
    }

    // 其次用目标坐标计算
    if (plane.uattack._Target_ptr_loc != nullptr && plane.uattack._Target_ptr_loc->is_valid()) {
        update_dist_source(2);
        valid = true;
        return plane.current_loc.get_distance(plane.uattack._Target_ptr_loc->target_loc);
    }

    update_dist_source(0);
    return 0.0f;
}

void ModeAttackLoc::update_dist_source(uint8_t src)
{
    if (src == _dist_src) {
        return;
    }
    _dist_src = src;
    switch (src) {
        case 1:
            gcs().send_text(MAV_SEVERITY_INFO, "Dist src: target_dist");
            break;
        case 2:
            gcs().send_text(MAV_SEVERITY_INFO, "Dist src: coordinate");
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "Dist src: none");
            break;
    }
}

void ModeAttackLoc::start_recover()
{
    bool have_loc = (plane.uattack._Target_ptr_loc != nullptr && plane.uattack._Target_ptr_loc->is_valid());
    recover_center = have_loc ? plane.uattack._Target_ptr_loc->target_loc : plane.current_loc;

    int32_t alt_cm = 0;
    if (have_loc && recover_center.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm) && alt_cm != 0) {
        recover_alt_cm = alt_cm;
    } else {
        recover_alt_cm = plane.current_loc.alt;
    }

    plane.prev_WP_loc = plane.current_loc;
    plane.next_WP_loc = recover_center;
    plane.next_WP_loc.alt = recover_alt_cm;
    plane.set_target_altitude_location(plane.next_WP_loc);

    plane.auto_state.crosstrack = false;
    plane.loiter_angle_reset();
    plane.loiter.start_time_ms = 0;
    plane.auto_state.vtol_loiter = false;
}

void ModeAttackLoc::navigate()
{
    switch (stage) {
        case stage_class::APPROACH:
            plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
            break;
        case stage_class::RECOVER:
            plane.update_loiter(plane.uattack.recover_radius.get());
            break;
        default:
            break;
    }
}

void ModeAttackLoc::set_stage(ModeAttackLoc::stage_class stage_in)
{
    stage = stage_in;
    switch(stage) {
        case stage_class::APPROACH:
            gcs().send_text(MAV_SEVERITY_INFO, "In Approach");
            break;
        case stage_class::AIM:
            gcs().send_text(MAV_SEVERITY_INFO, "In Aim");
            _cmd_throttle = MAX(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle), plane.aparm.throttle_cruise);
            plane.uattack.attack_roll_pid.reset_I();
            plane.uattack.attack_roll_pid.reset_filter();
            plane.uattack.attack_roll_pid.set_integrator(0);
            break;
        case stage_class::AWAY:
            gcs().send_text(MAV_SEVERITY_INFO, "In Away");
            break;
        case stage_class::RECOVER:
            start_recover();
            // 盘旋需要 TECS 工作，清掉进入模式时的油门抑制
            plane.throttle_suppressed = false;
            gcs().send_text(MAV_SEVERITY_INFO, "In Recover");
            break;
        default:
            break;
    }
}

float ModeAttackLoc::get_cmd_throttle() {
    return _cmd_throttle;
}
