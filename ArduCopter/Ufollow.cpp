#include "Copter.h"

void Ufollow::init()
{
    cmd_type = 0;
    cmd_ms = 0;
    set_distance(copter.g2.user_parameters.distance.get());
    group_type = 1;
    _role = 0;
    _role_start_ms = 0;
}

void Ufollow::update()
{
    if (cmd_type != 0) {
        if (millis() - cmd_ms > 3000) {
            cmd_type = 0;
            cmd_ms = 0;
        }
    }
}

void Ufollow::set_cmd(uint8_t cmd_in)
{
    cmd_type = cmd_in;
    cmd_ms = millis();
}

uint8_t Ufollow::get_cmd()
{
    return cmd_type;
}

void Ufollow::set_distance(float distance_in)
{
    distance = constrain_float(distance_in, 3.0f, 50.0f);
}

float Ufollow::get_distance() 
{
    return distance;
}

void Ufollow::set_role(int8_t role_in) {
    if (_role != role_in) {
        gcs().send_text(MAV_SEVERITY_INFO, "Set Role %d", role_in);
    }
    _role = role_in;
    if (_role == 1) {
        _role_start_ms = millis();
    }
}

uint8_t Ufollow::get_role() {
    return _role;
}

uint32_t Ufollow::get_role_start_ms() {
    if (_role == 1) {
        return (millis() - _role_start_ms);
    } else {
        return 0;
    }
}

void Ufollow::auxSwitch_leader(const RC_Channel::AuxSwitchPos ch_flag)
{
    // myfollow aux switch handler (CHx_OPT = 220)
    // if (copter.flightmode != &copter.mode_myfollow) {
    //     return;
    // }
    switch (ch_flag) {
    case RC_Channel::AuxSwitchPos::HIGH:
        if (!copter.set_mode(Mode::Number::MYFOLLOW, ModeReason::GCS_COMMAND)) {
            return;
        }
        if (get_role() == 0) {
            set_role(1);
            if (!copter.mode_myfollow.is_disarmed_or_landed()) {
                copter.mode_myfollow.loiter_start();
            }
        }
        break;
    case RC_Channel::AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case RC_Channel::AuxSwitchPos::LOW:
        // nothing
        break;
    }
}

void Ufollow::auxSwitch_armtkoff(const RC_Channel::AuxSwitchPos ch_flag)
{
    // myfollow aux switch handler (CHx_OPT = 221)
    // if (copter.flightmode != &copter.mode_myfollow) {
    //     return;
    // }
    switch (ch_flag) {
    case RC_Channel::AuxSwitchPos::HIGH:
        if (!copter.set_mode(Mode::Number::MYFOLLOW, ModeReason::GCS_COMMAND)) {
            return;
        }
        if (get_role() == 1) {
            cmd_type = 1; // packet.armtkoff_command = 1
            cmd_ms = millis();
            set_cmd(1);
        }
        if (copter.mode_myfollow.is_disarmed_or_landed() && copter.arming.arm(AP_Arming::Method::MAVLINK)) {
            if (copter.mode_myfollow.myfollow_mode == ModeMYFOLLOW::SubMode::THROW) {
                ;
            } else {
                copter.mode_myfollow.do_user_takeoff_start(300.0f);
            }
        }
        break;
    case RC_Channel::AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case RC_Channel::AuxSwitchPos::LOW:
        // nothing
        break;
    }
}

void Ufollow::auxSwitch_distance(const RC_Channel::AuxSwitchPos ch_flag)
{
    // myfollow aux switch handler (CHx_OPT = 222)
    if (copter.flightmode != &copter.mode_myfollow) {
        return;
    }
    switch (ch_flag) {
    case RC_Channel::AuxSwitchPos::HIGH:
        set_distance(get_distance() + 1.0f);
        break;
    case RC_Channel::AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case RC_Channel::AuxSwitchPos::LOW:
        set_distance(get_distance() - 1.0f);
        break;
    }
}

void Ufollow::auxSwitch_throw(const RC_Channel::AuxSwitchPos ch_flag)
{
    // myfollow aux switch handler (CHx_OPT = 223)
    switch (ch_flag) {
    case RC_Channel::AuxSwitchPos::HIGH:
        if (!copter.set_mode(Mode::Number::MYFOLLOW, ModeReason::GCS_COMMAND)) {
            return;
        }
        if (copter.mode_myfollow.is_disarmed_or_landed()) {
            copter.mode_myfollow.throw_start();
        }
        break;
    case RC_Channel::AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case RC_Channel::AuxSwitchPos::LOW:
        // nothing
        break;
    }
}

void Ufollow::handle_msg(const mavlink_message_t &msg) {
    // if (copter.flightmode != &copter.mode_myfollow) {
    //     return;
    // }
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_MY_FOLLOW: {
            // gcs().send_text(MAV_SEVERITY_INFO, "Receive");
            // decode message
            mavlink_my_follow_t packet;
            mavlink_msg_my_follow_decode(&msg, &packet);
            // role==0 will return 0, so it will never enter this condition
            if (packet.start_ms < get_role_start_ms()) {
                //switch to follower because new leader appear
                set_role(0);
                copter.mode_myfollow.standby_start();
            }
            if (get_role() == 0) {
                update_offset(msg.sysid, packet.distance);
                if (packet.armtkoff_command) {                    
                    if (!copter.set_mode(Mode::Number::MYFOLLOW, ModeReason::GCS_COMMAND)) {
                        return;
                    }
                    if (get_cmd() == 0) {
                        auxSwitch_armtkoff(RC_Channel::AuxSwitchPos::HIGH);
                        set_cmd(1);
                    }
                }
            }
            break;
        default:
            break;
        }
    }
}

void Ufollow::send_myfollow(mavlink_channel_t chan)
{
    if (copter.flightmode != &copter.mode_myfollow) {
        return;
    }
    if (get_role() == 1 && (!copter.mode_myfollow.is_disarmed_or_landed())) {
        mavlink_msg_my_follow_send(chan, get_role_start_ms(), get_distance(), get_cmd());
    }
}

void Ufollow::update_offset(int16_t leader_id, float group_distance) {
    copter.g2.follow.set_target_sysid(leader_id);
    set_distance(group_distance);
    Vector3f group_offset = group_1.get_offset(copter.g.sysid_this_mav, leader_id, distance);
    if (get_role() == 0) {
        group_offset.z += -2.0f;
    }
    copter.g2.follow.set_target_offset(group_offset);
}
