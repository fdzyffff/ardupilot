#include "Copter.h"

UMission::UMission()
{
    ;
}

void UMission::update()
{
    send_mav();
}

bool UMission::target_pos_prob_valid()
{
    if (millis() - _last_loc_prob_ms < 50000) {
        return true;
    } 
    return false;
}

bool UMission::target_pos_valid()
{
    if (millis() - _last_loc_ms < 50000) {
        return true;
    } 
    return false;
}

Location UMission::get_target_pos()
{
    return _target_loc;
}

Location UMission::get_target_pos_prob()
{
    return _target_loc_prob;
}

// USER_4: 目标概略位置
// USER_5: 目标解析位置
void UMission::handle_mission_msg(const mavlink_message_t &msg) {
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        // decode packet
        // gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(&msg, &packet);
        switch(packet.command) {
            case MAV_CMD_USER_5:
                {
                    if (int16_t(packet.param1) == 1) {
                        _target_loc_prob.lat = (int32_t)(packet.param5*1e7);
                        _target_loc_prob.lng = (int32_t)(packet.param6*1e7);
                        _target_loc_prob.alt = (int32_t)(packet.param7*1e2);
                        _last_loc_prob_ms = millis();
                        if (int16_t(packet.param2) == 1) {
                           copter.mode_mission.set_cruise_state();
                        }
                    }
                    if (int16_t(packet.param1) == 2) {
                        _target_loc.lat = (int32_t)(packet.param5*1e7);
                        _target_loc.lng = (int32_t)(packet.param6*1e7);
                        _target_loc.alt = (int32_t)(packet.param7*1e2);
                        _last_loc_ms = millis();
                    }
                }
                break;
            default:
                break;
        }
    }
}

void UMission::send_mav()
{
    static uint32_t _last_send_ms = 0;
    uint32_t tnow = millis();
    // uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    float param1 = (uint8_t)(copter.mode_mission.get_state());
    if (copter.flightmode->mode_number() != Mode::Number::MISSION) {
        param1 = -1.0f;
    }
    float param2 = copter.ugimbal.have_target();
    float param3 = copter.ugimbal._cam_pitch;
    float param4 = copter.ugimbal._cam_yaw;
    float param5 = 0.0f;
    float param6 = 0.0f;
    float param7 = 0.0f;
    if (tnow - _last_send_ms > 1000) {
        _last_send_ms = tnow;
        for (uint8_t i=0; i<gcs().num_gcs(); i++) {
            mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
            // if (mask & (1U<<i)) {
                if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 50) {
                    mavlink_msg_command_long_send(
                        channel,
                        0,
                        0,
                        MAV_CMD_USER_1,
                        0,
                        param1,
                        param2,
                        param3,
                        param4,
                        param5,
                        param6,
                        param7);
                    }
            // }
        }
    }
}
