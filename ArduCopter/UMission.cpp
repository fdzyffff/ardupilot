#include "Copter.h"

UMission::UMission()
{
    ;
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
            case MAV_CMD_USER_4:
                {
                    _target_loc_prob.lat = (int32_t)(packet.param5*1e7);
                    _target_loc_prob.lng = (int32_t)(packet.param6*1e7);
                    _target_loc_prob.alt = (int32_t)(packet.param7*1e2);
                    _last_loc_prob_ms = millis();
                }
                break;
            case MAV_CMD_USER_5:
                {
                    _target_loc.lat = (int32_t)(packet.param5*1e7);
                    _target_loc.lng = (int32_t)(packet.param6*1e7);
                    _target_loc.alt = (int32_t)(packet.param7*1e2);
                    _last_loc_ms = millis();
                }
                break;
            default:
                break;
        }
    }
}