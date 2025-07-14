#pragma once

#include <AP_HAL/AP_HAL.h>
#include <FD_Gimbal/FD_Gimbal.h>

class UMission {

public:

    friend class Copter;
    friend class ModeMission;

    // constructor, destructor
    UMission();

    void update();
    void send_mav();

    bool target_pos_prob_valid();
    bool target_pos_valid();

    Location get_target_pos_prob();
    Location get_target_pos();

    uint32_t _last_loc_prob_ms;
    uint32_t _last_loc_ms;

    Location _target_loc_prob;
    Location _target_loc;

    void handle_mission_msg(const mavlink_message_t &msg);
};
