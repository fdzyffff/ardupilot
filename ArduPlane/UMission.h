#pragma once

#include "mode.h"
#include <GCS_MAVLink/GCS_MAVLink.h>    // MAVLink GCS definitions

class UMission {
    UMission();
public:
    enum class Mission_Role{
        Leader = 0;
        Follower = 1;
    }

    void init();
    void update();
    void handle_info(const mavlink_command_long_t* packet);
    void do_offboard_cruise(float p2);
    void do_group();
    void do_set_leader(float p2);
    void update_leader();
    void update_group();
    void set_mode(Mode& new_mode);


    struct group_state_t {
        bool in_group;
        uint8_t type;
        Mode &mode;
    };
    group_state_t group_state;
    Mission_Role _role;
}