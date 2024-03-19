#pragma once

#include "mode.h"
#include <GCS_MAVLink/GCS_MAVLink.h>    // MAVLink GCS definitions

class UMission {
public:
    UMission();
    enum class Mission_Role{
        Leader = 0,
        Follower = 1,
    };

    void init();
    void update();
    void handle_msg(const mavlink_message_t &msg);
    void do_offboard_control(float p2);
    void do_group_init();
    void keep_group();
    void do_set_leader(float p2);
    void update_leader();
    void update_group();
    void set_mode(Mode& new_mode);
    uint8_t get_leader_id() {return _leader_id;}
    void set_target_loc(Location& loc_in);
    void update_base();

    struct group_state_t {
        bool in_group;
        uint8_t type;
        Mode *mode;
    };

    uint8_t _leader_id;
    group_state_t group_state;
    Mission_Role _role;

    LowPassFilterVector3f _base_target_pos{1.0};
    Location _base_target_loc_raw;
    Location _base_target_loc;
    uint32_t _base_target_ms;
    bool _base_target_valid;
    bool in_base_target_guided_mode;
};
