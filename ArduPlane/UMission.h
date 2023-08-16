#pragma once

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
    void do_offboard_cruise();
    void do_group();
    void set_leader_id();

    Mission_Role _role;
}