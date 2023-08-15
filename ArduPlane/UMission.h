#pragma once

class UMission {
    UMission();
public:
    enum class Mission{
        Leader = 0;
        Follower = 1;
    }
    void init();
    void update();
    void handle_info(const mavlink_command_long_t* packet);
    void do_offboard();
    void do_group();
}