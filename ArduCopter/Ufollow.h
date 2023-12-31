#pragma once

#include "Ugroup.h"
#include "RC_Channel.h"
#include <GCS_MAVLink/GCS_MAVLink.h>

class Ufollow {
public:
    Ufollow() {;}
    void init();
    void update();

    void auxSwitch_leader(const RC_Channel::AuxSwitchPos ch_flag);
    void auxSwitch_armtkoff(const RC_Channel::AuxSwitchPos ch_flag);
    void auxSwitch_distance(const RC_Channel::AuxSwitchPos ch_flag);
    void handle_msg(const mavlink_message_t &msg);
    void update_offset(int16_t leader_id, float group_distance);

    void set_cmd(uint8_t cmd_in);
    uint8_t get_cmd();

    void set_distance(float distance_in);
    float get_distance();

    void set_role(int8_t role_in);
    uint8_t get_role();
    uint32_t get_role_start_ms();

    void send_myfollow(mavlink_channel_t chan);

    uint8_t cmd_type;
    uint32_t cmd_ms;
    float distance;

    uint8_t group_type;
    my_group_1_t group_1;
    my_group_2_t group_2;

    uint8_t _role;
    uint32_t _role_start_ms;
};

