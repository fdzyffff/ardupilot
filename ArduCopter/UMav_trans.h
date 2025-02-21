#pragma once

#include "GCS_MAVLink.h"

class UMav_trans_status {
public:
    UMav_trans_status() {};
    void update();
    void send_mission_msg();
    void handle_mission_msg(const mavlink_message_t &msg);
    void send_bsq_msg();

    mavlink_wxbs_status_t packet;

    uint32_t repeat_time_ms = 1000;
    uint32_t last_send_bsq_ms = 0;
};

class UMav_trans_selfcheck {
public:
    UMav_trans_selfcheck() {};
    void update();
    void handle_bsq_msg(const mavlink_message_t &msg);
    void send_mission_msg();
    void handle_mission_msg(const mavlink_message_t &msg);
    void send_bsq_msg();

    mavlink_wxbs_do_selfcheck_t in_packet;
    mavlink_wxbs_selfcheck_result_t out_packet;

    uint32_t receive_bsq_ms;
    uint32_t timeout_bsq_ms = 60000;
    uint32_t repeat_time_ms = 20000;
    uint32_t last_send_mission_ms;
    bool bsq_waiting;
};

class UMav_trans_target {
public:
    UMav_trans_target() {};
    void update();
    void handle_bsq_msg(const mavlink_message_t &msg);
    void send_mission_msg();
    void handle_mission_msg(const mavlink_message_t &msg);
    void send_bsq_msg();

    mavlink_wxbs_target_t in_packet;
    mavlink_wxbs_target_result_t out_packet;

    uint32_t receive_bsq_ms;
    uint32_t timeout_bsq_ms = 60000;
    uint32_t repeat_time_ms = 20000;
    uint32_t last_send_mission_ms;
    bool bsq_waiting;
};

class UMav_trans_mission {
public:
    UMav_trans_mission() {};
    void update();
    void handle_bsq_msg(const mavlink_message_t &msg);
    void send_mission_msg();
    void handle_mission_msg(const mavlink_message_t &msg);
    void send_bsq_msg();

    mavlink_wxbs_mission_t in_packet;
    mavlink_wxbs_mission_result_t out_packet;

    uint32_t receive_bsq_ms;
    uint32_t timeout_bsq_ms = 60000;
    uint32_t repeat_time_ms = 20000;
    uint32_t last_send_mission_ms;
    bool bsq_waiting;
};

class UMav_trans_relay_positon {
public:
    UMav_trans_relay_positon() {};
    void update();
    void handle_bsq_msg(const mavlink_message_t &msg);
    void send_mission_msg();
    void handle_mission_msg(const mavlink_message_t &msg);
    void send_bsq_msg();

    mavlink_wxbs_relay_position_t in_packet;
    mavlink_wxbs_relay_position_result_t out_packet;

    uint32_t receive_bsq_ms;
    uint32_t timeout_bsq_ms = 60000;
    uint32_t repeat_time_ms = 20000;
    uint32_t last_send_mission_ms;
    bool bsq_waiting;
};
