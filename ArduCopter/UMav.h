#pragma once

#include "UMav_trans.h"
#include "FD_UART/FD_UART.h"

class UMav {

public:

    // constructor, destructor
    UMav();

    void init();
    void update();

    void send_raw_imu();
    void read_bsq_message();
    void handle_bsq_msg(const mavlink_message_t &msg);
    void send_bsq_message(const mavlink_message_t &msg, uint16_t len);
    void send_apm_status();

    void handle_mission_msg(const mavlink_message_t &msg);
    // void handle_selfcheck(const mavlink_message_t &msg);
    // void handle_target(const mavlink_message_t &msg);
    // void handle_target_result(const mavlink_message_t &msg);
    // void handle_selfcheck_result(const mavlink_message_t &msg);
    // void handle_status(const mavlink_message_t &msg);
    // void handle_mission(const mavlink_message_t &msg);
    // void handle_mission_result(const mavlink_message_t &msg);
    // void handle_relay_position(const mavlink_message_t &msg);
    // void handle_relay_position_result(const mavlink_message_t &msg);
    // void handle_attack_info(const mavlink_message_t &msg);
    // void handle_attack_cmd(const mavlink_message_t &msg);
    // void handle_nav_cmd(const mavlink_message_t &msg);
    // void send_do_selfcheck();
    // void send_target();
    // void send_status();
    // void send_selfcheck_result();
    // void send_target_result();
    // void send_mission();
    // void send_mission_result();
    // void send_relay_position();
    // void send_relay_position_result();

    void handle_info_test(int16_t p1);
    void send_all();

    struct {
        // socket to telem2 on aircraft
        bool connected;
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink;

    FD_UART FD_uart_imu{AP_SerialManager::SerialProtocol_IMU};
    FD_UART FD_uart_bsq{AP_SerialManager::SerialProtocol_BSQ};

    UMav_trans_status          trans_status;
    UMav_trans_selfcheck       trans_selfcheck;
    UMav_trans_target          trans_target;
    UMav_trans_mission         trans_mission;
    UMav_trans_relay_positon   trans_relay_positon;

};
