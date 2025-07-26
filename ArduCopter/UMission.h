#pragma once

#include <AP_HAL/AP_HAL.h>
#include <FD1_UART/FD1_UART.h>

class UMission {

public:

    friend class Plane;

    // constructor, destructor
    UMission();
    void init();
    void update();
    void update_log();
    void update_uart_read();
    void update_uart_send();
    void update_valid();
    void handle_msg_control();
    void handle_msg_ground();
    void send_status();
    void send_trans();
    void handle_msg(const mavlink_message_t &msg); // for test purpose
    void update_mav();

    bool valid() {return _valid;}

    float get_control_corr_bfy() {return _control_corr_bfy;}
    float get_control_corr_bfz() {return _control_corr_bfz;}

    void send_raw_imu_loop();
    void send_raw_imu();

private:
    FD1_UART _uart_control{AP_SerialManager::SerialProtocol_SwarmControl};

    bool _valid;
    uint32_t _last_ms;
    uint32_t _last_log_ms;
    uint32_t _last_mav_ms;

    float _control_corr_bfy;
    float _control_corr_bfz;
};
