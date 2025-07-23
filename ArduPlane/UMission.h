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
    void get_Time(uint8_t &year_out, uint8_t &month_out, uint8_t &day_out, uint8_t &hour_out, uint8_t &minute_out, uint8_t &second_out);
    void handle_msg(const mavlink_message_t &msg); // for test purpose

    bool valid() {return _valid;}
    uint8_t get_control_type() {return _control_type;}
    float get_control_altitude() {return _control_altitude;}
    float get_control_speed() {return _control_speed;}
    float get_control_roll() {return _control_roll;}
    float get_control_course() {return _control_course;}

private:
    FD1_UART _uart_control{AP_SerialManager::SerialProtocol_SwarmControl};
    FD1_UART _uart_link{AP_SerialManager::SerialProtocol_SwarmLink};

    bool _valid;
    uint32_t _last_ms;
    uint32_t _last_log_ms;

    uint8_t _control_type;
    float _control_altitude;
    float _control_speed;
    float _control_roll;
    float _control_course;
};
