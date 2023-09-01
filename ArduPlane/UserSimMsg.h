#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

class UserSimMsg {
public:
    UserSimMsg(enum AP_SerialManager::SerialProtocol protocol);
    void init();
    void handle_msg(const mavlink_message_t &msg);
    void update();
    void read_uart();
    void send_mav();
    void send_msg(mavlink_message_t *msg);

    struct {
        // socket to telem2 on aircraft
        bool connected;
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink;

    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;
};
