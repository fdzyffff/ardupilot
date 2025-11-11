#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD_msg_RK3588.h"

class FD_RK3588 {
public:

    FD_RK3588(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
        init();
    }

    FD_RK3588(AP_HAL::UARTDriver *port_in)
    {
        _port = port_in;
        _initialized = true;
    }

    /* Do not allow copies */
    FD_RK3588(const FD_RK3588 &other) = delete;
    FD_RK3588 &operator=(const FD_RK3588&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void read();
    void read(uint8_t temp);
    void write();
    void write(uint8_t temp);

    uint32_t port_avaliable();

    FD_msg_RK3588& get_msg_RK3588()   { return _msg_RK3588; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD_msg_RK3588 _msg_RK3588;
};
