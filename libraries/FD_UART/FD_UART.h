#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD_payload2apm.h"

class FD_UART {
public:

    FD_UART(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
    }

    /* Do not allow copies */
    FD_UART(const FD_UART &other) = delete;
    FD_UART &operator=(const FD_UART&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void read();
    void write();

    FD_payload2apm& get_msg_payload2apm() { return _msg_payload2apm; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD_payload2apm _msg_payload2apm;
};
