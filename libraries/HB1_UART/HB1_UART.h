#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "HB1_ins2apm.h"

class HB1_UART {
public:

    HB1_UART(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
    }

    /* Do not allow copies */
    HB1_UART(const HB1_UART &other) = delete;
    HB1_UART &operator=(const HB1_UART&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void read();
    void write();

    HB1_ins2apm& get_msg_ins2apm() { return _msg_ins2apm; }
    // HB1_apm2payload& get_msg_apm2payload() { return _msg_apm2payload; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    HB1_ins2apm _msg_ins2apm;
    // HB1_apm2payload _msg_apm2payload;
};
