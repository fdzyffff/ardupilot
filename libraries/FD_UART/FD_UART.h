#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD_engine.h"

#define MAX_ENGINE_PORT_NUM 7

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
    AP_HAL::UARTDriver* get_port() {return _port;}

    FD_engine& get_msg_engine();
    FD_engine& get_msg_engine(uint8_t instance);

private:

    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD_engine _msg_engine[MAX_ENGINE_PORT_NUM];
};
