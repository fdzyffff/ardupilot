#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD1_msg_K230.h"
#include "FD1_msg_RK3588.h"
#include "FD1_msg_SIYIA8mini.h"

class FD1_UART {
public:

    FD1_UART(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
    }

    FD1_UART(AP_HAL::UARTDriver *port_in)
    {
        _port = port_in;
        _initialized = true;
    }

    /* Do not allow copies */
    FD1_UART(const FD1_UART &other) = delete;
    FD1_UART &operator=(const FD1_UART&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}

    AP_HAL::UARTDriver* get_port() {return _port;}

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;
};
