#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD_msg_K230.h"

class FD_K230 {
public:

    FD_K230(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
        init();
    }

    FD_K230(AP_HAL::UARTDriver *port_in)
    {
        _port = port_in;
        _initialized = true;
    }

    /* Do not allow copies */
    FD_K230(const FD_K230 &other) = delete;
    FD_K230 &operator=(const FD_K230&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void read();
    void read(uint8_t temp);
    void write();
    void write(uint8_t temp);

    uint32_t port_avaliable();

    FD_msg_K230& get_msg_K230()   { return _msg_K230; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD_msg_K230 _msg_K230;
};
