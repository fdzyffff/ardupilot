#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD_msg_HaoFu.h"

class FD_HaoFu {
public:

    FD_HaoFu(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
        init();
    }

    FD_HaoFu(AP_HAL::UARTDriver *port_in)
    {
        _port = port_in;
        _initialized = true;
    }

    /* Do not allow copies */
    FD_HaoFu(const FD_HaoFu &other) = delete;
    FD_HaoFu &operator=(const FD_HaoFu&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void read();
    void read(uint8_t temp);
    void write();
    void write(uint8_t temp);

    uint32_t port_avaliable();

    FD_msg_HaoFu& get_msg_HaoFu()   { return _msg_HaoFu; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD_msg_HaoFu _msg_HaoFu;
};
