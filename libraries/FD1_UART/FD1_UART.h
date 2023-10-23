#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD1_msg_init.h"
#include "FD1_msg_control.h"
#include "FD1_msg_mission.h"
#include "FD1_msg_guide.h"
#include "FD1_msg_info.h"

class FD1_UART {
public:

    FD1_UART(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
    }

    /* Do not allow copies */
    FD1_UART(const FD1_UART &other) = delete;
    FD1_UART &operator=(const FD1_UART&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void read();
    void write();

    uint32_t port_avaliable();

    FD1_msg_init& get_msg_init()   { return _msg_init; }
    FD1_msg_control& get_msg_control()   { return _msg_control; }
    FD1_msg_mission& get_msg_mission()   { return _msg_mission; }
    FD1_msg_guide& get_msg_guide()   { return _msg_guide; }
    FD1_msg_info& get_msg_info()   { return _msg_info; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD1_msg_init _msg_init;
    FD1_msg_control _msg_control;
    FD1_msg_mission _msg_mission;
    FD1_msg_guide _msg_guide;
    FD1_msg_info _msg_info;
};
