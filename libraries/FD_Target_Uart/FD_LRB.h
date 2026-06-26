#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD_LRB_CMD.h"
#include "FD_LRB_TARGET.h"
#include "FD_LRB_STATUS.h"

class FD_LRB {
public:

    FD_LRB(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
        init();
    }

    FD_LRB(AP_HAL::UARTDriver *port_in)
    {
        _port = port_in;
        _initialized = true;
    }

    /* Do not allow copies */
    FD_LRB(const FD_LRB &other) = delete;
    FD_LRB &operator=(const FD_LRB&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void read();
    void read(uint8_t temp);
    void write();
    void write(uint8_t temp);

    uint32_t port_avaliable();

    FD_LRB_CMD& get_msg_cam_cmd()          { return _msg_cam_cmd; }
    FD_LRB_TARGET& get_msg_cam_target()    { return _msg_cam_target; }
    FD_LRB_STATUS& get_msg_cam_status()    { return _msg_cam_status; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD_LRB_CMD _msg_cam_cmd;
    FD_LRB_TARGET _msg_cam_target;
    FD_LRB_STATUS _msg_cam_status;
};
