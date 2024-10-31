#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD_CAM_CMD.h"
#include "FD_CAM_TARGET.h"
#include "FD_CAM_STATUS.h"

class FD_CAM {
public:

    FD_CAM(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
        init();
    }

    FD_CAM(AP_HAL::UARTDriver *port_in)
    {
        _port = port_in;
        _initialized = true;
    }

    /* Do not allow copies */
    FD_CAM(const FD_CAM &other) = delete;
    FD_CAM &operator=(const FD_CAM&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void read();
    void read(uint8_t temp);
    void write();
    void write(uint8_t temp);

    uint32_t port_avaliable();

    FD_CAM_CMD& get_msg_cam_cmd()          { return _msg_cam_cmd; }
    FD_CAM_TARGET& get_msg_cam_target()    { return _msg_cam_target; }
    FD_CAM_STATUS& get_msg_cam_status()    { return _msg_cam_status; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD_CAM_CMD _msg_cam_cmd;
    FD_CAM_TARGET _msg_cam_target;
    FD_CAM_STATUS _msg_cam_status;
};
