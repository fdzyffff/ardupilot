#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "FD_msg_YOLO.h"

class FD_YOLO {
public:
    FD_YOLO(enum AP_SerialManager::SerialProtocol protocol)
        : _protocol(protocol), _port(nullptr), _initialized(false)
    {
        init();
    }

    FD_YOLO(const FD_YOLO &other) = delete;
    FD_YOLO &operator=(const FD_YOLO&) = delete;

    bool init();
    bool initialized() const { return _initialized; }
    void read();

    FD_msg_YOLO& get_msg() { return _msg; }

private:
    AP_HAL::UARTDriver *_port;
    AP_SerialManager::SerialProtocol _protocol;
    bool _initialized;
    FD_msg_YOLO _msg;
};
