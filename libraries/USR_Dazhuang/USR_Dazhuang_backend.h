#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

#include "FD1_msg_in.h"
#include "FD1_msg_out.h"

class USR_Dazhuang_backend {
public:

    USR_Dazhuang_backend(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
        _start_time_ms = 0;
    }

    /* Allow copies */

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void update(void);
    void read(void);
    void start(void);
    void close(void);
    void setup(float value);
    void write(void);
    AP_SerialManager::SerialProtocol get_protocol() {return _protocol;}

    FD1_msg_in& get_msg_in() { return _msg_in; }
    FD1_msg_out& get_msg_out() { return _msg_out; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;
    uint32_t _start_time_ms;

    FD1_msg_in _msg_in;
    FD1_msg_out _msg_out;
};
