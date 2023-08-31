#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD1_msg_DYTTELEM.h"
#include "FD1_msg_DYTTARGET.h"
#include "FD1_msg_APM2DYTCONTROL.h"
#include "FD1_msg_APM2DYTTELEM.h"

class FD1_UART {
public:

    FD1_UART(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
        init();
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
    void read();
    void read(uint8_t temp);
    void write();

    uint32_t port_avaliable();

    FD1_msg_DYTTELEM& get_msg_DYTTELEM()   { return _msg_DYTTELEM; }
    FD1_msg_DYTTARGET& get_msg_DYTTARGET()   { return _msg_DYTTARGET; }
    FD1_msg_APM2DYTCONTROL& get_msg_APM2DYTCONTROL()   { return _msg_APM2DYTCONTROL; }
    FD1_msg_APM2DYTTELEM& get_msg_APM2DYTTELEM()   { return _msg_APM2DYTTELEM; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD1_msg_DYTTELEM _msg_DYTTELEM;
    FD1_msg_DYTTARGET _msg_DYTTARGET;
    FD1_msg_APM2DYTCONTROL _msg_APM2DYTCONTROL;
    FD1_msg_APM2DYTTELEM _msg_APM2DYTTELEM;
};
