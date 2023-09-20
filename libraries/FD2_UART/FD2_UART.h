#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "FD2_msg_ue4_ahrs.h"
#include "FD2_msg2apm_ue4_gimbal.h"

class FD2_UART {
public:

    FD2_UART(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
    }

    /* Do not allow copies */
    FD2_UART(const FD2_UART &other) = delete;
    FD2_UART &operator=(const FD2_UART&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void read();
    void write();

    uint32_t port_avaliable();

    FD2_msg_ue4_ahrs& get_msg_ue4_ahrs()   { return _msg_ue4_ahrs; }
    FD2_msg2apm_ue4_gimbal& get_msg2apm_ue4_gimbal()   { return _msg2apm_ue4_gimbal; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to handle and send data
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    FD2_msg_ue4_ahrs _msg_ue4_ahrs;
    FD2_msg2apm_ue4_gimbal _msg2apm_ue4_gimbal;
};
