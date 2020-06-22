#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "HB1_mission2apm_v1.h"
#include "HB1_mission2apm.h"
#include "HB1_mission2cam.h"
#include "HB1_cam2mission.h"
#include "HB1_power2apm.h"
#include "HB1_apm2mission.h"
#include "HB1_apm2cam.h"
#include "HB1_apm2power.h"

class HB1_UART {
public:

    HB1_UART(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
    {
        _port = NULL;
        _initialized = false;
    }

    /* Do not allow copies */
    HB1_UART(const HB1_UART &other) = delete;
    HB1_UART &operator=(const HB1_UART&) = delete;

    // init - perform required initialisation
    bool init();
    bool initialized() {return _initialized;}
    void read();
    void write();

    //HB1_mission2apm_v1& get_msg_mission2apm_v1() { return _msg_mission2apm_v1; }
    HB1_mission2apm& get_msg_mission2apm() { return _msg_mission2apm; }
    HB1_mission2cam& get_msg_mission2cam() { return _msg_mission2cam; }
    HB1_cam2mission& get_msg_cam2mission() { return _msg_cam2mission; }
    HB1_power2apm& get_msg_power2apm() { return _msg_power2apm; }
    HB1_apm2mission& get_msg_apm2mission() { return _msg_apm2mission; }
    HB1_apm2cam& get_msg_apm2cam() { return _msg_apm2cam; }
    HB1_apm2power& get_msg_apm2power() { return _msg_apm2power; }

private:

    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    //HB1_mission2apm_v1 _msg_mission2apm_v1;
    HB1_mission2apm _msg_mission2apm;
    HB1_mission2cam _msg_mission2cam;
    HB1_cam2mission _msg_cam2mission;
    HB1_power2apm _msg_power2apm;
    HB1_apm2mission _msg_apm2mission;
    HB1_apm2cam _msg_apm2cam;
    HB1_apm2power _msg_apm2power;
};
