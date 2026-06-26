#pragma once

#include <AP_HAL/AP_HAL.h>
#include "FD_CAN.h"

class FD_CAN;

class FD_BATT {
public:
    friend class FD_CAN;

    FD_BATT(FD_CAN* frotend);
    ~FD_BATT();

    /* Do not allow copies */
    FD_BATT(const FD_BATT &other) = delete;
    FD_BATT &operator=(const FD_BATT&) = delete;

    void handle_info(AP_HAL::CANFrame &in_frame, bool do_print = false);
    void send_cmd(uint32_t id, uint8_t *data);

    FD_CAN* _frotend_ptr;

    struct status_t {
        float vfc;
        float vout;
        float I;
        float T1;
        float T2;
        float P;
        uint8_t PWM1;
        uint8_t PWM2;
        float vli;
        float vhy;
        float vbus;
        float power;
        uint8_t HPWM1;
        uint8_t HPWM2;
        uint8_t error;
        uint8_t run;
    };

    status_t status;
};
