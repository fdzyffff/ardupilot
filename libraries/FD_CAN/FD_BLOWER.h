#pragma once

#include <AP_HAL/AP_HAL.h>
#include "FD_CAN_2.h"

class FD_CAN_2;

class FD_BLOWER {
public:
    friend class FD_CAN_2;

    FD_BLOWER(FD_CAN_2* frotend);
    ~FD_BLOWER();

    /* Do not allow copies */
    FD_BLOWER(const FD_BLOWER &other) = delete;
    FD_BLOWER &operator=(const FD_BLOWER&) = delete;

    void handle_info(AP_HAL::CANFrame &in_frame, bool do_print = false);
    void do_power_on();
    void do_power_off();
    void do_on();
    void do_off();
    void update();
    void update_cmd();
    void send_cmd(uint32_t id, uint8_t *data);
    FD_CAN_2* _frotend_ptr;

    struct status_t {
        uint16_t p;
        uint16_t temperature;
        uint8_t error[8];
    };

    status_t status;
    uint8_t _data[8];
};
