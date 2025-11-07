#pragma once

#include <AP_HAL/AP_HAL.h>
#include "FD_CAN_2.h"

class FD_CAN_2;

class FD_BMS {
public:
    friend class FD_CAN_2;

    FD_BMS(FD_CAN_2* frotend);
    ~FD_BMS();

    /* Do not allow copies */
    FD_BMS(const FD_BMS &other) = delete;
    FD_BMS &operator=(const FD_BMS&) = delete;

    void handle_info(AP_HAL::CANFrame &in_frame, bool do_print = false);
    void do_power_on();
    void do_power_off();
    void do_power_on(uint8_t id);
    void do_power_off(uint8_t id);
    void update();
    void update_cmd();
    void send_cmd(uint32_t id, uint8_t *data);
    FD_CAN_2* _frotend_ptr;

    struct status_t {
        uint16_t voltage;
        uint16_t soc;
        uint16_t temperature;
        uint8_t error1[8];
        uint8_t error2[8];
    };

    status_t status;
    uint8_t _data[8];
};
