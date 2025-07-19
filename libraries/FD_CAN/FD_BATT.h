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
    void update_cmd();

    FD_CAN* _frotend_ptr;

    struct status_t {
        uint32_t id;
        float current;
        float voltage;
        float temperature;
        bool  brake;
        bool  brake_confirm;
        bool  zero;
        bool  zero_confirm;
        float pos;
        // union PACKED{
        //     int16_t v;
        //     uint8_t data[2];
        // } pos;
    };

    status_t last_ask_status_ms;
    uint8_t _data[8];
};
