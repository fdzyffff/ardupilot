#pragma once

#include <AP_HAL/AP_HAL.h>
#include "FD_CAN.h"

class FD_CAN;

class FD_BMS {
public:
    friend class FD_CAN;

    FD_BMS(FD_CAN* frotend);
    ~FD_BMS();

    /* Do not allow copies */
    FD_BMS(const FD_BMS &other) = delete;
    FD_BMS &operator=(const FD_BMS&) = delete;

    void handle_info(AP_HAL::CANFrame &in_frame, bool do_print = false);
    void set_id(uint8_t id_in);
    void set_thr(int16_t thr_in);
    void update();
    void update_cmd();
    void send_cmd(uint32_t id, uint8_t *data);
    void set_switch(uint8_t switch_in);
    FD_CAN* _frotend_ptr;

    struct status_t {
        uint8_t SOC;
        uint8_t SOH;
        float Volt;
        float Curr;
        uint8_t allow_charge;
        uint8_t allow_discharge;
        uint8_t error_charge_level;
        uint8_t error_charge_code;
        uint8_t error_discharge_level;
        uint8_t error_discharge_code;
        uint8_t batter_status;
        uint8_t other_error_code;
    };

    status_t status;
    uint8_t _data[8];
};
