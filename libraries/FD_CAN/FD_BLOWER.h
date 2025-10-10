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
        // uint32_t id;
        // uint8_t mode_in; // 0: standby, 1: torque, 2: rpm
        // uint16_t rpm_in;
        // uint32_t last_mot_ms;
        // uint8_t send_count;

        // uint8_t mode_out;
        // uint16_t rpm_out;

        // float pitch_in;
        // float pitch_out;
        // uint32_t last_pitch_ms;
    };

    status_t status;
    uint8_t _data[8];
};
