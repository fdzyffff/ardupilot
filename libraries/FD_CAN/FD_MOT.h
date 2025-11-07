#pragma once

#include <AP_HAL/AP_HAL.h>
#include "FD_CAN_1.h"

class FD_CAN_1;

class FD_MOT {
public:
    friend class FD_CAN_1;

    FD_MOT(FD_CAN_1* frotend);
    ~FD_MOT();

    /* Do not allow copies */
    FD_MOT(const FD_MOT &other) = delete;
    FD_MOT &operator=(const FD_MOT&) = delete;

    void handle_info(AP_HAL::CANFrame &in_frame, bool do_print = false);
    void set_id(uint8_t id_in);
    void set_rev(bool rev_in);
    void set_mode(uint8_t mode_in);
    void set_rpm(int16_t rpm_in);
    void set_pitch(float pitch_in);
    void update();
    void update_cmd();
    void sumcheck();
    void send_cmd(uint32_t id, uint8_t *data);
    FD_CAN_1* _frotend_ptr;

    struct status_t {
        uint32_t id;
        uint8_t mode_in; // 0: standby, 1: torque, 2: rpm
        int16_t rpm_in;
        uint32_t last_mot_ms;
        uint8_t send_count;

        uint8_t mode_out;
        int16_t rpm_out;

        float pitch_in;
        float pitch_out;
        uint32_t last_pitch_ms;

        uint16_t mot_temperature;
        uint16_t controller_temperature;
        uint16_t mot_error;
        uint16_t propeller_error;
    };

    status_t status;
    uint8_t _data[8];
};
