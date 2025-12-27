#pragma once

#include <AP_HAL/AP_HAL.h>
#include "FD_CAN.h"

class FD_CAN;

class FD_MOT {
public:
    friend class FD_CAN;

    FD_MOT(FD_CAN* frotend);
    ~FD_MOT();

    /* Do not allow copies */
    FD_MOT(const FD_MOT &other) = delete;
    FD_MOT &operator=(const FD_MOT&) = delete;

    void handle_info(AP_HAL::CANFrame &in_frame, bool do_print = false);
    void set_id(uint8_t id_in);
    void set_thr(int16_t thr_in);
    void update();
    void update_cmd();
    void send_cmd(uint32_t id, uint8_t *data);
    FD_CAN* _frotend_ptr;

    struct status_t {
        uint32_t id;
        uint8_t order;
        uint8_t group;
        uint16_t thr_in;
        uint16_t rpm;
        uint16_t temp;
        uint32_t last_mot_ms;
        uint32_t last_rpm_ms;
        uint32_t last_temp_ms;
    };

    status_t status;
    uint8_t _data[8];
};
