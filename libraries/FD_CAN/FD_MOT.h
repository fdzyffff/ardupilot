#pragma once

#define FD_CAN_MAX_MOT_NUM 8

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
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
    void init();
    void set_pwm(uint8_t id_in, uint16_t pwm_in);
    bool get_throttle_address(uint8_t i_mot, uint32_t &address);
    void update();
    void update_cmd();
    void update_status();
    void update_log();
    void send_cmd(uint32_t id, uint8_t *data);
    FD_CAN* _frotend_ptr;

    struct status_t {
        uint16_t thr_in;
        bool have_thr;
        uint16_t rpm1;
        uint16_t rpm2;
        uint16_t temp1;
        uint16_t temp2;
        uint32_t last_status_ms;
        uint32_t last_mot_ms;
        uint32_t last_rpm_ms;
        uint32_t last_temp_ms;
        uint32_t last_print_ms;
    };

    status_t status[FD_CAN_MAX_MOT_NUM];
    uint8_t _data[8];

    uint32_t _last_log_ms;
    bool _allow_log;
};
