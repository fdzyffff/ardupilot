#pragma once

#include <AP_HAL/AP_HAL.h>
#include "FD_CAN_1.h"

class FD_CAN_1;

class FD_SERVO {
public:
    friend class FD_CAN_1;

    FD_SERVO(FD_CAN_1* frotend);
    ~FD_SERVO();

    /* Do not allow copies */
    FD_SERVO(const FD_SERVO &other) = delete;
    FD_SERVO &operator=(const FD_SERVO&) = delete;

    void handle_info(AP_HAL::CANFrame &in_frame, bool do_print = false);
    void set_pos(float pos_in);
    void set_brake(bool brake_in);
    void enable_brake(bool enable);
    void set_id(uint8_t id_in);
    bool get_brake();
    void update();
    void update_cmd_nobrake();
    void update_cmd_brake();
    void send_cmd(uint32_t id, uint8_t *data);
    FD_CAN_1* _frotend_ptr;

    struct status_t {
        uint32_t id;
        float current;
        float voltage;
        float temperature;
        bool  brake;    //舵机当前制动状态
        bool  target_brake; //舵机目标制动状态
        bool  brake_confirm;    //制动状态确认标志（true = 目标与当前状态一致，false = 待确认）
        bool  zero;
        bool  zero_confirm;
        float pos;
        float last_pos;
        bool  have_brake;   //是否启用制动功能（true = 有制动功能，如襟翼舵机；false = 无制动功能）
        uint16_t AngleFb;
        uint16_t AngleCtrl;
        uint8_t Current;
        uint8_t Voltage;
        uint16_t SelfCheckState;
        uint32_t last_brake_ms;
        uint32_t last_pos_ms;
        uint32_t last_send_pos_ms;
        uint32_t last_ask_status_ms;
    };

    status_t status;
    uint8_t _data[8];
};
