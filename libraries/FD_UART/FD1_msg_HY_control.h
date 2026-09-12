#pragma once
/*
 * FD1_msg_HY_control — 慧眼(HY) 4.4.5 目标检测后自动锁定指令组包
 *
 * 指令帧: 58 07 03 05 04 DATA[4] CHK 59
 */
#include "FD1_message.h"

#define FD1_MSG_HY_CONTROL_LEN 11

class FD1_msg_HY_control : public FD1_message {
public:
    struct PACKED MSG_Collection {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t cmd0;
        uint8_t cmd1;
        uint8_t len;
        uint8_t payload[4];
        uint8_t sum;
        uint8_t end;
    };

    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_HY_CONTROL_LEN];
    };

    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD1_MSG_HY_CONTROL_LEN;
        Content_1 content;
    };

    FD1_msg_HY_control();

    FD1_msg_HY_control(const FD1_msg_HY_control &other) = delete;
    FD1_msg_HY_control &operator=(const FD1_msg_HY_control&) = delete;

    void process_message(void) override {}
    void parse(uint8_t temp) override { (void)temp; }
    void swap_message() override {}

    void pack_detect(uint8_t mode);
    void pack_auto_lock(uint8_t mode, uint8_t strategy);
    void make_sum();

    FD1UART_MSG_1 _msg_1;
};
