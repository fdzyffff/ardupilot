#pragma once
/*
 * FD1_msg_HY_ack — 慧眼(HY) 算法功能设置(4.4.x)响应帧解析
 *
 * 4.4.1 目标检测控制响应: 78 07 03 81 02 [结果][检测状态] CHK 79
 * 4.4.5 自动锁定响应:     78 07 03 85 04 [结果][锁定态][策略][保留] CHK 79
 * 小端字节序；CHK为CMD0至最后一个数据字节的累加和低字节。
 */
#include "FD1_message.h"

#define FD1_MSG_HY_ACK_MAX_LEN 11

class FD1_msg_HY_ack : public FD1_message {
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
        uint8_t data[FD1_MSG_HY_ACK_MAX_LEN];
    };

    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD1_MSG_HY_ACK_MAX_LEN;
        Content_1 content;
    };

    struct PACKED FD1UART_msg_parser {
        enum {
            FD1UART_PREAMBLE1 = 0,
            FD1UART_PREAMBLE2,
            FD1UART_CMD0,
            FD1UART_CMD1,
            FD1UART_LEN,
            FD1UART_DATA,
            FD1UART_SUM,
            FD1UART_END,
        } msg_state;

        uint16_t read;
        uint16_t length;
        uint8_t sum;
        uint8_t data[FD1_MSG_HY_ACK_MAX_LEN];
    } _msg;

    FD1_msg_HY_ack();

    FD1_msg_HY_ack(const FD1_msg_HY_ack &other) = delete;
    FD1_msg_HY_ack &operator=(const FD1_msg_HY_ack&) = delete;

    const uint8_t PREAMBLE1 = 0x78;
    const uint8_t PREAMBLE2 = 0x07;
    const uint8_t MSG_CMD0 = 0x03;
    const uint8_t MSG_CMD1_DETECT = 0x81;    // 4.4.1 响应
    const uint8_t MSG_CMD1_AUTOLOCK = 0x85;  // 4.4.5 响应
    const uint8_t MSG_END = 0x79;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    FD1UART_MSG_1 _msg_1;
};
