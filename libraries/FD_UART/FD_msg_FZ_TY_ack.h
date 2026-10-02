#pragma once
/*
 * FD_msg_FZ_TY_ack — 引信安解装置(TY) 回帧解析
 *
 * 帧: BB C4 D1 D2 CHK 55，D2 = D1 (冗余)
 *   D1: bit0-1 装定工作模式 / bit2 已收一解 / bit3 已收二解 / bit4 工作电源>14V / bit5 已自锁
 *   CHK = (0xC4 + D1 + D2) 取低字节
 */
#include "FD1_message.h"

#define FD_MSG_FZ_TY_ACK_LEN 6

class FD_msg_FZ_TY_ack : public FD1_message {
public:
    struct PACKED MSG_Collection {
        uint8_t head_1;
        uint8_t dest;
        uint8_t status;
        uint8_t status_dup;
        uint8_t sum;
        uint8_t end;
    };

    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD_MSG_FZ_TY_ACK_LEN];
    };

    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD_MSG_FZ_TY_ACK_LEN;
        Content_1 content;
    };

    struct PACKED FD1UART_msg_parser {
        enum {
            FD1UART_PREAMBLE1 = 0,
            FD1UART_PREAMBLE2,
            FD1UART_DATA,
            FD1UART_SUM,
            FD1UART_END,
        } msg_state;

        uint16_t read;
        uint8_t data[FD_MSG_FZ_TY_ACK_LEN];
    } _msg;

    FD_msg_FZ_TY_ack();

    /* Do not allow copies */
    FD_msg_FZ_TY_ack(const FD_msg_FZ_TY_ack &other) = delete;
    FD_msg_FZ_TY_ack &operator=(const FD_msg_FZ_TY_ack&) = delete;

    const uint8_t HEAD_1 = 0xBB;
    const uint8_t DEST_UAV = 0xC4;      // 数据发往无人机
    const uint8_t END = 0x55;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    uint8_t mode() const;               // bit0-1
    uint8_t arm1() const;               // bit2
    uint8_t arm2() const;               // bit3
    uint8_t pwr_ok() const;             // bit4
    uint8_t locked() const;             // bit5

    FD1UART_MSG_1 _msg_1;
};
