#pragma once
/*
 * FD_msg_FZ_TCQ_ack — 引信(TCQ) 回帧解析
 *
 * 自检返回: 55 BB DATA CHK  (DATA: 0x55 正常 / 0xAA 异常)
 * 装定返回: AA 55 ZG   CHK  (ZG 与装定值一致)
 * 校验和: 前三字节累加和取低字节。
 */
#include "FD1_message.h"

#define FD_MSG_FZ_TCQ_ACK_LEN 4

class FD_msg_FZ_TCQ_ack : public FD1_message {
public:
    struct PACKED MSG_Collection {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t value;
        uint8_t sum;
    };

    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD_MSG_FZ_TCQ_ACK_LEN];
    };

    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD_MSG_FZ_TCQ_ACK_LEN;
        Content_1 content;
    };

    struct PACKED FD1UART_msg_parser {
        enum {
            FD1UART_PREAMBLE1 = 0,
            FD1UART_PREAMBLE2,
            FD1UART_DATA,
            FD1UART_SUM,
        } msg_state;

        uint16_t read;
        uint8_t data[FD_MSG_FZ_TCQ_ACK_LEN];
    } _msg;

    FD_msg_FZ_TCQ_ack();

    /* Do not allow copies */
    FD_msg_FZ_TCQ_ack(const FD_msg_FZ_TCQ_ack &other) = delete;
    FD_msg_FZ_TCQ_ack &operator=(const FD_msg_FZ_TCQ_ack&) = delete;

    const uint8_t SELFTEST_HEAD1 = 0x55;
    const uint8_t SELFTEST_HEAD2 = 0xBB;
    const uint8_t ZG_HEAD1 = 0xAA;
    const uint8_t ZG_HEAD2 = 0x55;

    const uint8_t SELFTEST_OK = 0x55;
    const uint8_t SELFTEST_FAIL = 0xAA;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    // 自检回帧判定: 1 正常 / 2 异常 / 0 未收到
    int8_t selftest_result() const;
    // 装定确认值(实际炸高 x2)，0 表示未确认
    uint8_t zg_zd() const;
    bool is_selftest() const;

    FD1UART_MSG_1 _msg_1;
};
