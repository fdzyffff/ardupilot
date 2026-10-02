#pragma once
/*
 * FD_msg_DYT2_ack — DYT 导引头回令帧解析（31 31 / 标识 0x05）
 *
 * 自检回令: 31 31 05 06 <结果码> SUM
 *   结果码: 0x00 正常 / 0x01 图像传感器异常 / 0x02 伺服组件异常
 * 设置回令: 31 31 05 0A <B> <C int16> <D int16> SUM
 *   B/C/D 回显该指令对应的当前状态（与下行设置指令同一字段定义）
 * LEN = 整帧字节数(6 或 10)；SUM = 字节1..字节N-1 累加和取低字节；小端字节序。
 */
#include "FD1_message.h"

#define FD_MSG_DYT2_ACK_MAX_LEN 10

class FD_msg_DYT2_ack : public FD1_message {
public:
    struct PACKED MSG_Collection {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t id;
        uint8_t len;
        uint8_t b;
        int16_t c;
        int16_t d;
        uint8_t sum;
    };

    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD_MSG_DYT2_ACK_MAX_LEN];
    };

    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = 0;
        Content_1 content;
    };

    struct PACKED FD1UART_msg_parser {
        enum {
            FD1UART_PREAMBLE1 = 0,
            FD1UART_PREAMBLE2,
            FD1UART_ID,
            FD1UART_LEN,
            FD1UART_DATA,
            FD1UART_SUM,
        } msg_state;

        uint16_t read;
        uint16_t length;
        uint8_t sum;
        uint8_t data[FD_MSG_DYT2_ACK_MAX_LEN];
    } _msg;

    FD_msg_DYT2_ack();

    /* Do not allow copies */
    FD_msg_DYT2_ack(const FD_msg_DYT2_ack &other) = delete;
    FD_msg_DYT2_ack &operator=(const FD_msg_DYT2_ack&) = delete;

    const uint8_t PREAMBLE1 = 0x31;
    const uint8_t PREAMBLE2 = 0x31;
    const uint8_t MSG_ID = 0x05;
    const uint8_t MSG_LEN_SELFTEST = 0x06;
    const uint8_t MSG_LEN_SET = 0x0A;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    // 丢半帧/长时间无字节后重同步
    void reset();

    bool is_selftest() const;       // 自检回令(6B)
    bool is_set_ack() const;        // 设置指令回令(10B)
    uint8_t selftest_result() const;
    uint8_t set_cmd() const;        // 设置回令的 B 字段（指令码）
    int16_t set_c() const;
    int16_t set_d() const;

    FD1UART_MSG_1 _msg_1;

private:
    // 标识/长度不符时的重同步（当前字节是帧头1 则作为新帧起点）
    void restart_or_reset(uint8_t temp);
};
