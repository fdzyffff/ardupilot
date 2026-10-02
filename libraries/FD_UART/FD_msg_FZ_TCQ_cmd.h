#pragma once
/*
 * FD_msg_FZ_TCQ_cmd — 引信(TCQ) 下行指令组包
 *
 * 自检: AA BB 33 CHK   (CHK = 前三字节累加和取低字节)
 * 装定: AA 55 ZG CHK   (ZG = 实际炸高 x2，0.5m 步进；例 8m->AA 55 10 0F，4m->AA 55 08 07)
 */
#include "FD1_message.h"

#define FD_MSG_FZ_TCQ_CMD_LEN 4

class FD_msg_FZ_TCQ_cmd : public FD1_message {
public:
    struct PACKED MSG_Collection {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t value;
        uint8_t sum;
    };

    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD_MSG_FZ_TCQ_CMD_LEN];
    };

    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD_MSG_FZ_TCQ_CMD_LEN;
        Content_1 content;
    };

    struct PACKED FD1UART_msg_parser {
        enum {
            FD1UART_PREAMBLE1 = 0,
        } msg_state;

        uint16_t read;
        uint8_t data[FD_MSG_FZ_TCQ_CMD_LEN];
    } _msg;

    FD_msg_FZ_TCQ_cmd();

    /* Do not allow copies */
    FD_msg_FZ_TCQ_cmd(const FD_msg_FZ_TCQ_cmd &other) = delete;
    FD_msg_FZ_TCQ_cmd &operator=(const FD_msg_FZ_TCQ_cmd&) = delete;

    const uint8_t PREAMBLE1 = 0xAA;
    const uint8_t SELFTEST_HEAD2 = 0xBB;
    const uint8_t SELFTEST_CMD = 0x33;
    const uint8_t ZG_HEAD2 = 0x55;

    void process_message(void) override {}
    void parse(uint8_t temp) override { (void)temp; }
    void swap_message() override {}

    // 自检指令
    void pack_selftest();
    // 炸高装定: zg_x2 = 实际炸高 x2 (1.0~8.0m -> 2~16)
    void pack_zg(uint8_t zg_x2);
    void make_sum();

    FD1UART_MSG_1 _msg_1;
};
