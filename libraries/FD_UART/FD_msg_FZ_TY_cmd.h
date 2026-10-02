#pragma once
/*
 * FD_msg_FZ_TY_cmd — 引信安解装置(TY) 下行指令组包
 *
 * 帧: AA 4C D1 D2 CHK 55
 *   D1 = 命令字: 0x11 自检 / 0x12 装定工作模式 / 0x13 一解 / 0x14 二解 / 0x15 起爆
 *   D2 = D1 (冗余)；仅 D1=0x12 时 D2 = 工作模式 (0 无装定 / 1 触发 / 2 近炸+触发 / 3 指令起爆)
 *   CHK = (0x4C + D1 + D2) 取低字节
 */
#include "FD1_message.h"

#define FD_MSG_FZ_TY_CMD_LEN 6

class FD_msg_FZ_TY_cmd : public FD1_message {
public:
    struct PACKED MSG_Collection {
        uint8_t head_1;
        uint8_t dest;
        uint8_t cmd;
        uint8_t arg;
        uint8_t sum;
        uint8_t end;
    };

    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD_MSG_FZ_TY_CMD_LEN];
    };

    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD_MSG_FZ_TY_CMD_LEN;
        Content_1 content;
    };

    struct PACKED FD1UART_msg_parser {
        enum {
            FD1UART_PREAMBLE1 = 0,
        } msg_state;

        uint16_t read;
        uint8_t data[FD_MSG_FZ_TY_CMD_LEN];
    } _msg;

    FD_msg_FZ_TY_cmd();

    /* Do not allow copies */
    FD_msg_FZ_TY_cmd(const FD_msg_FZ_TY_cmd &other) = delete;
    FD_msg_FZ_TY_cmd &operator=(const FD_msg_FZ_TY_cmd&) = delete;

    const uint8_t HEAD_1 = 0xAA;
    const uint8_t DEST_DEVICE = 0x4C;   // 数据发往控制电路

    const uint8_t CMD_SELFTEST = 0x11;
    const uint8_t CMD_SET_MODE = 0x12;
    const uint8_t CMD_ARM1 = 0x13;
    const uint8_t CMD_ARM2 = 0x14;
    const uint8_t CMD_FIRE = 0x15;

    const uint8_t MODE_NONE = 0;        // 无装定(不响应一解/二解/起爆)
    const uint8_t MODE_TRIGGER = 1;
    const uint8_t MODE_PROX_TRIGGER = 2;
    const uint8_t MODE_CMD_FIRE = 3;

    void process_message(void) override {}
    void parse(uint8_t temp) override { (void)temp; }
    void swap_message() override {}

    void pack(uint8_t cmd, uint8_t arg);
    void pack_selftest();
    void pack_set_mode(uint8_t mode);
    void pack_arm1();
    void pack_arm2();
    void pack_fire();
    void make_sum();

    FD1UART_MSG_1 _msg_1;
};
