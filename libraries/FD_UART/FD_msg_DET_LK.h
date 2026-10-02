#pragma once
/*
 * FD_msg_DET_LK — 近距探测器(lk) 目标信息帧解析
 *
 * 上行帧: 5A 5A N D1..DN
 *   无校验和、无帧尾；单工只上报（探测器不接收任何下行）。
 *   每字节表示目标距离: 实际距离 = 上报值 x 0.25 m（字节上限 255 -> 63.75m）。
 *   文档只说明"一帧最多 8 个目标、上报只取前 4"，故 N 按 1~8 校验，越界即重同步。
 */
#include "FD1_message.h"

#define FD_MSG_DET_LK_MAX_TARGETS 8
#define FD_MSG_DET_LK_LEN (3 + FD_MSG_DET_LK_MAX_TARGETS)

class FD_msg_DET_LK : public FD1_message {
public:
    struct PACKED MSG_Collection {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t count;
        uint8_t dist[FD_MSG_DET_LK_MAX_TARGETS];
    };

    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD_MSG_DET_LK_LEN];
    };

    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD_MSG_DET_LK_LEN;
        Content_1 content;
    };

    struct PACKED FD1UART_msg_parser {
        enum {
            FD1UART_PREAMBLE1 = 0,
            FD1UART_PREAMBLE2,
            FD1UART_COUNT,
            FD1UART_DATA,
        } msg_state;

        uint16_t read;
        uint8_t count;
        uint8_t data[FD_MSG_DET_LK_LEN];
    } _msg;

    FD_msg_DET_LK();

    /* Do not allow copies */
    FD_msg_DET_LK(const FD_msg_DET_LK &other) = delete;
    FD_msg_DET_LK &operator=(const FD_msg_DET_LK&) = delete;

    const uint8_t PREAMBLE1 = 0x5A;
    const uint8_t PREAMBLE2 = 0x5A;
    const uint8_t MAX_TARGETS = FD_MSG_DET_LK_MAX_TARGETS;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    // 长时间无字节后重同步，避免半帧导致粘帧
    void reset();

    // 上报值换算实际距离(m)
    float distance_m(uint8_t idx) const;

    FD1UART_MSG_1 _msg_1;
};
