#pragma once
/*
 * FD1_msg_HY_telem — 慧眼(HY) 4.1.3 测偏数据(脱靶量)报文解析
 *
 * 反馈帧: 78 07 00 81 0E DATA[14] CHK 79
 * 小端字节序；CHK为CMD0至最后一个数据字节的累加和低字节。
 */
#include "FD1_message.h"

#define FD1_MSG_HY_MISS_LEN 21

class FD1_msg_HY_miss : public FD1_message {
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    union PACKED HY_miss_value {
        int32_t i;
        float f;
    };

    struct PACKED MSG_Collection {
        FD1_msg_header header;
        uint8_t cmd0;
        uint8_t cmd1;
        uint8_t len;
        uint8_t status;
        uint8_t channel;
        HY_miss_value offset_x;
        HY_miss_value offset_y;
        uint16_t width;
        uint16_t height;
        uint8_t sum;
        uint8_t end;
    };

    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_HY_MISS_LEN];
    };

    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD1_MSG_HY_MISS_LEN;
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
        uint8_t count;
        uint8_t sum;
        uint8_t data[FD1_MSG_HY_MISS_LEN];
    } _msg;

    FD1_msg_HY_miss();

    FD1_msg_HY_miss(const FD1_msg_HY_miss &other) = delete;
    FD1_msg_HY_miss &operator=(const FD1_msg_HY_miss&) = delete;

    const uint8_t PREAMBLE1 = 0x78;
    const uint8_t PREAMBLE2 = 0x07;
    const uint8_t MSG_CMD0 = 0x00;
    const uint8_t MSG_CMD1 = 0x81;
    const uint8_t MSG_LEN = 0x0E;
    const uint8_t MSG_END = 0x79;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    FD1UART_MSG_1 _msg_1;
};
