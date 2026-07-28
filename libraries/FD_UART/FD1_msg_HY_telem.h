#pragma once
/*
 * FD1_msg_HY_telem — 慧眼(HY)图像跟踪板反馈报文解析
 *
 * 协议：慧眼_串口通信协议 v3.1
 *   反馈帧: 78 07 CMD0 CMD1 LEN DATA... CHK 79
 *   小端字节序；CHK = CMD0(含) 至最后一个数据字节(含)的累加和低字节，不含帧头/CHK/帧尾
 *
 * 本文件含三个接收类（同一串口字节流喂给三个解析器，各自按 CMD1 过滤）：
 *   FD1_msg_HY_miss  00 81 测偏数据(脱靶量)，定长
 *   FD1_msg_HY_ai    00 82 AI目标检测，变长
 *   FD1_msg_HY_hb    00 83 心跳，定长
 */
#include "FD1_message.h"

#define FD1_MSG_HY_MISS_LEN      21     // 2头+2命令+1长度+14数据+1校验+1帧尾
#define FD1_MSG_HY_HB_LEN        13     // ...+6数据...
#define FD1_MSG_HY_AI_MAX_DATA   245    // 3 + 22目标*11字节
#define FD1_MSG_HY_AI_MAX_LEN    (FD1_MSG_HY_AI_MAX_DATA + 7)

// ---------------------------------------------------------------------------
// 00 81 测偏数据(脱靶量)报文
// ---------------------------------------------------------------------------
class FD1_msg_HY_miss : public FD1_message {
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // 测偏量：像素(int32)或角度(float)，由 status bit2 指示
    union PACKED HY_miss_value {
        int32_t i;
        float f;
    };

    struct PACKED MSG_Collection {
        FD1_msg_header header;   // 0x78 0x07
        uint8_t cmd0;            // 0x00
        uint8_t cmd1;            // 0x81
        uint8_t len;             // 0x0E
        uint8_t status;          // bit2 1=角度(float)/0=像素; bit1 1=算法停止; bit0 1=数据有效
        uint8_t channel;         // 视频通道ID
        HY_miss_value offset_x;  // 左右测偏量，右正左负
        HY_miss_value offset_y;  // 上下测偏量，上正下负
        uint16_t width;          // 目标宽 像素
        uint16_t height;         // 目标高 像素
        uint8_t sum;
        uint8_t end;             // 0x79
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
    const uint8_t MSG_CMD0  = 0x00;
    const uint8_t MSG_CMD1  = 0x81;
    const uint8_t MSG_LEN   = 0x0E;
    const uint8_t MSG_END   = 0x79;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    FD1UART_MSG_1 _msg_1;
};

// ---------------------------------------------------------------------------
// 00 82 AI目标检测报文（变长）
// ---------------------------------------------------------------------------
class FD1_msg_HY_ai : public FD1_message {
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    struct PACKED HY_ai_target {   // 每目标11字节
        uint8_t id;        // 目标编号 1~64
        uint8_t type;      // 类型ID 0~14, 15=质心目标
        uint8_t conf;      // 置信度 0~100
        uint16_t x;        // 左上角X
        uint16_t y;        // 左上角Y
        uint16_t w;        // 宽
        uint16_t h;        // 高
    };

    // 注意：本帧变长，sum/end 在 data[5+len]/data[6+len]，
    // 结构体的固定偏移仅在 len 最大时有效；头部与 targets 字段可正常按名访问
    struct PACKED MSG_Collection {
        FD1_msg_header header;   // 0x78 0x07
        uint8_t cmd0;            // 0x00
        uint8_t cmd1;            // 0x82
        uint8_t len;
        uint8_t frame_id;        // 帧ID 0~255循环
        uint8_t total;           // 总目标个数(>22时分包)
        uint8_t count;           // 本报文目标个数(<=22)
        HY_ai_target targets[22];
        uint8_t sum;             // 变长帧：位置不固定，见上注释
        uint8_t end;
    };

    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_HY_AI_MAX_LEN];
    };

    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD1_MSG_HY_AI_MAX_LEN;
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
        uint8_t data[FD1_MSG_HY_AI_MAX_LEN];
    } _msg;

    FD1_msg_HY_ai();

    FD1_msg_HY_ai(const FD1_msg_HY_ai &other) = delete;
    FD1_msg_HY_ai &operator=(const FD1_msg_HY_ai&) = delete;

    const uint8_t PREAMBLE1 = 0x78;
    const uint8_t PREAMBLE2 = 0x07;
    const uint8_t MSG_CMD0  = 0x00;
    const uint8_t MSG_CMD1  = 0x82;
    const uint8_t MSG_END   = 0x79;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    FD1UART_MSG_1 _msg_1;
};

// ---------------------------------------------------------------------------
// 00 83 心跳报文
// ---------------------------------------------------------------------------
class FD1_msg_HY_hb : public FD1_message {
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    struct PACKED MSG_Collection {
        FD1_msg_header header;   // 0x78 0x07
        uint8_t cmd0;            // 0x00
        uint8_t cmd1;            // 0x83
        uint8_t len;             // 0x06
        uint16_t count;          // 报文计数
        uint32_t code;           // 自检码，0正常
        uint8_t sum;
        uint8_t end;             // 0x79
    };

    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_HY_HB_LEN];
    };

    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD1_MSG_HY_HB_LEN;
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
        uint8_t data[FD1_MSG_HY_HB_LEN];
    } _msg;

    FD1_msg_HY_hb();

    FD1_msg_HY_hb(const FD1_msg_HY_hb &other) = delete;
    FD1_msg_HY_hb &operator=(const FD1_msg_HY_hb&) = delete;

    const uint8_t PREAMBLE1 = 0x78;
    const uint8_t PREAMBLE2 = 0x07;
    const uint8_t MSG_CMD0  = 0x00;
    const uint8_t MSG_CMD1  = 0x83;
    const uint8_t MSG_LEN   = 0x06;
    const uint8_t MSG_END   = 0x79;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    FD1UART_MSG_1 _msg_1;
};
