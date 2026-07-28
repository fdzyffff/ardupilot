#pragma once
/*
 * FD1_msg_HY_control — 慧眼(HY)图像跟踪板控制指令组包
 *
 * 协议：慧眼_串口通信协议 v3.1
 *   指令帧: 58 07 CMD0 CMD1 LEN DATA... CHK 59
 *   CHK = CMD0(含) 至最后一个数据字节(含)的累加和低字节
 *
 * 变长帧：len 决定 sum/end 实际位置，组包一律通过 content.data[] 按下标写，
 * MSG_Collection 结构体的 sum/end 字段仅在 len=10 时与实际偏移重合。
 */
#include "FD1_message.h"

#define FD1_MSG_HY_CONTROL_MAX_LEN 17   // 2头+2命令+1长度+10数据+1校验+1帧尾

class FD1_msg_HY_control : public FD1_message {
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    struct PACKED MSG_Collection {
        FD1_msg_header header;   // 0x58 0x07
        uint8_t cmd0;
        uint8_t cmd1;
        uint8_t len;
        uint8_t payload[10];
        uint8_t sum;             // 变长：位置见文件头注释
        uint8_t end;             // 0x59
    };

    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD1_MSG_HY_CONTROL_MAX_LEN];
    };

    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD1_MSG_HY_CONTROL_MAX_LEN;
        Content_1 content;
    };

    // 本类仅用于发送，不解析接收流；保留解析结构以贴合库风格
    struct PACKED FD1UART_msg_parser {
        enum {
            FD1UART_PREAMBLE1 = 0,
            FD1UART_PREAMBLE2,
            FD1UART_INFO,
            FD1UART_DATA,
            FD1UART_SUM,
        } msg_state;

        uint16_t read;
        uint16_t length;
        uint8_t count;
        uint8_t sum;
        uint8_t data[FD1_MSG_HY_CONTROL_MAX_LEN];
    } _msg;

    FD1_msg_HY_control();

    FD1_msg_HY_control(const FD1_msg_HY_control &other) = delete;
    FD1_msg_HY_control &operator=(const FD1_msg_HY_control&) = delete;

    static const uint8_t PREAMBLE1 = 0x58;
    static const uint8_t PREAMBLE2 = 0x07;
    static const uint8_t MSG_END   = 0x59;

    void process_message(void) override {}
    void parse(uint8_t temp) override { (void)temp; }   // 指令响应由 telem 侧按需扩展
    void swap_message() override {}

    // 组包接口（组完置 need_send，发送 content.data 前 length 字节）
    void pack_open_detection();        // 03 01 开启目标检测+多目标跟踪
    void pack_close_detection();       // 03 01 关闭目标检测
    void pack_detect_all_types();      // 03 03 检测所有AI目标类型
    void pack_auto_lock();             // 03 05 自动锁定(循环, 最接近十字策略)
    void pack_stop_track();            // 03 11 停止跟踪
    void pack_track_id(uint8_t id);    // 03 11 ID跟踪

    void make_sum();

    FD1UART_MSG_1 _msg_1;
};
