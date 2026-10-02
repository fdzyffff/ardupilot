#pragma once
/*
 * FD_msg_DYT2_cmd — 反无吊舱(DYT) 下行组包（协议 2026-09-26 接口及协议）
 *
 * 飞控->DYT 帧: 36 31 ID LEN payload... SUM
 *   LEN = 整帧字节数(含帧头与校验和)；SUM = 字节1..字节N 累加和取低字节
 *   0x02 设置指令      LEN 0x0A : B(1B) + C(int16) + D(int16)
 *   0x03 飞机状态(周期) LEN 0x1C : 23 字节 payload
 * 小端字节序，多字节量低字节先发。
 * 本类只做"飞控发得出去"的组包；B 字段指令码由调用方给出（见 FD_Target_DYT2）。
 */
#include "FD1_message.h"

#define FD_MSG_DYT2_CMD_SET_LEN 10
#define FD_MSG_DYT2_CMD_STATE_LEN 28

class FD_msg_DYT2_cmd : public FD1_message {
public:
    struct PACKED MSG_Set {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t id;
        uint8_t len;
        uint8_t b;
        int16_t c;
        int16_t d;
        uint8_t sum;
    };

    // 飞机状态: 帧头 4B + payload 23B + 校验 1B = 28B (LEN 0x1C)
    struct PACKED MSG_State {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t id;
        uint8_t len;
        int16_t vel_n;          // 北向速度 0.01 m/s
        int16_t vel_e;          // 东向速度 0.01 m/s
        int16_t vel_d;          // 升降速度 0.01 m/s
        int16_t ground_speed;   // 卫星地速 0.2 m/s
        int16_t pitch;          // 俯仰角 0.01 deg
        int16_t roll;           // 滚转角 0.01 deg
        uint16_t heading;       // 磁航向角 0.01 deg
        uint16_t tilt;          // 倾转角 0.1 deg
        uint16_t dist_reframe;  // bit0-11 弹目距离 0.1m / bit12-15 重新框选次数
        uint32_t timestamp_ms;  // 飞控时间戳 ms
        uint8_t reserved;
        uint8_t sum;
    };

    // 飞机状态输入（工程单位，组包时按协议分辨率换算）
    struct State {
        float vel_n_ms;
        float vel_e_ms;
        float vel_d_ms;
        float ground_speed_ms;
        float pitch_deg;
        float roll_deg;
        float heading_deg;      // 0~360
        float tilt_deg;
        uint16_t dist_reframe;
        uint32_t timestamp_ms;
    };

    union PACKED Content_Set {
        MSG_Set msg;
        uint8_t data[FD_MSG_DYT2_CMD_SET_LEN];
    };

    union PACKED Content_State {
        MSG_State msg;
        uint8_t data[FD_MSG_DYT2_CMD_STATE_LEN];
    };

    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD_MSG_DYT2_CMD_SET_LEN;
        Content_Set content;
    };

    struct PACKED FD1UART_MSG_2 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD_MSG_DYT2_CMD_STATE_LEN;
        Content_State content;
    };

    FD_msg_DYT2_cmd();

    /* Do not allow copies */
    FD_msg_DYT2_cmd(const FD_msg_DYT2_cmd &other) = delete;
    FD_msg_DYT2_cmd &operator=(const FD_msg_DYT2_cmd&) = delete;

    static const uint8_t PREAMBLE1 = 0x36;
    static const uint8_t PREAMBLE2 = 0x31;
    static const uint8_t MSG_ID_SET = 0x02;
    static const uint8_t MSG_ID_STATE = 0x03;
    // B 字段指令码
    static const uint8_t B_CENTER = 0xD4;       // 归中(DYT方位0度/俯仰0度/增稳)
    static const uint8_t B_AUTO_TRACK = 0xB4;   // 跟踪模式-自动

    void process_message(void) override {}
    void parse(uint8_t temp) override { (void)temp; }
    void swap_message() override {}

    // 通用设置指令: B 字段 + C/D 参数（该指令无参数时 C/D 填 0）
    void pack_set(uint8_t b, int16_t c, int16_t d);
    void pack_center();
    void pack_auto_track();
    // 通讯频率修改: cmd 为 B 字段指令码(默认 0xBD)，level 1..5 -> 10/25/50/75/100Hz
    void pack_report_rate(uint8_t cmd, uint8_t level);
    void pack_vehicle_state(const State &state);

    void make_sum_set();
    void make_sum_state();

    FD1UART_MSG_1 _msg_1;
    FD1UART_MSG_2 _msg_2;
};
