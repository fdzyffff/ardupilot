#pragma once
/*
 * FD_msg_DYT2_telem — DYT 导引头周期状态帧解析（31 34 / 0x04 / LEN 0x31=49B）
 *
 * 帧: 31 34 04 31 <44B payload> SUM
 *   5     电子变倍(0x01~0x05 -> 1x~5x)
 *   6     DYT状态(0x00 工作 / 0x01 休眠 / 0x02 保护态 / 0x03 电锁)
 *   7     图像跟踪态(1=跟踪态 0=非跟踪态)
 *   8-9   DYT框架高低角 0.1deg 上+
 *   10-11 DYT框架方位角 0.1deg 右+
 *   12-13 图像纵向失调角 0.01deg 右+
 *   14-15 图像横向失调角 0.01deg 上+
 *   16-17 视线高低角速率 0.01deg/s 上+ (惯性系, 范围 +-300)
 *   18-19 视线方位角速率 0.01deg/s 右+
 *   20-25 DYT惯导 x/y/z 轴角速率 0.01deg/s (DYT框架系, 前右下)
 *   26    录像状态(1=正在录像) / 27 存储剩余百分比
 *   28    检测状态(1=检测到目标) / 29 功率状态(1=正常功率 0=小功率)
 *   30-47 预留 / 48 帧计数(0~255 循环) / 49 校验和(字节1..48 累加和低字节)
 * 小端字节序。
 */
#include "FD1_message.h"

#define FD_MSG_DYT2_TELEM_LEN 49
#define FD_MSG_DYT2_TELEM_RESERVED_LEN 18

class FD_msg_DYT2_telem : public FD1_message {
public:
    struct PACKED MSG_Collection {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t id;
        uint8_t len;
        uint8_t zoom;
        uint8_t state;
        uint8_t track;
        int16_t frame_pitch;
        int16_t frame_yaw;
        int16_t offset_yaw;
        int16_t offset_pitch;
        int16_t los_rate_pitch;
        int16_t los_rate_yaw;
        int16_t ins_rate_x;
        int16_t ins_rate_y;
        int16_t ins_rate_z;
        uint8_t record;
        uint8_t storage;
        uint8_t detect;
        uint8_t power;
        uint8_t reserved[FD_MSG_DYT2_TELEM_RESERVED_LEN];
        uint8_t frame_count;
        uint8_t sum;
    };

    union PACKED Content_1 {
        MSG_Collection msg;
        uint8_t data[FD_MSG_DYT2_TELEM_LEN];
    };

    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        uint16_t length = FD_MSG_DYT2_TELEM_LEN;
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
        uint8_t data[FD_MSG_DYT2_TELEM_LEN];
    } _msg;

    FD_msg_DYT2_telem();

    /* Do not allow copies */
    FD_msg_DYT2_telem(const FD_msg_DYT2_telem &other) = delete;
    FD_msg_DYT2_telem &operator=(const FD_msg_DYT2_telem&) = delete;

    const uint8_t PREAMBLE1 = 0x31;
    const uint8_t PREAMBLE2 = 0x34;
    const uint8_t MSG_ID = 0x04;
    const uint8_t MSG_LEN = 0x31;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    // 丢半帧/长时间无字节后重同步
    void reset();
    // ---- 换算后的量（deg / deg/s）----
    float frame_pitch_deg() const;      // DYT框架高低角 上+
    float frame_yaw_deg() const;        // DYT框架方位角 右+
    float offset_pitch_deg() const;     // 图像横向失调角 上+
    float offset_yaw_deg() const;       // 图像纵向失调角 右+
    float los_rate_pitch_dps() const;   // 视线高低角速率(惯性系) 上+
    float los_rate_yaw_dps() const;     // 视线方位角速率(惯性系) 右+
    float ins_rate_x_dps() const;       // DYT惯导角速率
    float ins_rate_y_dps() const;
    float ins_rate_z_dps() const;
    bool tracking() const;              // 图像跟踪态
    bool detected() const;              // 检测到目标
    uint8_t dyt_state() const;
    uint8_t zoom() const;
    uint8_t record() const;
    uint8_t storage_percent() const;
    uint8_t power_state() const;
    uint8_t frame_count() const;

    FD1UART_MSG_1 _msg_1;

private:
    // 标识/长度不符时的重同步（当前字节是帧头1 则作为新帧起点）
    void restart_or_reset(uint8_t temp);
};
