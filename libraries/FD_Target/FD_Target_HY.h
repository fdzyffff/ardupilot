#pragma once

/*
 * FD_Target_HY — 慧眼(HY)图像跟踪板目标源
 *
 * 数据来源（协议解析见 FD_UART/FD1_msg_HY_telem）：
 *   00 81 测偏数据(脱靶量) —— 制导主源，角度右正/上正，与 UAttack los_c 约定一致
 *   00 82 AI目标检测      —— 目标捕获源，选目标后发 ID 跟踪指令
 *   00 83 心跳            —— 健康监控（自检码）
 * 指令发送见 FD_UART/FD1_msg_HY_control。
 */

#include "FD_Target.h"
#include <FD_UART/FD1_msg_HY_telem.h>
#include <FD_UART/FD1_msg_HY_control.h>

class FD_Target_HY : public FD_Target_Base {
public:
    FD_Target_HY();

    static const struct AP_Param::GroupInfo var_info[];

    bool init() override;
    void update() override;
    void handle_msg(const mavlink_message_t &msg) override {}

    AP_HAL::UARTDriver *get_port() const { return _port; }

private:
    void handle_miss_msg();
    void handle_ai_msg();
    void handle_heartbeat_msg();

    // 像素坐标(左上原点, x右 y下) -> 相机角度(yaw右正, pitch上正, deg)
    bool pixel_to_angle_deg(float px, float py,
                            float &yaw_deg, float &pitch_deg) const;

    // 组包并发送当前 control 帧
    void send_control();

    void update_log_miss(int16_t yaw_cdeg, int16_t pitch_cdeg,
                         uint16_t w, uint16_t h, uint8_t status);
    void update_log_ai();
    void update_log_heartbeat(uint16_t count, uint32_t code);

    AP_Int32 target_timeout;      // TH_TOUT 脱靶量超时 ms
    AP_Int32 center_time;         // TH_CENT 无目标时重发开启指令周期 ms
    AP_Int32 track_time;          // TH_TRKT ID跟踪重发周期 ms
    AP_Float res_x;               // TH_RESX 输出图像宽 像素
    AP_Float res_y;               // TH_RESY 输出图像高 像素
    AP_Float fov_x;               // TH_FOVX 水平视场角 deg
    AP_Float fov_y;               // TH_FOVY 垂直视场角 deg
    AP_Float pix_coef;            // TH_COEF 像素模式 像素->角度系数 deg/像素
    AP_Int8  target_type;         // TH_TYPE 期望目标类型ID, -1=任意
    AP_Int8  min_conf;            // TH_CONF AI置信度阈值 0~100
    AP_Float reacq_limit_deg;     // TH_RLIM 脱靶量超过该角度则放弃跟踪重新捕获

    AP_HAL::UARTDriver *_port;
    FD1_msg_HY_miss    uart_msg_HY_miss;
    FD1_msg_HY_ai      uart_msg_HY_ai;
    FD1_msg_HY_hb      uart_msg_HY_hb;
    FD1_msg_HY_control uart_msg_HY_control;

    // 最近脱靶量（用于越限判断）
    float miss_yaw_deg = 0.0f;
    float miss_pitch_deg = 0.0f;

    // 最近一帧AI最佳目标
    bool     ai_target_valid = false;
    uint8_t  ai_target_id = 0;
    uint8_t  ai_target_type = 0;
    uint8_t  ai_target_conf = 0;
    float    ai_yaw_deg = 0.0f;
    float    ai_pitch_deg = 0.0f;
    uint32_t last_ai_ms = 0;

    uint32_t last_cmd_ms = 0;
    uint32_t last_track_ms = 0;
    uint32_t selfcheck_code = 0;
    uint32_t last_selfcheck_warn_ms = 0;
};
