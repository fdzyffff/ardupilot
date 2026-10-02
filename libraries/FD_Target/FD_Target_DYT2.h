#pragma once

#include "FD_Target_Cam.h"

#include <FD_UART/FD_msg_DYT2_cmd.h>
#include <FD_UART/FD_msg_DYT2_telem.h>
#include <FD_UART/FD_msg_DYT2_ack.h>

/*
 * FD_Target_DYT2 —— 反无吊舱(DYT) 导引头目标源（协议: 2026-09-26 接口及协议）
 *
 * 上行(导引头->飞控): 31 34/04 周期状态(49B, 默认 50Hz, 上电配到 100Hz)；
 *                     31 31/05 回令(自检 6B / 设置 10B)。
 * 下行(飞控->导引头): 36 31/02 设置指令(归中 D4H / 跟踪模式-自动 B4H / 通讯频率修改)；
 *                     36 31/03 飞机状态(28B, 按 CAM_STIME 周期发送)。
 *
 * 目标有效: 图像跟踪态=1 且 检测到目标=1(仅供本类判定是否需要发归中)。
 * 归中 + 跟踪模式-自动(1s 后发第二条) 的两条通道：
 *   ① 周期通道: 只在"没目标"时（未锁定/收不到状态帧）每 UATK_TD_CENT(默认 5s) 一次
 *   ② 事件通道: 框架高低/方位角超 UATK_TD_ANG(默认 30°) 立即发一次（不等 5s 周期——
 *      飞机转弯时角度快速超限，等周期太慢）；跟踪态下同样适用（角度还大多半是锁错目标），
 *      1s 内不重复以免归中过程中刷屏
 * 取数: 目标角度 = 框架角 + 失调角（相对导引头自身 0 位 → set_type(0)，与 HY 同口径，
 * 锁定/检测到目标时经 handle_info() 喂给基类，成为相机源）; 视线高低/方位角速率(惯性系)
 * 经 handle_info_rate() 发布（r1=方位 右+ / r2=高低 上+）; 另记录 DYT框架角与图像失调角。
 * 帧率由本类 1s 窗口自行统计。
 */
class FD_Target_DYT2 : public FD_Target_Cam {
public:
    FD_Target_DYT2();

    static const struct AP_Param::GroupInfo var_info[];

    bool init() override;
    void update() override;

    // 帧率覆写：本类自统计 1s 窗口帧率（串口链路在线即有值，不受基类 _valid 影响）
    float get_fps() const override { return fps; }
    // 本类带 LOS 角速率通道（导引头自带的视线角速率，协议 16-19 字节）
    bool has_rate_channel() const override { return true; }

    // ---- 导引头给的目标量（无有效帧时保持最后一次值，有效性看 is_valid()）----
    float get_los_rate_pitch_dps() const { return uart_msg_telem.los_rate_pitch_dps(); }  // 视线高低角速率 上+
    float get_los_rate_yaw_dps() const { return uart_msg_telem.los_rate_yaw_dps(); }      // 视线方位角速率 右+
    float get_frame_pitch_deg() const { return uart_msg_telem.frame_pitch_deg(); }        // DYT框架高低角 上+
    float get_frame_yaw_deg() const { return uart_msg_telem.frame_yaw_deg(); }            // DYT框架方位角 右+
    float get_offset_pitch_deg() const { return uart_msg_telem.offset_pitch_deg(); }      // 图像横向失调角 上+
    float get_offset_yaw_deg() const { return uart_msg_telem.offset_yaw_deg(); }          // 图像纵向失调角 右+

private:
    // 事件触发最小间隔 ms：角度超阈值走"立即发"通道，但归中过程中框架角仍超阈值，
    // 1s 内不重复，避免按 update 频率(200Hz)刷屏（周期通道仍是 CAM_CENT=5s）
    static const uint32_t EVENT_HOLD_MS = 1000;
    // 归中(D4H) 与 自动跟踪(B4H) 的间隔 ms：协议为发送-回令式，同一毫秒连发两条
    // 设备可能只处理一条，故归中后延迟这么久再发自动跟踪
    static const uint32_t TRACK_DELAY_MS = 1000;
    // 上电频率配置：发送次数与间隔（防上电竞态）
    static const uint8_t CFG_SEND_TIMES = 3;
    static const uint32_t CFG_PERIOD_MS = 500;

    bool target_locked() const;
    void handle_telem();
    void handle_ack();
    void run_startup_config(uint32_t now_ms);
    void send_vehicle_state(uint32_t now_ms);
    void send_center();
    void send_auto_track();
    void update_log();

    // CAM_CENT 无目标重发周期 ms（数据超时后保底重发）
    AP_Int32 center_time;
    // CAM_ANG 归中触发阈值 deg（框架高低/方位角超此值立即发一次）
    AP_Float center_angle_deg;
    // CAM_STIME 飞机状态帧周期 ms（默认 20 = 50Hz）
    AP_Int32 state_time;
    // CAM_RATE 导引头上报频率档位 1..5 -> 10/25/50/75/100Hz（默认 5 = 100Hz）
    AP_Int8 report_rate;

    FD_msg_DYT2_cmd uart_msg_cmd;
    FD_msg_DYT2_telem uart_msg_telem;
    FD_msg_DYT2_ack uart_msg_ack;

    uint32_t last_telem_ms;    // 最近一次收到状态帧的时刻
    uint32_t last_send_ms;     // 上次"归中+自动跟踪"发送时刻
    uint32_t last_state_ms;    // 上次飞机状态帧发送时刻
    uint32_t last_log_ms;
    uint32_t cfg_last_ms;
    uint8_t cfg_send_count;
    uint32_t last_fps_ms;      // 状态帧帧率统计窗口起点
    uint16_t frame_cnt;        // 窗口内状态帧计数
    float fps;                 // 状态帧帧率(1s 窗口，_valid 恒假时不走基类统计)
    uint32_t center_sent_ms;      // 上次归中发送时刻(自动跟踪延迟基准)
    bool track_pending;           // 归中已发、自动跟踪待发
};
