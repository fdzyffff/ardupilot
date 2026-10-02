#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>

// HUOJIAN: 工程自定义 MAVLink 收发模块（结构参照 Copter-4.7.0-ZFJL 的 FD_DATA）。
// 现状只含 CHASE 协议 11050/11051/11052 三条上行报文；接收入口与参数组骨架保留，便于后续扩展。

class FD_DATA
{
public:
    // CHASE_STATUS(11050) 上报数据，字段含义见地面指控接口协议 V2.0
    struct chase_status_t {
        uint8_t state;          // 任务阶段 0~10
        uint8_t armed;          // 解锁标志
        uint8_t gcs_ok;         // 地面站在线
        uint8_t hb_ok;          // 导引头心跳在线
        uint8_t bbox_valid;     // 目标识别有效
        uint8_t lla_valid;      // 引导坐标有效
        uint8_t chase_valid;    // 追击点有效
        uint8_t relay_state;    // IO 输出状态位图
        float throttle;         // 当前油门输出 0~1
    };

    // CHASE_FUZE(11052) 上报数据: 探测器目标距离 + 引信状态
    struct chase_fuze_t {
        float det_dist1;        // 目标1距离 m
        float det_dist2;        // 目标2距离 m
        float det_dist3;        // 目标3距离 m
        float det_dist4;        // 目标4距离 m
        int8_t det_selftest;    // 探测器自检: 0 未收到 1 正常 2 异常
        uint8_t det_zg_zd;      // 炸高装定确认值(m x2), 0 未确认
        uint8_t det_target_count;   // 目标个数
        uint8_t fz_mode;        // 引信工作模式 0 无装定 1 触发 2 近炸+触发 3 指令起爆
        uint8_t fz_arm1;        // 一解命令已收到
        uint8_t fz_arm2;        // 二解命令已收到
        uint8_t fz_pwr_ok;      // 工作电源大于14V
        uint8_t fz_locked;      // 引信自锁
    };

    FD_DATA();

    /* Do not allow copies */
    FD_DATA(const FD_DATA &other) = delete;
    FD_DATA &operator=(const FD_DATA &other) = delete;

    static FD_DATA *get_singleton()
    {
        return _singleton;
    }

    static const AP_Param::GroupInfo var_info[];

    // 1Hz 周期处理（扩展点）
    void update();

    // 自定义 MAVLink 接收统一入口，由 GCS_MAVLINK_Copter::packetReceived 调用
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg);

    // 业务侧(UChase)填数据
    void set_chase_status(const chase_status_t &s)
    {
        _chase_status = s;
    }

    // 业务侧(UDetector/UPayload)填数据
    void set_chase_fuze(const chase_fuze_t &f)
    {
        _chase_fuze = f;
    }

    // 上行发送，由 GCS_MAVLINK::try_send_message 按流速率调用
    void send_chase_status(mavlink_channel_t chan);
    void send_chase_tracker(mavlink_channel_t chan);
    void send_chase_fuze(mavlink_channel_t chan);

private:
    static FD_DATA *_singleton;

    chase_status_t _chase_status {};
    chase_fuze_t _chase_fuze {};
};

namespace AP {
FD_DATA &fd_data();
}
