#pragma once

#include <GCS_MAVLink/GCS.h>
#include <AP_Common/Location.h>

// HUOJIAN: 地面指控通信层——CHASE 协议上行 11050/11051/11052 与下行指令(31010~31018、181)
class UChase {
public:
    void init();
    // 10Hz 调度，内部按 1Hz 发送三个上报报文
    void update();
    // 收到任意地面站报文时调用，用于 gcs_ok 计时
    void msg_received(const mavlink_message_t &msg);
    // CHASE 协议下行指令，返回 true 表示本模块已处理（result 为应答结果）
    bool handle_command_int(const mavlink_command_int_t &packet, MAV_RESULT &result);
    // 协议任务阶段 state(0~10)
    uint8_t get_mission_state() const;

    // HUOJIAN: 31011 目标位置(靶机信息)有效性——超过 5s 未更新视为目标丢失
    bool have_target() const { return _target_loc_valid; }
    Location get_target_loc() const { return _target_loc; }
    // HUOJIAN: 31011 附带的靶机速度(N/E m/s)与航向(deg)，供目标位置外推
    Vector3f get_target_vel_ned_ms() const { return _target_vel_ned_ms; }
    float get_target_heading_deg() const { return _target_heading_deg; }

private:
    void handle_launch();
    void handle_target_info(const mavlink_command_int_t &packet);
    void update_target_timeout();
    void handle_interceptor_gps(const mavlink_command_int_t &packet);
    void fill_report_data();
    uint8_t relay_state_bitmap() const;
    bool gcs_ok() const;

    uint32_t last_gcs_msg_ms;

    // HUOJIAN: 31011 目标位置缓存与超时
    static const uint32_t TARGET_TIMEOUT_MS = 5000;
    Location _target_loc;
    Vector3f _target_vel_ned_ms;
    float _target_heading_deg;
    uint32_t _target_loc_ms;
    bool _target_loc_valid;
};
