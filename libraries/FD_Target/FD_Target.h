#pragma once

#include <AP_Common/Location.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

class FD_Target_Base {
public:
    FD_Target_Base();
    virtual ~FD_Target_Base() {}

    virtual bool init() { return false; }
    virtual void update() = 0;
    virtual void handle_msg(const mavlink_message_t &msg) {}

    bool is_valid() const { return _valid; }
    void handle_info(float p1, float p2);
    bool get_info(float &p1, float &p2);
    // 目标源自带 LOS 角速率：惯性系（地理系）下的 LOS 角速率两分量
    // r1=方位角速率 右+ / r2=高低角速率 上+，与 FD_LPF 链"由 LOS 角微分算出的速率"同一种量。
    // 它与 p1/p2 只是符号约定一致、参考系不同（源的角度相对自身 0 位，速率是惯性量），
    // FD_Attack 侧会在地理系里组向量后再旋到相机系。
    void handle_info_rate(float r1, float r2);
    bool get_info_rate(float &r1, float &r2);   // 每个新速率帧取一次
    bool has_info_rate() const { return _has_rate; }
    void recover_info();
    uint8_t get_type();
    void set_type(uint8_t type_in);
    void set_valid(bool valid_in);
    // 帧率：基类统计的是 handle_info() 收到的观测帧率（_valid 为假时返回 0）。
    // 派生类若自己能统计（如 DYT2：串口链路在线但未锁定时也要能报在线），覆写本函数。
    virtual float get_fps() const { return _valid ? _fps : 0.0f; }
    // 本类是否提供 LOS 角速率通道（handle_info_rate 有没有数据源）。
    // FD_Attack 在 LOS_TYPE=2 下用它判断"该源根本没有速率"→ 一次性告警，而不是静默回退。
    virtual bool has_rate_channel() const { return false; }
    virtual Location &get_target_loc() { return _target_loc; }

protected:
    uint32_t _last_ms;
    bool _new_data;
    bool _valid;
    float _p1;
    float _p2;
    bool _has_rate;      // 目标源是否提供 LOS 角速率
    bool _new_rate;      // 有新速率帧待消费
    float _r1;           // 方位角速率 右+
    float _r2;           // 高低角速率 上+
    float _fps;
    uint32_t _fps_last_ms;
    uint8_t _type;
    Location _target_loc;
};
