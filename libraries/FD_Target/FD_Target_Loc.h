#pragma once

#include "FD_Target.h"

#include <AP_Param/AP_Param.h>
#include <Filter/DerivativeFilter.h>

class FD_Target_Loc : public FD_Target_Base {
public:
    FD_Target_Loc();

    static const struct AP_Param::GroupInfo var_info[];

    bool init() override;
    void update() override;
    void handle_msg(const mavlink_message_t &msg) override;
    void handle_info_test(float p1, float p2);
    void set_target_loc(Location &loc_in);
    Location &get_target_loc() override;
    // 本源自算惯性系 LOS 角速率（地理系方位/高低 微分滤波），提供 handle_info_rate 通道
    bool has_rate_channel() const override { return true; }

    Location _current_loc;
    // HUOJIAN: 协议 31011 附带的目标速度与航向
    Vector3f _target_vel_ned_ms;
    float _target_heading_deg;

    AP_Int32 target_timeout;
    AP_Float target_distout;
    AP_Float nav_radius;
    AP_Int8 use_external_loc;

private:
    uint32_t _last_target_ms;
    uint32_t last_update_ms;
    // 自算惯性 LOS 角速率：先把方位展开成连续角（增量过 wrap_180），再交给微分滤波器
    // （DerivativeFilter：输入 角度+时间戳，slope() 直接得到角速率，与 FD_LPF 同一用法）；
    // 输出直接用，不再额外低通
    void update_los_rate(float az_deg, float el_deg, uint32_t now_ms);
    float _last_az_deg;      // 上次地理系方位角 deg（原始, 用于展开增量）
    float _accum_az_deg;     // 展开后的连续方位角 deg（送微分滤波）
    uint32_t _rate_last_ms;
    DerivativeFilterFloat_Size7 _deriv_az;    // 方位微分滤波 → 方位角速率
    DerivativeFilterFloat_Size7 _deriv_el;    // 高低微分滤波 → 高低角速率
};