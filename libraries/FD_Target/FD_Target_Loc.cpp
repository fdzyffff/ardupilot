#include "FD_Target_Loc.h"

#include <AP_AHRS/AP_AHRS.h>

const AP_Param::GroupInfo FD_Target_Loc::var_info[] = {
    AP_GROUPINFO("TOUT", 0, FD_Target_Loc, target_timeout, 5000),
    AP_GROUPINFO("DOUT", 1, FD_Target_Loc, target_distout, 30),
    // HUOJIAN: 拦截半径——目标超出该距离视为无效(默认 8000m)
    AP_GROUPINFO("NRAD", 2, FD_Target_Loc, nav_radius, 8000),
    AP_GROUPINFO("ELOC", 3, FD_Target_Loc, use_external_loc, 0),
    AP_GROUPEND
};

FD_Target_Loc::FD_Target_Loc() :
    _last_target_ms(0),
    last_update_ms(0),
    _last_az_deg(0.0f),
    _accum_az_deg(0.0f),
    _rate_last_ms(0)
{
    // HUOJIAN: 协议附带的目标速度/航向初值
    _target_vel_ned_ms.zero();
    _target_heading_deg = 0.0f;
    AP_Param::setup_object_defaults(this, var_info);
}

bool FD_Target_Loc::init()
{
    _valid = false;
    set_type(2);
    _deriv_az.reset();
    _deriv_el.reset();
    _last_az_deg = 0.0f;
    _accum_az_deg = 0.0f;
    _rate_last_ms = 0;
    return true;
}

void FD_Target_Loc::update()
{
    bool have_position = AP::ahrs().get_location(_current_loc);
    if (!have_position) {
        _valid = false;
        return;
    }

    if (_valid && (AP_HAL::millis() - _last_ms) > 16) {
        Vector3f off_ef = _current_loc.get_distance_NED(_target_loc);

        off_ef.normalized();
        Matrix3f tmp_earth_yaw_m;
        tmp_earth_yaw_m.from_euler(radians(0.0f), radians(0.0f), AP::ahrs().get_yaw_rad());
        tmp_earth_yaw_m.transpose();
        Vector3f off_eyf = tmp_earth_yaw_m * off_ef;
        off_eyf.normalized();

        float p1 = degrees(wrap_180(atan2f(off_eyf.y, off_eyf.x)));
        float p2 = degrees(wrap_180(atan2f(-off_eyf.z, off_eyf.xy().length())));
        handle_info(p1, p2);
        // 本源自算速率用【地理系】方位/高低（不是上面喂角度的 yaw 系角）：发布的 r1 才是
        // 惯性方位角速率，与导引头自带的 LOS rate 同口径；off_ef 此时已是单位向量
        const float az_e_deg = degrees(wrap_180(atan2f(off_ef.y, off_ef.x)));
        const float el_e_deg = degrees(wrap_180(atan2f(-off_ef.z, off_ef.xy().length())));
        update_los_rate(az_e_deg, el_e_deg, AP_HAL::millis());
    }

    uint32_t now_ms = AP_HAL::millis();
    if ((target_timeout.get() > 0) &&
        (now_ms - _last_target_ms > (uint32_t)target_timeout.get())) {
        _valid = false;
    }

    if (_valid && AP::ahrs().get_location(_current_loc) &&
        (_current_loc.get_distance(_target_loc) < target_distout.get())) {
        _valid = false;
    }

    // HUOJIAN: 超出拦截半径(默认 8km)的目标不接受——该距离外雷达给的多半是错点
    if (_valid && (nav_radius.get() > 0) &&
        (_current_loc.get_distance(_target_loc) > nav_radius.get())) {
        _valid = false;
    }

    if (now_ms - last_update_ms > 1000) {
        last_update_ms = now_ms;
    }
}

void FD_Target_Loc::handle_msg(const mavlink_message_t &msg)
{
    // HUOJIAN: 目标信息按协议改为 31011(MAV_CMD_USER_2)：
    // param5/6=目标经纬度(1e-7deg)、param7=目标相对 home 高度(m)、param1/2=北/东向速度、param3=航向
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
        mavlink_command_int_t packet;
        mavlink_msg_command_int_decode(&msg, &packet);
        if (packet.command == MAV_CMD_USER_2) {
            Location temp_loc;
            temp_loc.lat = packet.x;
            temp_loc.lng = packet.y;
            // HUOJIAN: 协议 z 是相对 home 高度，全局目标统一存海拔(MSL)，需要相对高度时由消费侧换算
            int32_t base_alt_cm = 0;
            if (AP::ahrs().home_is_set()) {
                base_alt_cm = AP::ahrs().get_home().alt;
            } else {
                Location origin;
                if (AP::ahrs().get_origin(origin)) {
                    base_alt_cm = origin.alt;
                }
            }
            temp_loc.set_alt_cm((int32_t)(packet.z * 100.0f) + base_alt_cm, Location::AltFrame::ABSOLUTE);
            _target_vel_ned_ms.x = packet.param1;
            _target_vel_ned_ms.y = packet.param2;
            _target_heading_deg = packet.param3;
            set_target_loc(temp_loc);
        }
    }
}

void FD_Target_Loc::set_target_loc(Location &loc_in)
{
    _target_loc = loc_in;
    _valid = true;
    _last_target_ms = AP_HAL::millis();
}

Location &FD_Target_Loc::get_target_loc()
{
    return _target_loc;
}

void FD_Target_Loc::handle_info_test(float p1, float p2)
{
}

// 本源自算惯性 LOS 角速率：对【地理系】方位 az(右+)/高低 el(上+) 做微分滤波。
// 用 ArduPilot 的 DerivativeFilter（输入 角度+时间戳，slope() 直接给角速率，×1000 → deg/s，
// 与 FD_LPF 链里 _derivative_x.update(...) / slope()*1000.0f 同一用法）。
// 方位会跨 ±180°：先把增量 wrap_180 后累积成连续角再进滤波器，否则跨 ±180° 处会出假尖峰。
// 输出直接发布，不再额外低通。
void FD_Target_Loc::update_los_rate(float az_deg, float el_deg, uint32_t now_ms)
{
    _accum_az_deg += wrap_180(az_deg - _last_az_deg);
    _last_az_deg = az_deg;

    const bool first = (_rate_last_ms == 0);
    const uint32_t dt_ms = now_ms - _rate_last_ms;
    const bool dt_ok = !first && (dt_ms > 1U) && (dt_ms < 500U);

    // 微分滤波：每帧都喂（含首帧），滤波器内部按时间戳算平滑斜率
    _deriv_az.update(_accum_az_deg, now_ms);
    _deriv_el.update(el_deg, now_ms);

    if (!dt_ok) {
        // 首帧或数据间断：清滤波器，避免跨间断的假斜率（间断后的前几帧 slope 自然为 0）
        if (!first) {
            _deriv_az.reset();
            _deriv_el.reset();
        }
        _rate_last_ms = now_ms;
        return;
    }

    const float r1 = _deriv_az.slope() * 1000.0f;   // 方位角速率（右+，惯性系，deg/s）
    const float r2 = _deriv_el.slope() * 1000.0f;   // 高低角速率（上+，惯性系，deg/s）
    handle_info_rate(r1, r2);
    _rate_last_ms = now_ms;
}
