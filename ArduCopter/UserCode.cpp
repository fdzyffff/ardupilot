#include "Copter.h"

// USER_FORCE_SAFE: 强制安全降落, 1Hz 由 userhook_SuperSlowLoop 调用
// 两层结构:
//   第一层 北京敏感区:  >101km 复位触发标志; 100~101km 首次进入触发一次 RTL;
//                       95~100km 每秒强制 LAND; <=95km 强制上锁
//   第二层 21 个允许区域: 必须位于任一允许圆内, 出圈 0~5km 每秒强制 RTL, 出圈 >5km 直接上锁
// 敏感区优先: 三个敏感区分支均 return, 不再进入白名单判定
#define USER_FORCE_SAFE_FENCE_LAT  401570981     // deg * 1e7
#define USER_FORCE_SAFE_FENCE_LNG  1164080429    // deg * 1e7
#define USER_FORCE_SAFE_RADIUS_M   100000.0f     // 100 km: 强制 LAND
#define USER_FORCE_SAFE_REARM_M    101000.0f     // 101 km: 复位触发标志
#define USER_FORCE_SAFE_DISARM_M    95000.0f     //  95 km: 强制上锁
#define USER_FORCE_SAFE_OUTSIDE_DISARM_M 5000.0f // 出允许圆 5 km: 直接上锁

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    uart.init();
    uattack.init();
    yolo_drop.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    uart.update();
    uattack.update();
    yolo_drop.update();
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    gcs().send_message(MSG_HXTS_BAT_CAN_STATUS);
    gcs().send_message(MSG_HXTS_CAN_MOT_INFO);
    gcs().send_message(MSG_HXTS_HY_BMS_CELL);
    gcs().send_message(MSG_HXTS_HY_BMS_TEMP);
    gcs().send_message(MSG_HXTS_HY_BMS_STRV);
    gcs().send_message(MSG_HXTS_HY_BMS_STRV2);
    AP::fd_data().set_is_flying(copter.motors->armed() && (!ap.land_complete));
    AP::fd_data().update();

    if ((uattack.print.get() & (1<<0)) && uattack.display_info.new_data) {
        gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f , %0.0f , %0.0f , %0.0f",
            uattack.display_info.count_log, uattack.display_info.p1,
            uattack.display_info.p2, uattack.display_info.p3, uattack.display_info.p4);
        uattack.display_info.new_data = false;
    }
    if (uattack.print.get() & (1<<1)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "ef_angle (%0.2f , %0.2f) on:%d",
            uattack.get_ef_info().x, uattack.get_ef_info().y, uattack.is_active());
    }
    if (uattack.print.get() & (1<<2)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "ef_rate (%0.2f , %0.2f) on:%d",
            uattack.get_ef_rate_info().x, uattack.get_ef_rate_info().y, uattack.is_active());
    }
    if (uattack.print.get() & (1<<6)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "%0.0f , %0.0f , %0.0f , %0.0f",
            uattack.display_info.p11, uattack.display_info.p12,
            uattack.display_info.p13, uattack.display_info.p14);
    }
    yolo_drop.print_debug();

    check_forced_land_or_rtl();
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case RC_Channel::AuxSwitchPos::LOW:
        AP::fd_data().do_switch(0);
        break;
    case RC_Channel::AuxSwitchPos::MIDDLE:
        break;
    case RC_Channel::AuxSwitchPos::HIGH:
        AP::fd_data().do_switch(1);
        break;
    }
}
#endif

struct UserForceSafeArea {
    int32_t lat;
    int32_t lng;
    float radius_m;
};

static const UserForceSafeArea user_force_safe_areas[] = {
    { 322500000,  830000000, 490000.0f }, //  1 西藏阿里
    { 455000000, 1300000000, 480000.0f }, //  2 黑龙江东部
    { 222500000, 1112500000, 530000.0f }, //  3 粤西、广西沿海
    { 385000000, 1045000000, 510000.0f }, //  4 宁夏、甘肃中部
    { 452500000,  850000000, 480000.0f }, //  5 北疆
    { 282500000,  995000000, 500000.0f }, //  6 川西、滇西北
    { 337500000, 1177500000, 510000.0f }, //  7 苏皖、黄淮地区
    { 445000000, 1142500000, 480000.0f }, //  8 内蒙古中部
    { 357500000,  950000000, 480000.0f }, //  9 青海南部
    { 310000000, 1092500000, 520000.0f }, // 10 渝鄂、三峡库区
    { 387500000,  785000000, 480000.0f }, // 11 南疆西部
    { 280000000, 1172500000, 520000.0f }, // 12 赣闽、浙南
    { 307500000,  900000000, 490000.0f }, // 13 西藏中部
    { 245000000, 1037500000, 530000.0f }, // 14 滇中、滇南
    { 412500000, 1217500000, 480000.0f }, // 15 辽宁地区
    { 327500000, 1000000000, 490000.0f }, // 16 青东南、川北
    { 392500000, 1120000000, 510000.0f }, // 17 山西、陕北
    { 425000000,  955000000, 500000.0f }, // 18 河西走廊
    { 495000000, 1220000000, 470000.0f }, // 19 内蒙古东北、黑龙江西部
    { 385000000,  870000000, 470000.0f }, // 20 新疆中部
    { -353627273, 1491651642, 100000.0f }, // 21 澳大利亚 SITL
};

void Copter::check_forced_land_or_rtl()
{
    if (!arming.is_armed()) {
        return;
    }

    Location cur = current_loc;
    if (cur.lat == 0 && cur.lng == 0) {
        // current_loc 无数据, 尝试 primary GPS 后备
        if (AP::gps().status() < AP_GPS::GPS_OK_FIX_2D) {
            return;
        }
        cur = AP::gps().location();
    }

    // ---------------- 第一层: 北京敏感区 ----------------
    const Location fence_loc(USER_FORCE_SAFE_FENCE_LAT,
                             USER_FORCE_SAFE_FENCE_LNG,
                             0,
                             Location::AltFrame::ABOVE_ORIGIN);
    const float fence_dist_m = cur.get_distance(fence_loc);

    if (fence_dist_m > USER_FORCE_SAFE_REARM_M) {
        force_safe_triggered = false;
    } else if (fence_dist_m > USER_FORCE_SAFE_RADIUS_M) {
        if (!force_safe_triggered) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Force RTL for Safety Reason");
            set_mode(Mode::Number::RTL, ModeReason::USER_FORCE_SAFE);
            force_safe_triggered = true;
        }
        return;
    } else if (fence_dist_m > USER_FORCE_SAFE_DISARM_M) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Force Land for Safety Reason");
        set_mode(Mode::Number::LAND, ModeReason::USER_FORCE_SAFE);
        return;
    } else {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Force Disarm for Safety Reason");
        arming.disarm(AP_Arming::Method::TERMINATION);
        return;
    }

    // ---------------- 第二层: 21 个允许区域白名单 ----------------
    //   位于任一允许圆内(含边界) -> 放行
    //   全部圆之外: 最小出圈距离 0~5000m -> 每秒强制 RTL ; >5000m -> 直接上锁
    float minimum_outside_dist_m = 0.0f;
    bool minimum_outside_dist_valid = false;

    for (uint8_t i = 0; i < ARRAY_SIZE(user_force_safe_areas); i++) {
        const UserForceSafeArea &area = user_force_safe_areas[i];
        Location area_center = cur;
        area_center.lat = area.lat;
        area_center.lng = area.lng;
        const float outside_dist_m = cur.get_distance(area_center) - area.radius_m;

        if (outside_dist_m <= 0.0f) {
            // 位于允许区域内
            return;
        }
        if (!minimum_outside_dist_valid || outside_dist_m < minimum_outside_dist_m) {
            minimum_outside_dist_m = outside_dist_m;
            minimum_outside_dist_valid = true;
        }
    }

    if (minimum_outside_dist_m <= USER_FORCE_SAFE_OUTSIDE_DISARM_M) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Force RTL Outside Allowed Area");
        set_mode(Mode::Number::RTL, ModeReason::USER_FORCE_SAFE);
    } else {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Force Disarm Outside Allowed Area");
        arming.disarm(AP_Arming::Method::TERMINATION);
    }
}
