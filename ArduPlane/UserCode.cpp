#include "Plane.h"

// USER_FORCE_SAFE: 判定飞机解锁且位于 fence 坐标 100km 内时强制 QLAND/RTL/DISARM
// 滞回: dist > 101km 复位触发; 100~101km 触发 RTL 一次;
//       95~100km 每秒强制 QLAND/RTL; <95km 强制上锁
#define USER_FORCE_SAFE_FENCE_LAT     401570981    // deg * 1e7
#define USER_FORCE_SAFE_FENCE_LNG     1164080429   // deg * 1e7
#define USER_FORCE_SAFE_RADIUS_M      100000.0f    // 100 km 强制半径
#define USER_FORCE_SAFE_REARM_M       101000.0f    // 101 km 复位半径
#define USER_FORCE_SAFE_DISARM_M       95000.0f    //  95 km 强制上锁半径

void Plane::check_forced_rtl_or_qland()
{
    if (!arming.is_armed()) {
        return;
    }

    Location cur = current_loc;
    if (cur.lat == 0 && cur.lng == 0) {
        // current_loc 无数据, 尝试 GPS 后备
        if (AP::gps().status() < AP_GPS::GPS_OK_FIX_2D) {
            return;
        }
        cur = AP::gps().location();
    }

    Location fence_loc(USER_FORCE_SAFE_FENCE_LAT,
                       USER_FORCE_SAFE_FENCE_LNG,
                       0,
                       Location::AltFrame::ABOVE_ORIGIN);
    const float dist = cur.get_distance(fence_loc);
    // 滞回状态机:
    //   dist > 101km        复位触发, 不动作
    //   100km < dist <=101  维持上次状态
    //   95km < dist <=100   触发并持续强制 QLAND/RTL
    //   dist <= 95km        强制上锁
    if (dist > USER_FORCE_SAFE_REARM_M) {
        force_safe_triggered = false;
    } else if (dist > USER_FORCE_SAFE_RADIUS_M) {
        if (!force_safe_triggered) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Force RTL for Safety Reason");
            set_mode(mode_rtl, ModeReason::USER_FORCE_SAFE);
            force_safe_triggered = true;
        }
    } else if (dist > USER_FORCE_SAFE_DISARM_M) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Force Land for Safety Reason");
        if (quadplane.available()) {
            set_mode(mode_qland, ModeReason::USER_FORCE_SAFE);
        } else {
            set_mode(mode_rtl, ModeReason::USER_FORCE_SAFE);
        }
    } else {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Force Disarm for Safety Reason");
        arming.disarm(AP_Arming::Method::TERMINATION);
    }
}
