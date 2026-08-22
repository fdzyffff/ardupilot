#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    ufence.init();
    uart.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    uart.update();

    static bool last_fs_loss = false;
    if (motors->get_thrust_boost() && !last_fs_loss) {
        if (flightmode->mode_number() != Mode::Number::LAND) {
            set_mode(Mode::Number::LAND, ModeReason::MOT_FAIL);
            last_fs_loss = true;
        }
    }
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
    ufence.update();
#if AP_SIM_ENABLED
    gcs().send_message(MSG_SIM_STATE);
#endif
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    user_gps_fail_check();
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    AP::fd_data().set_is_flying(copter.motors->armed() && (!ap.land_complete));
    AP::fd_data().update();

    AP::fd_data().set_uav_status(user_get_uav_status());

    // gcs().send_message(MSG_ZF8888_STATUS); //电子桩, F4
    gcs().send_message(MSG_ZF6666_STATUS); //飞控, H7

    check_forced_land_or_rtl();
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
    switch(ch_flag) {
    case RC_Channel::AuxSwitchPos::HIGH: {
        // engage mode (if not possible we remain in current flight mode)
        AP::fd_data().set_mot_fail(true);
        break;
    }
    default:
        AP::fd_data().set_mot_fail(false);
    }
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::fd_data_update()
{
    AP::fd_data().set_is_flying(motors->armed() && !ap.land_complete);
    AP::fd_data().update();

    uint8_t status = 2;
    if (ap.land_complete) {
        status = 1;
    } else if (motors->armed() &&
               ((flightmode->requires_position() && !position_ok()) || AP_Notify::flags.ekf_bad)) {
        status = 3;
    }
    AP::fd_data().set_uav_status(status);
}

void Copter::rid_update()
{
    uart.update();
}

uint8_t Copter::user_get_uav_status()
{
    uint8_t status = 0;
    if (ap.land_complete) {
        status = 1;
    } else if (((flightmode->requires_position() && !position_ok()) || AP_Notify::flags.ekf_bad) && motors->armed()) {
        status = 3;
    } else {
        status = 2;
    }
    return status;
}

bool Copter::user_arm_switch_count()
{
    static uint32_t last_ms = millis();
    static uint8_t last_count = 0;
    uint32_t now_ms = millis();
    if (now_ms - last_ms < 1500) {
        last_ms = now_ms;
        last_count++;
    } else {
        last_ms = now_ms;
        last_count = 0;
        last_count++;
    }
    if (last_count >= 3) {
        last_count = 0;
        return true;
    }
    return false;
}

void Copter::user_update_assit(float &target_roll, float &target_pitch)
{
    if (is_zero(g2.user_parameters.assit_gain.get())) {return;}
    if (!position_ok() || !motors->armed()) {
        return;
    }
    float kp = g2.user_parameters.assit_gain.get();
    Vector3f vec;
    if (!ahrs.get_velocity_NED(vec)) {
        return;
    }
    Vector2f bf_vel = ahrs.earth_to_body2D(vec.xy());
    // 4.7: target_roll/target_pitch are in radians; convert assist from cd to rad
    float assit_roll = radians(-bf_vel.y*100.f*kp*0.01f);
    float assit_pitch = radians(bf_vel.x*100.f*kp*0.01f);
    float assit_max = radians(15.0f);
    if (target_roll >= 0.0f && assit_roll > 0.0f) {
        target_roll = constrain_float(target_roll, assit_roll, assit_max);
    }
    if (target_roll <= 0.0f && assit_roll < 0.0f) {
        target_roll = constrain_float(target_roll, -assit_max, assit_roll);
    }
    if (target_pitch >= 0.0f && assit_pitch > 0.0f) {
        target_pitch = constrain_float(target_pitch, assit_pitch, assit_max);
    }
    if (target_pitch <= 0.0f && assit_pitch < 0.0f) {
        target_pitch = constrain_float(target_pitch, -assit_max, assit_pitch);
    }
}

void Copter::user_gps_fail_check()
{
    //EK3_SRC1：正常使用，GPS位置，GPS高度，GPS航向
    //EK3_SRC2：紧急使用，无位置，气压高度，无航向
    //EK3_SRC3：紧急使用，无位置，气压高度，无航向
    //切换条件：SRC1时，如位置失效，检查SRC2和SRC3设置，如果允许则切换至SRC2/3，否则直接LAND。切换后，都进行ALT_HOLD。
    if (((flightmode->requires_position() && !position_ok()) || AP_Notify::flags.ekf_bad) && motors->armed()) {
        if (AP::ahrs().get_posvelyaw_source_set() == 0) {
            if (user_ekf_second_ok()) {
                AP::ahrs().set_posvelyaw_source_set(AP_NavEKF_Source::SourceSetSelection::SECONDARY);
                set_mode(Mode::Number::ALT_HOLD, ModeReason::GPS_GLITCH);
                gcs().send_text(MAV_SEVERITY_WARNING, "No GPS, ALT2");
            } else if (user_ekf_third_ok()) {
                AP::ahrs().set_posvelyaw_source_set(AP_NavEKF_Source::SourceSetSelection::TERTIARY);
                set_mode(Mode::Number::ALT_HOLD, ModeReason::GPS_GLITCH);
                gcs().send_text(MAV_SEVERITY_WARNING, "No GPS, ALT3");
            } else {
                if (flightmode->mode_number() != Mode::Number::LAND) {
                    set_mode(Mode::Number::LAND, ModeReason::GPS_GLITCH);
                    gcs().send_text(MAV_SEVERITY_WARNING, "No GPS, Force LAND");
                }
            }
        }
    }
}

bool Copter::user_ekf_second_ok()
{
    float value_2_POSXY = 0.0f;
    float value_2_VELXY = 0.0f;
    float value_2_POSZ = 0.0f;
    float value_2_VELZ = 0.0f;
    float value_2_YAW = 0.0f;

    bool find_ekf_src2 = true;
    find_ekf_src2 = find_ekf_src2&&AP_Param::get("EK3_SRC2_POSXY", value_2_POSXY);
    find_ekf_src2 = find_ekf_src2&&AP_Param::get("EK3_SRC2_VELXY", value_2_VELXY);
    find_ekf_src2 = find_ekf_src2&&AP_Param::get("EK3_SRC2_POSZ", value_2_POSZ);
    find_ekf_src2 = find_ekf_src2&&AP_Param::get("EK3_SRC2_VELZ", value_2_VELZ);
    find_ekf_src2 = find_ekf_src2&&AP_Param::get("EK3_SRC2_YAW", value_2_YAW);

    if (find_ekf_src2) {
        if (((uint8_t)value_2_POSXY == 0)
            &&((uint8_t)value_2_VELXY == 0)
            &&((uint8_t)value_2_POSZ == 1)
            &&((uint8_t)value_2_VELZ == 0)
            &&((uint8_t)value_2_YAW == 0)
        ) {
            return true;
        }
    }
    return false;
}

bool Copter::user_ekf_third_ok()
{
    float value_3_POSXY = 0.0f;
    float value_3_VELXY = 0.0f;
    float value_3_POSZ = 0.0f;
    float value_3_VELZ = 0.0f;
    float value_3_YAW = 0.0f;

    bool find_ekf_src3 = true;
    find_ekf_src3 = find_ekf_src3&&AP_Param::get("EK3_SRC3_POSXY", value_3_POSXY);
    find_ekf_src3 = find_ekf_src3&&AP_Param::get("EK3_SRC3_VELXY", value_3_VELXY);
    find_ekf_src3 = find_ekf_src3&&AP_Param::get("EK3_SRC3_POSZ", value_3_POSZ);
    find_ekf_src3 = find_ekf_src3&&AP_Param::get("EK3_SRC3_VELZ", value_3_VELZ);
    find_ekf_src3 = find_ekf_src3&&AP_Param::get("EK3_SRC3_YAW", value_3_YAW);

    if (find_ekf_src3) {
        if (((uint8_t)value_3_POSXY == 0)
            &&((uint8_t)value_3_VELXY == 0)
            &&((uint8_t)value_3_POSZ == 1)
            &&((uint8_t)value_3_VELZ == 0)
            &&((uint8_t)value_3_YAW == 0)
        ) {
            return true;
        }
    }
    return false;
}

// USER_FORCE_SAFE: 判定飞机解锁且位于 fence 坐标 100km 内时强制 LAND/RTL/DISARM
// fence 中心: 40.1570981, 116.4080429 (北京)
// 滞回: dist > 101km 复位触发; 100~101km 触发 RTL 一次;
//       95~100km 每秒强制 LAND; <95km 强制上锁
#define USER_FORCE_SAFE_FENCE_LAT     401570981    // deg * 1e7
#define USER_FORCE_SAFE_FENCE_LNG     1164080429   // deg * 1e7
#define USER_FORCE_SAFE_RADIUS_M      100000.0f    // 100 km 强制半径
#define USER_FORCE_SAFE_REARM_M       101000.0f    // 101 km 复位半径
#define USER_FORCE_SAFE_DISARM_M       95000.0f    //  95 km 强制上锁半径

// USER_FORCE_SAFE: 滞回判定, 1Hz 由 userhook_SuperSlowLoop 调用
//   dist > 101km        复位触发标志, 不动作
//   100km < dist <=101  首次进入触发 RTL 一次
//   95km < dist <=100   每秒强制 LAND (不可切出)
//   dist <= 95km        强制上锁
void Copter::check_forced_land_or_rtl()
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

    if (dist > USER_FORCE_SAFE_REARM_M) {
        force_safe_triggered = false;
    } else if (dist > USER_FORCE_SAFE_RADIUS_M) {
        if (!force_safe_triggered) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Force Land for Safety Reason");
            set_mode(Mode::Number::RTL, ModeReason::USER_FORCE_SAFE);
            force_safe_triggered = true;
        }
    } else if (dist > USER_FORCE_SAFE_DISARM_M) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Force Land for Safety Reason");
        set_mode(Mode::Number::LAND, ModeReason::USER_FORCE_SAFE);
    } else {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Force Disarm for Safety Reason");
        arming.disarm(AP_Arming::Method::TERMINATION);
    }
}
