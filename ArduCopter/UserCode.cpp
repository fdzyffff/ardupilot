#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    ufence.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
    ufence.update();
#if AP_SIM_ENABLED
    // case MSG_SIMSTATE:
    //     CHECK_PAYLOAD_SIZE(SIMSTATE);
    //     send_simstate();
    //     break;

    // case MSG_SIM_STATE:
    //     CHECK_PAYLOAD_SIZE(SIM_STATE);
    //     send_sim_state();
    //     break;
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
    // gcs().send_message(MSG_ZF8888_STATUS); //电子桩, F4
    gcs().send_message(MSG_ZF6666_STATUS); //飞控, H7
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
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

bool Copter::user_arm_switch_count() {
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
    float assit_roll = -bf_vel.y*100.f*kp;
    float assit_pitch = bf_vel.x*100.f*kp;
    float assit_max = 15.f*100.f;
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
    if (((flightmode->requires_GPS() && !position_ok()) || AP_Notify::flags.ekf_bad) && motors->armed()) {
        if (AP::ahrs().get_posvelyaw_source_set() == 0) {
            if (user_ekf_second_ok()) {
                AP::ahrs().set_posvelyaw_source_set(1);
                set_mode(Mode::Number::ALT_HOLD, ModeReason::GPS_GLITCH);
                gcs().send_text(MAV_SEVERITY_WARNING, "No GPS, ALT2");
                AP_Notify::flags.ekf_switch = 1;
            } else if (user_ekf_third_ok()) {
                AP::ahrs().set_posvelyaw_source_set(2);
                set_mode(Mode::Number::ALT_HOLD, ModeReason::GPS_GLITCH);
                gcs().send_text(MAV_SEVERITY_WARNING, "No GPS, ALT3");
                AP_Notify::flags.ekf_switch = 1;
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
