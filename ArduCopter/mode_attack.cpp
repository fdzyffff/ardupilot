#include "Copter.h"

bool ModeAttack::init(bool ignore_checks)
{
    if (!copter.uattack.is_active()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Attack: target unavailable");
        return false;
    }

    throttle_out = constrain_float(copter.motors->get_throttle_hover(), 0.0f, 1.0f);
    target_lost_start_ms = 0;

    // 发射架保护：进入模式时飞机被锁定在发射架上（约10s），期间只能走
    // 纯速率路径（_2_rads）；清零姿态控制器状态，并记录进入高度作为离架判据基准。
    // 与 Plane QACRO 的 _enter() 同款做法（relax_attitude_controllers）。
    attitude_control->relax_attitude_controllers();
    rate_locking_active = false;
    entry_alt_cm = pos_control->get_pos_estimate_U_m()*100.f;
    launch_move_start_ms = 0;
    // 锁架期指令未被执行，先关闭 UAttack 滞后补偿（防幽灵滞后累积），
    // 离架锁存时再打开（见 check_launched）
    copter.uattack.set_lag_offset_enabled(false);
    // 平时档：pitch 速率环 FF 恒 0（ATC_RAT_PIT_FF 须保持 0），离架确认后才注入 UATK_FF_ATK；
    // 无条件写 0 保证状态自愈，即使上次异常退出有残留也被清掉
    attitude_control->get_rate_pitch_pid().set_ff(0.0f);
    return true;
}

void ModeAttack::exit()
{
    // 退出 ATTACK（含锁架期目标丢失 RTL）恢复滞后补偿默认使能
    copter.uattack.set_lag_offset_enabled(true);
    // FF 必须归零，否则残留前馈会在其他模式/下一次飞行中直接出力炸机
    attitude_control->get_rate_pitch_pid().set_ff(0.0f);
}

// 离架检测。判据用气压计+IMU 融合的高度/升降速率（锁架时≈0，可靠；不依赖水平速度——
// 无 GPS 时水平速度会漂移）。两条判据任一满足即视为离架，一次性锁存不切回：
//   高度相对进模式时变化>10m：积分量，抗气压噪声，即时判定无需持续确认；
//     前提假设：锁架期间 EKF 高度漂移须远小于10m，否则会误判（一次锁存无回退）。
//   或升降速率>2m/s 且持续 1.5s：快速离架通道，持续确认防瞬时噪声。
bool ModeAttack::check_launched()
{
    if (rate_locking_active) {
        return true;
    }
    const float alt_change_cm = fabsf(pos_control->get_pos_estimate_U_m()*100.f - entry_alt_cm);
    const float climb_cms = fabsf(pos_control->get_vel_estimate_U_ms()*100.f);
    const bool moving_1 = (alt_change_cm > 1000.0f);
    const bool moving_2 = (climb_cms > 200.0f);
    if (moving_1) {
        rate_locking_active = true;
        // 切换瞬间清零锁架期间速率环可能积下的 I 项，防止脱锁冲击
        attitude_control->relax_attitude_controllers();
        // 离架确认：重新允许 UAttack 滞后补偿（锁架期已禁用，offset 从 0 起步）
        copter.uattack.set_lag_offset_enabled(true);
        // 离架确认：注入拦截档 pitch FF（与滞后补偿同一锁存沿）
        attitude_control->get_rate_pitch_pid().set_ff(copter.uattack.get_ff_atk());
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Attack: launched, rate mode -> 3_rads");
    }

    if (moving_2) {
        if (launch_move_start_ms == 0) {
            launch_move_start_ms = AP_HAL::millis();
        } else if (AP_HAL::millis() - launch_move_start_ms > 1500U) {
            rate_locking_active = true;
            // 切换瞬间清零锁架期间速率环可能积下的 I 项，防止脱锁冲击
            attitude_control->relax_attitude_controllers();
            // 离架确认：重新允许 UAttack 滞后补偿（锁架期已禁用，offset 从 0 起步）
            copter.uattack.set_lag_offset_enabled(true);
            // 离架确认：注入拦截档 pitch FF（与滞后补偿同一锁存沿）
            attitude_control->get_rate_pitch_pid().set_ff(copter.uattack.get_ff_atk());
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Attack: launched, rate mode -> 3_rads");
        }
    } else {
        launch_move_start_ms = 0;
    }
    return rate_locking_active;
}

void ModeAttack::run()
{
    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        attitude_control->reset_target_and_rate(true);
        attitude_control->reset_rate_controller_I_terms();
        throttle_out = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        attitude_control->reset_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        throttle_out = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        break;
    }

    Vector3f target_rate_b_dps;
    if (copter.uattack.is_active()) {
        target_rate_b_dps = copter.uattack.get_target_rate_b_dps();
        target_lost_start_ms = 0;
    } else {
        target_rate_b_dps.zero();

        const uint32_t now_ms = AP_HAL::millis();
        if (target_lost_start_ms == 0) {
            target_lost_start_ms = now_ms;
        } else if (now_ms - target_lost_start_ms >= 2000U) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Attack: target lost, RTL");
            if (!copter.set_mode(Mode::Number::RTL, ModeReason::FAILSAFE)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Attack: RTL failed, LAND");
                copter.set_mode(Mode::Number::LAND, ModeReason::FAILSAFE);
            }
            return;
        }
    }

    // 离架前（锁架约10s）走纯速率路径 _2_rads，不积分姿态误差；
    // check_launched() 确认离架后一次性切换到 _3_rads：把速率环没跟上的
    // 转角积分成姿态误差并补回来，抑制高速气动扰动造成的姿态丢失
    const Vector3f target_rate_b_rads(radians(target_rate_b_dps.x),
                                      radians(target_rate_b_dps.y),
                                      radians(target_rate_b_dps.z));
    if (check_launched()) {
        attitude_control->input_rate_bf_roll_pitch_yaw_3_rads(target_rate_b_rads.x,
                                                              target_rate_b_rads.y,
                                                              target_rate_b_rads.z);
    } else {
        attitude_control->input_rate_bf_roll_pitch_yaw_2_rads(target_rate_b_rads.x,
                                                              target_rate_b_rads.y,
                                                              target_rate_b_rads.z);
    }

    const float throttle_target = constrain_float(copter.uattack.get_throttle(), 0.0f, 1.0f);
    const float throttle_delta_max = MAX(copter.uattack.get_throttle_rate(), 0.0f) * G_Dt;
    throttle_out += constrain_float(throttle_target - throttle_out,
                                    -throttle_delta_max,
                                    throttle_delta_max);
    throttle_out = constrain_float(throttle_out, 0.0f, 1.0f);

    attitude_control->set_throttle_out(throttle_out, false, copter.g.throttle_filt);
}