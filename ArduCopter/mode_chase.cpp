#include "Copter.h"

#if MODE_GUIDED_ENABLED

// HUOJIAN: CHASE 任务容器模式——起飞/等待/飞行三态，起飞与航点飞行复用 GUIDED 控制器

bool ModeChase::init(bool ignore_checks)
{
    if (!copter.mode_guided.init(ignore_checks)) {
        return false;
    }
    last_fly_dest_ms = 0;
    takeoff_requested = false;
    chase_state = Chase_State::Standby;   // HUOJIAN: 先给确定值，避免与未初始化值比较
    // HUOJIAN: 进入模式 29 时做一次判定——
    // 地面/未解锁 -> 起飞态（解锁后自动起飞）；飞行中 -> 有目标 Fly，无目标 Standby
    if (!copter.motors->armed() || copter.ap.land_complete) {
        takeoff_requested = true;
        set_state(Chase_State::Takeoff);
    } else if (target_valid()) {
        set_state(Chase_State::Fly);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHASE: enter");
    return true;
}

void ModeChase::exit()
{
    last_fly_dest_ms = 0;
    takeoff_requested = false;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHASE: exit");
}

// HUOJIAN: 起飞请求（31010 或进入模式时的地面判定）；实际起飞在解锁后由 update_state 发起
void ModeChase::start_takeoff()
{
    takeoff_requested = true;
    set_state(Chase_State::Takeoff);
}

void ModeChase::set_state(Chase_State state_in)
{
    if (chase_state == state_in) {
        return;
    }

    switch (state_in) {
    case Chase_State::Takeoff:
        // HUOJIAN: 只切状态；实际起飞由 update_state 在解锁后发起（31010 不负责解锁）
        copter.mode_guided.hold_position();
        chase_state = state_in;
        break;

    case Chase_State::Standby:
        // HUOJIAN: 等待态用零速度/零加速度目标悬停
        copter.mode_guided.hold_position();
        chase_state = state_in;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHASE: standby");
        break;

    case Chase_State::Fly:
        // HUOJIAN: 先清速度目标，避免切飞行态瞬间的冲量
        copter.mode_guided.hold_position();
        last_fly_dest_ms = 0;
        chase_state = state_in;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHASE: fly");
        break;
    }
}

void ModeChase::update_state()
{
    switch (chase_state) {
    case Chase_State::Takeoff:
        // HUOJIAN: 未解锁只等待（安全要求：解锁由地面站 ARM 指令完成）
        if (!copter.motors->armed()) {
            break;
        }
        if (takeoff_requested) {
            // HUOJIAN: 请求先清除再发起，避免起飞完成后在 Takeoff/Standby 反复振荡
            takeoff_requested = false;
            if (copter.mode_guided.do_user_takeoff_start_m(copter.uattack.get_chase_takeoff_alt_m())) {
                copter.set_auto_armed(true);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHASE: takeoff");
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CHASE: takeoff fail");
            }
            break;
        }
        // HUOJIAN: GUIDED 起飞子模式结束即视为起飞完成，回等待态
        if (!copter.mode_guided.is_taking_off()) {
            set_state(Chase_State::Standby);
        }
        break;

    case Chase_State::Standby:
        // HUOJIAN: 解锁且目标有效就进入飞行态
        if (copter.motors->armed() && target_valid()) {
            set_state(Chase_State::Fly);
        }
        break;

    case Chase_State::Fly:
        // HUOJIAN: 目标丢失回等待态悬停
        if (!target_valid()) {
            set_state(Chase_State::Standby);
        }
        break;
    }
}

// HUOJIAN: 目标点按 1Hz 灌给 GUIDED 航点控制器
void ModeChase::fly_update()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (last_fly_dest_ms != 0 && now_ms - last_fly_dest_ms < 1000U) {
        return;
    }
    last_fly_dest_ms = now_ms;

    Location dest_loc = copter.uattack.get_target_loc();

    // HUOJIAN: 不按目标高度直接飞——在目标高度上叠加 UATK_CH_DH（默认 -20m，即比目标低 20m；正值表示比目标高），
    // 且不低于起飞点上方 UATK_CH_MIN(默认 40m)。
    // 31011 给的目标高度是相对 home 的，这里按 ABOVE_HOME 换算后再写回，保持与 GUIDED 航点的基准一致。
    float target_alt_m = 0.0f;
    if (dest_loc.get_alt_m(Location::AltFrame::ABOVE_HOME, target_alt_m)) {
        float chase_alt_m = target_alt_m + copter.uattack.get_chase_alt_dh_m();
        const float alt_min_m = copter.uattack.get_chase_alt_min_m();
        if (chase_alt_m < alt_min_m) {
            chase_alt_m = alt_min_m;
        }
        dest_loc.set_alt_m(chase_alt_m, Location::AltFrame::ABOVE_HOME);
    }

    copter.mode_guided.set_destination(dest_loc);
}

bool ModeChase::target_valid() const
{
    return copter.uattack.is_target_loc_valid();
}

float ModeChase::target_distance_m() const
{
    Location target_loc = copter.uattack.get_target_loc();
    return copter.current_loc.get_distance(target_loc);
}

void ModeChase::run()
{
    update_state();

    if (chase_state == Chase_State::Fly) {
        fly_update();
    }

    copter.mode_guided.run();

    // HUOJIAN: 进入 UATK_AUTO_DIST 以内且 UATK_AUTO=1 时自动切打击模式
    if (chase_state == Chase_State::Fly &&
        copter.uattack.get_auto_attack() &&
        target_valid() &&
        target_distance_m() < copter.uattack.get_auto_attack_dist_m()) {
        if (copter.set_mode(Mode::Number::ATTACK, ModeReason::GCS_COMMAND)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHASE: auto switch ATTACK");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CHASE: ATTACK switch failed");
        }
    }
}

// HUOJIAN: 地面站"与目标距离/方位"读数——GCS_MAVLink_Copter.cpp:219 用 flightmode->wp_distance_m() 组 NAV_CONTROLLER_OUTPUT，
// 基类 mode.h:177 默认 0；目标有效即报，与 UATK_AUTO_DIST 自动切门限同源
float ModeChase::wp_distance_m() const
{
    if (target_valid()) {
        return target_distance_m();
    }
    return 0.0f;
}

float ModeChase::wp_bearing_deg() const
{
    if (target_valid()) {
        const Location target_loc = copter.uattack.get_target_loc();
        return degrees(copter.current_loc.get_bearing(target_loc));
    }
    return 0.0f;
}

#endif  // MODE_GUIDED_ENABLED
