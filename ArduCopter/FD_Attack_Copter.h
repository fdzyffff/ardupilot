#pragma once

#include <AC_PID/AC_PID.h>
#include <FD_Attack/FD_Attack.h>

/*
 * FD_Attack_Copter —— Copter 端的"控制量合成"实现（FD_Attack 的派生类）。
 *
 * 基类算 LOS，本类算控制量：阶段机（PITCH_CAPTURE → ANGLE_CAPTURE → LOS_RATE）、
 * 两个 LOS 速率环 PID 与限幅、可见性限制、滞后补偿（有本机速度用直接法——光轴
 * 与速度方向夹角；否则回退半衰期积分法，LAG_BC 扣除补偿器自身贡献以打断
 * "指令→偏置→指令"正反馈）、控制侧日志（UAT5~UAT7）与打印、外部目标注入
 * （UChase 31011 / AP_Follow，经 set_target_loc 落地）。
 * 参数前缀 UATK_，本类序号 21 起（基类占 0~20）。
 */
class FD_Attack_Copter : public FD_Attack {
public:
    FD_Attack_Copter();

    static const struct AP_Param::GroupInfo var_info[];

    // 调度入口转发：SCHED_TASK_CLASS 的 Functor::bind 要求方法指针属于对象自身的类型，
    // 而 update()/do_print() 定义在基类
    void update() { FD_Attack::update(); }
    void do_print() { FD_Attack::do_print(); }

    // ---- 控制量输出（mode_attack / mode_chase / UChase 读取）----
    const Vector3f &get_target_rate_c_dps() const { return _target_rate_c_dps; }
    const Vector3f &get_target_rate_b_dps() const { return _target_rate_b_dps; }
    float get_throttle() const { return throttle.get(); }
    float get_throttle_rate() const { return throttle_rate.get(); }
    float get_ff_atk() const { return ff_atk.get(); }
    // CHASE 需要的目标点有效性与自动切打击模式参数
    bool get_auto_attack() const { return auto_attack.get() != 0; }
    float get_auto_attack_dist_m() const { return auto_attack_dist.get(); }
    float get_chase_takeoff_alt_m() const { return chase_takeoff_alt.get(); }
    // CHASE 飞行高度策略（拦截机与目标的相对高度差 / 相对起飞点最低高度）
    float get_chase_alt_dh_m() const { return chase_alt_dh.get(); }
    float get_chase_alt_min_m() const { return chase_alt_min.get(); }
    // 滞后补偿使能（锁架期由 ModeAttack 关闭；发射锁存后/退出模式时恢复）
    void set_lag_offset_enabled(bool en);

protected:
    // ---- 控制侧参数（序号 21 起）----
    AP_Float yaw_angle_gain;
    AP_Float pitch_angle_gain;
    AP_Float rate_limit_dps;
    AP_Float visibility_limit_deg;
    AC_PID los_yaw_rate_pid{3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f};
    AC_PID los_pitch_rate_pid{3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f};
    AP_Float throttle;
    AP_Float throttle_rate;
    AP_Float roll_level_gain;
    AP_Float forward_pitch_deg;
    AP_Int8 forward_pitch_en;
    AP_Float track_yaw_gain;
    AP_Float track_pitch_gain;
    AP_Float lag_pitch_half_life_s;
    AP_Float lag_pitch_max_deg;
    AP_Float lag_yaw_half_life_s;
    AP_Float lag_yaw_max_deg;
    AP_Int8  lag_bc_en;
    AP_Float ff_atk;
    AP_Int8  auto_attack;
    AP_Float auto_attack_dist;
    AP_Float chase_takeoff_alt;
    AP_Float chase_alt_dh;
    AP_Float chase_alt_min;
    AP_Int8 vel_use;

    // ---- 基类虚函数实现 ----
    void update_control() override;                        // 无参：输入取基类 _attitude_b_deg / _observation_dt_s
    void update_control_log() override;                    // UAT5~UAT7
    void do_print_control(int16_t print_mask) override;    // bit0 相机姿态 + bit3/4/5/7
    void update_external_targets() override;               // UChase 31011 / AP_Follow 注入
    void on_source_changed() override;                     // 源切换：速率环复位
    void on_target_lost() override;                        // 目标丢失：阶段 + 控制侧状态复位

    // ---- 控制侧内部实现 ----
    void apply_target_visibility_limit();
    // 滞后估计两条路径（有本机速度用直接法，否则回退积分法）
    void update_lag_offset_direct();
    void update_lag_offset_integral(float observation_dt_s);

    enum class ControlStage : uint8_t {
        NONE = 0,
        PITCH_CAPTURE = 1,
        ANGLE_CAPTURE = 2,
        LOS_RATE = 3
    };

    ControlStage _control_stage;
    Vector3f _camera_e_deg;        // 相机(地理系)姿态角，用于相机系滚转回平与打印
    Vector3f _target_rate_c_dps;   // 相机系目标角速率指令
    Vector3f _target_rate_b_dps;   // 机体系目标角速率指令（送姿态控制器）
    Vector3f _guid_rate_c_dps;     // 制导基础指令（不含滞后补偿分量），滞后估计器积分输入
    Vector2f _velocity_c_deg;      // 相机系速度方向角（未钳位：右/上为正）
    float _velocity_speed_ms;
    bool _velocity_valid;
    bool _enable_lag_offset = true;   // 滞后补偿估计使能
    float _lag_offset_pitch_deg;
    float _lag_offset_yaw_deg;
};
