#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <Filter/Filter.h>
#include <FD_Target/FD_Target_Cam.h>
#include <FD_Target/FD_Target_HY.h>
#include <FD_Target/FD_Target_DYT2.h>
#include <FD_Target/FD_Target_Loc.h>

#include <FD_Attack/UDelay.h>
#include <FD_Attack/UCamDelayEstimator.h>
#include <FD_Attack/FD_LPF.h>
#include <FD_Attack/FD_EKF.h>

/*
 * FD_Attack —— 拦截/攻击链的通用基类：只做"算 LOS"。
 *
 * 目标源管理（相机优先、位置兜底）+ 延迟姿态/陀螺取数（UDelay 按时间戳插值）
 * + 两条并行 LOS 估计链：FD_LPF"差分+低通"（默认）与 FD_EKF 7 状态联合估计
 * （Δ 进状态空间），按 UATK_LOS_TYPE 选一套作为最终量：
 *   0=LPF 链（默认） 1=EKF 链（还需 UATK_EKF_EN≠0 且两通道已初始化） 2=目标源自带 LOS rate
 * UATK_EKF_EN 只留 0/1（是否运行 EKF 链），"是否用于制导"由 LOS_TYPE 决定；
 * LOS_TYPE=2 时角度仍取 LPF 链（不做微分），只有角速度换成目标源直接给的那一份，
 * 折算到相机系口径与两条链一致（du_c/dt + ω_cam × u_c）；源速率丢失时先保持，
 * 超过 TOUT 回退 LPF（见 update_source_rate()）。
 * UATK_LPF_EN=0 时，只要 LPF 不是被选中的那一套（LOS_TYPE≠0）仍强制保留，
 * 作为兜底与 UAT3 的对照量（见 lpf_enabled()）。
 * 源变化（含 →NONE）时两链统一复位到初始态，Δ 回到预设值重新在线收敛——
 * LOC→CAM 必须复位：LOC 量测没有相机管线延迟，沿用旧 Δ 等于没有延迟补偿。
 *
 * 控制量合成由端侧派生类实现：纯虚 update_control() + 五个默认空实现的钩子。
 * 参数前缀 UATK_：本库序号 0~20，派生类 21 起；LPF/EKF 各自成组，
 * 组内首条带 AP_PARAM_FLAG_ENABLE（为 0 时组内其余参数不在列表显示）。
 */
class FD_Attack {
public:
    FD_Attack();

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();
    void do_print();
    void handle_attack_msg(const mavlink_message_t &msg);

    bool is_active() const { return _target_source != TargetSource::NONE; }
    bool is_active_loc() const { return _target_source == TargetSource::LOC; }
    bool is_active_cam() const { return _target_source == TargetSource::CAM; }

    const Vector2f &get_target_c_deg() const { return _target_c_deg; }
    const Vector2f &get_los_e_deg() const { return _los_e_deg; }
    // 最终相机系 LOS rate（按 UATK_LOS_TYPE 选中的那一条；控制律用的就是它）
    const Vector3f &get_los_c_rate() const { return _los_c_rate; }
    // CHASE 需要目标点有效性与自动切打击模式参数
    bool is_target_loc_valid() const { return (target_loc != nullptr) && target_loc->is_valid(); }
    // 导引头(相机)通信是否在线，用于协议 hb_ok 上报
    bool is_cam_online() const { return target_cam_initialized && (target_cam->get_fps() > 0.0f); }
    Location get_target_loc();
    void set_target_loc(Location &loc_in);

protected:
    enum class TargetSource : uint8_t {
        NONE = 0,
        LOC = 1,
        CAM = 2
    };

    void init_targets();
    void update_delay_history();
    bool get_delayed_state(Vector3f &attitude_b_deg, Vector3f &gyro_b_dps) const;

    // 控制量合成：纯虚、无参——由端侧派生类实现。它需要的量（延迟后姿态
    // _attitude_b_deg、观测帧间隔 _observation_dt_s、本周期是否有观测
    // _have_observation）都由基类在 update_los() 前备好，通过继承取用。
    virtual void update_control() = 0;
    // 五个钩子默认空实现，派生类按需覆盖（把控制侧内容留在端侧）
    virtual void update_control_log() {}                       // 控制侧日志（UAT5~UAT7）
    virtual void do_print_control(int16_t print_mask) { (void)print_mask; }  // 控制侧打印
    virtual void update_external_targets() {}                  // 外部目标注入（经 set_target_loc）
    virtual void on_source_changed() {}                        // 源切换时控制侧复位（速率环）
    virtual void on_target_lost() {}                           // 目标丢失时控制侧复位（阶段/速率环）

    // update() 的子步骤（定义统一放在 FD_Attack.cpp 后半，便于主流程一眼看结构）
    void update_runtime_params();   // 延迟历史 + 1Hz 滤波系数刷新 + 每周期 Δ/Q/R 刷新
    void update_targets();          // 目标源 update（含外部注入）+ 源判定与软重置
    void update_los();              // get_delayed_state() + update_lpf() + update_ekf() + 选择
    void update_lpf(uint32_t now_ms);   // "差分+低通"链（含在线 Δ 估计）→ _los_*_lpf / _dly_lpf
    void update_ekf(uint32_t now_ms);   // 7 状态联合 EKF（含每周期预测步）→ _los_*_ekf / _dly_ekf
    // 目标源自带 LOS rate（LOS_TYPE=2）→ _los_c_rate_src / _los_e_rate_src（含超时回退判定）
    void update_source_rate(uint32_t now_ms);
    // 源速率 → 惯性系 LOS 角速度（NED 分量, deg/s）；与 FD_LPF 由 LOS 角微分的构造完全一致
    Vector3f source_rate_earth(float r1, float r2) const;
    // 当前（未延迟）机体姿态 → NED 旋转矩阵（源速率折算用它，不用 LPF 链的延迟姿态）
    Matrix3f rotation_ned_from_body_current() const;
    void clear_target_output();
    // 两条估计链统一复位到初始状态。调用点唯一——update_targets() 的"源发生变化"
    // 分支（每次源变化各一次，含 X→NONE）。详见 FD_Attack.cpp 函数注释。
    void reset_chains();
    bool lpf_enabled() const;       // LPF 链是否运行（UATK_LPF_EN，或 EKF 未接管时强制启用）
    void update_log();

    // 联合 EKF 量测更新：送【原始相机离轴角 b】（未去旋转），去旋转在滤波器内部
    // 用状态 Δ̂ 完成；gyro_b_dps 仅用于机动门控（冻结 Δ 通道增益）
    void ekf_measure(const Vector2f &target_c_deg, uint32_t t_arr_ms, const Vector3f &gyro_b_dps);
    bool ekf_for_control() const;
    // UAH1/UAH2 的统一写入点（update_log() 调：hi_log=0 时 10Hz，=1 时每次 update）
    void write_uah_msgs();

    AP_Int8 use_target_cam;
    AP_Int8 use_target_loc;
    FD_LPF lpf;                  // UATK_LPF_* 参数组 + "差分+低通"链实现（同一个实例）
    FD_EKF ekf;                  // UATK_EKF_* 参数组 + 延迟补偿 EKF 实现（同一个实例）
    // UATK_LOS_TYPE 制导用的 LOS 角速率来源: 0=LPF 1=EKF 2=目标源自带（默认 0）
    AP_Int8 los_type;
    AP_Int16 debug_print;
    AP_Int8  hi_log;             // HILOG：1=每次 update 记高率日志 UAH1/UAH2（标定用，正常飞行关闭）

    // 相机类目标源：常驻实例 + 指针（UATK_TCAM_USE 选一个挂到 target_cam）
    FD_Target_HY target_HY;
    FD_Target_DYT2 target_DYT2;
    FD_Target_Loc target_loc_inst;
    FD_Target_Cam *target_cam;
    FD_Target_Loc *target_loc;
    bool target_cam_initialized;
    bool target_loc_initialized;
    TargetSource _target_source;

    UDelay attitude_delay;
    UDelay gyro_delay;

    // 相机管线延迟在线估计器（只服务 LPF 链；UATK_LPF_EST=1 时接管 _dly_lpf）
    UCamDelayEstimator _dly_est;

    Matrix3f rotation_body_to_camera;
    Matrix3f rotation_camera_to_body;
    Vector2f _target_c_deg;
    Vector2f _los_e_deg;          // 最终惯性系 LOS 角（两条链择一）
    Vector2f _los_e_deg_lpf;      // LPF 链的惯性系 LOS 角
    Vector2f _los_e_deg_ekf;      // EKF 链的惯性系 LOS 角
    Vector3f _los_e_rate;         // 最终惯性系 LOS 角速度（两条链择一）
    Vector3f _los_c_rate;         // 最终相机系 LOS 角速度（两条链择一，控制律用的是它）
    bool _have_observation;       // 本周期是否收到新的观测帧
    bool _lpf_en_warned;          // UATK_LPF_EN=0 且 LPF 被强制启用时只警告一次

    // 控制量合成所需的输入（观测帧到达时备好，派生类直接取用）
    Vector3f _attitude_b_deg;     // 延迟后的机体姿态角（LPF 链的延迟量）
    Vector3f _gyro_b_dps;         // 延迟后的机体角速度（EKF 的机动门控也用它）
    float _observation_dt_s;      // 本观测帧与上一帧的间隔
    Vector3f _los_e_unit;
    Vector3f _los_e_dot;
    Vector3f _los_e_rate_lpf;
    Vector3f _gyro_c_dps;
    Vector3f _los_c_rate_lpf;
    // EKF 输出的 LOS 角速度（惯性系/相机系），与 *_lpf 同口径，供对照与接管
    Vector3f _los_e_rate_ekf;
    Vector3f _los_c_rate_ekf;
    // 目标源自带 LOS rate（LOS_TYPE=2）折算出的相机系/惯性系分量，与 *_lpf/*_ekf 同口径
    Vector3f _los_e_rate_src;
    Vector3f _los_c_rate_src;
    bool _src_rate_valid;             // 源速率可用（收到过，且未超时）
    uint32_t _src_rate_last_ms;       // 最近一次收到源速率的时刻
    bool _src_rate_warned;            // 源速率丢失→回退 LPF 只警告一次
    bool _src_no_rate_warned;         // 源根本没有速率通道(LOS_TYPE=2)→只警告一次
    bool _los_type_warned;            // LOS_TYPE 选中链不可用→回退只警告一次

    bool have_last_target;
    bool los_rate_control_active;
    // 延迟量按链分开命名，最后按 UATK_EKF_EN 选一条作为全局量 _dly：
    //   _dly_lpf   LPF 链用的延迟（初值 UATK_LPF_DLY；EST=1 且 CAM 源时由 _dly_est 在线估计）
    //   _dly_ekf   EKF 链用的延迟（滤波器第 7 维状态）
    //   _dly       选中的那一条（控制/日志/打印用）
    float _dly_lpf;
    float _dly_ekf;
    float _dly;
    bool _ekf_ready;                  // EKF 两通道均已初始化（可参与制导）
    bool _cam_updated_this_cycle;     // 本周期是否有新相机量测（高率日志用）
    Vector2f _hilog_target_c_deg;     // 本周期相机量测（高率日志用）
    uint32_t last_observation_ms;
    uint32_t last_filter_config_ms;
    uint32_t last_log_ms;
    uint32_t update_call_count;
    uint32_t update_rate_last_ms;
};
