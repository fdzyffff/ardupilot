#include "FD_Attack.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

// 目标源自带 LOS rate 的超时门限：超过这么长时间没有新速率帧就判为不可用、回退 LPF
// （在此之内保持上一帧值，避免丢一两帧时链上跳变）。100Hz 源 ≈ 连续丢 50 帧。
static const uint32_t SRC_RATE_TOUT_MS = 500;

const AP_Param::GroupInfo FD_Attack::var_info[] = {
    // @Param: TCAM_USE
    // @DisplayName: Camera target source
    // @Description: 相机类导引头目标源选择: 0=不使用 1=HY(慧眼) 2=DYT2(反无吊舱)
    // @Values: 0:None,1:HY,2:DYT2
    // @User: Advanced
    AP_GROUPINFO("TCAM_USE", 0, FD_Attack, use_target_cam, 1),
    // 位置两路目标源默认打开，打击模式同时使用（相机优先、位置兜底）
    AP_GROUPINFO("TLOC_USE", 1, FD_Attack, use_target_loc, 1),
    AP_GROUPINFO("DEBUG", 2, FD_Attack, debug_print, 0),
    // @Param: HILOG
    // @DisplayName: High-rate IMU and camera logging
    // @Description: 1=每次 FD_Attack::update()（实测 200Hz）记两条高率日志：UAH1（两种方法的惯性系 LOS 角 θ 与角速率 θ̇ 并列：Lp="差分+低通"链、Ek=延迟补偿 EKF，各分横向/纵向，共 8 量）、UAH2（机体相机系角速度 p/y，2 量）。这两条日志**常开**：本开关=0 时它们按常规速率（10Hz）记录，=1 时提速到 200Hz（用于离线标定导引头管线延迟 Δ 与对比两种估计方法）。背景：常规 IMU 消息只有 25Hz（Nyquist 12.5Hz），不足以做 Δ 判据。开启后日志约增加 11KB/s（≈0.7MB/min），对 SD 卡写入与调度有负担，**正常飞行保持 0**，只在专门的地面/挂飞标定或对照架次打开。
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("HILOG", 3, FD_Attack, hi_log, 0),
    // 两条 LOS 估计链的参数各自成组（组内第一条带 AP_PARAM_FLAG_ENABLE：
    // 开关为 0 时该组其余参数不在参数列表/下载里出现）
    AP_SUBGROUPINFO(lpf, "LPF_", 4, FD_Attack, FD_LPF),
    AP_SUBGROUPINFO(ekf, "EKF_", 5, FD_Attack, FD_EKF),
    AP_SUBGROUPINFO(target_loc_inst, "TL_", 6, FD_Attack, FD_Target_Loc),
    AP_SUBGROUPINFO(target_HY, "TC_", 7, FD_Attack, FD_Target_HY),
    // DYT2(反无吊舱) 导引头参数组 UATK_TD_*
    AP_SUBGROUPINFO(target_DYT2, "TD_", 8, FD_Attack, FD_Target_DYT2),
    // @Param: LOS_TYPE
    // @DisplayName: LOS rate source
    // @Description: 制导用的 LOS 角速率来源: 0=差分+低通链(LPF) 1=延迟补偿EKF 2=目标源自带(导引头直接给出)。LOS_TYPE=2 时角度仍取 LPF 链，只有角速度换成源自带的那一份；EKF_EN=0 或 EKF 未就绪时 LOS_TYPE=1 自动回退 LPF。
    // @Values: 0:LPF,1:EKF,2:Source
    // @User: Advanced
    AP_GROUPINFO("LOS_TYPE", 9, FD_Attack, los_type, 0),
    // 序号约定：本库 0~20（9~20 留空备用），端侧派生类 21 起
    AP_GROUPEND
};

FD_Attack::FD_Attack()
{
    target_cam = nullptr;
    target_loc = nullptr;
    target_cam_initialized = false;
    target_loc_initialized = false;
    _target_source = TargetSource::NONE;
    have_last_target = false;
    los_rate_control_active = false;
    last_observation_ms = 0;
    last_filter_config_ms = 0;
    last_log_ms = 0;
    update_call_count = 0;
    update_rate_last_ms = 0;
    _los_e_rate_ekf.zero();
    _los_c_rate_ekf.zero();
    _dly_lpf = 0.0f;
    _dly_ekf = 0.0f;
    _dly = 0.0f;
    _ekf_ready = false;
    _cam_updated_this_cycle = false;
    _hilog_target_c_deg.zero();
    // 目标源自带 LOS rate（LOS_TYPE=2）的对外量与状态标志
    _los_e_rate_src.zero();
    _los_c_rate_src.zero();
    _src_rate_valid = false;
    _src_rate_last_ms = 0;
    _src_rate_warned = false;
    _src_no_rate_warned = false;
    _los_type_warned = false;
    // 这几个成员不能依赖 BSS 零值：_lpf_en_warned 决定"强制启用"警告是否打印；
    // _los_e_deg_lpf/_ekf 被 update_los() 每周期无条件取用回写 _los_e_deg，
    // 不清则首次捕获前 UAT1 的 Le 列 / bit0 打印是未初始化值
    _lpf_en_warned = false;
    _los_e_deg_lpf.zero();
    _los_e_deg_ekf.zero();
    AP_Param::setup_object_defaults(this, var_info);
}

void FD_Attack::init()
{
    attitude_delay.init();
    gyro_delay.init();
    rotation_camera_to_body.from_euler(0.0f, radians(90.0f), 0.0f);
    rotation_body_to_camera.from_euler(0.0f, radians(90.0f), 0.0f);
    rotation_body_to_camera.transpose();

    _target_c_deg.zero();
    _los_e_deg.zero();
    _los_e_deg_lpf.zero();
    _los_e_deg_ekf.zero();
    _lpf_en_warned = false;
    _los_e_unit.zero();
    _los_e_dot.zero();
    _los_e_rate_lpf.zero();
    _gyro_c_dps.zero();
    _los_c_rate_lpf.zero();

    lpf.init();
    lpf.set_cutoff_hz(lpf.hz.get());

    // 联合 EKF 初始化：Δ 的三项可调量（QD/DMAX/SIG0）先下发再 init()，
    // 使首帧协方差与 Δ 钳位立刻用上参数值；Δ 状态初值取 UATK_LPF_DLY
    ekf.set_dly_process_scale(ekf.qd.get());
    ekf.set_dly_max_ms(ekf.dmax_ms.get());
    ekf.set_dly_init_sigma_ms(ekf.sig0_ms.get());
    ekf.init(lpf.dly_ms.get());
    ekf.set_process_scale(ekf.q.get());
    ekf.set_measurement_sigma_deg(ekf.r.get());
    _los_e_rate_ekf.zero();
    _los_c_rate_ekf.zero();
    _ekf_ready = false;
    _cam_updated_this_cycle = false;
    _hilog_target_c_deg.zero();
    _dly_lpf = constrain_float(lpf.dly_ms.get(), 0.0f, 1000.0f);
    _dly_ekf = _dly_lpf;   // EKF 未跑前给判读一个可视值（真值由滤波器状态给出）
    _dly = _dly_lpf;
    // 目标源自带 LOS rate（LOS_TYPE=2）的对外量与就绪标志
    _los_e_rate_src.zero();
    _los_c_rate_src.zero();
    _src_rate_valid = false;
    _src_rate_last_ms = 0;
    _src_rate_warned = false;
    _src_no_rate_warned = false;
    _los_type_warned = false;
    // 在线 Δ 估计器以 UATK_LPF_DLY 为初值（钳位在 [10,200]ms 内）
    _dly_est.init(lpf.dly_ms.get());

    init_targets();
}

// 顶层骨架——只有五步，具体计算都在对应函数里：
//   update_runtime_params()  每周期参数/系数刷新（必须是第一步：延迟历史要早于
//                            目标层与 LOS 计算，Δ/Q/R 必须在两条链使用前刷新）
//   update_targets()         目标源管理 → 本帧相机离轴角 _target_c_deg
//                            + 观测帧簿记（_observation_dt_s / 相机更新标志 / 高率日志量）
//   update_los()             取延迟状态 → 两条链各自更新 + 源自带速率 → 按 UATK_LOS_TYPE 选一套
//   update_control()         控制量合成（纯虚，端侧实现）
//   update_log()             常规日志 100ms 一次（UAT1~UAT4 + 派生类 UAT5~UAT7），
//                            高率日志（UATK_HILOG=1）每次
void FD_Attack::update()
{
    update_call_count++;

    update_runtime_params();
    update_targets();
    update_los();
    update_control();
    update_log();
}

void FD_Attack::init_targets()
{
    target_cam_initialized = false;
    target_loc_initialized = false;
    target_cam = nullptr;
    target_loc = nullptr;

    // 相机源按 UATK_TCAM_USE 选实现（0=无 1=HY 2=DYT2），选中后挂指针
    if (use_target_cam.get() == 1) {
        target_cam = &target_HY;
    } else if (use_target_cam.get() == 2) {
        target_cam = &target_DYT2;
    }
    if (target_cam != nullptr) {
        target_cam_initialized = target_cam->init();
        if (!target_cam_initialized) {
            gcs().send_text(MAV_SEVERITY_WARNING, "UAttack CAM unavailable");
        }
    }

    if (use_target_loc.get() != 0) {
        target_loc = &target_loc_inst;
        target_loc_initialized = target_loc->init();
        if (!target_loc_initialized) {
            gcs().send_text(MAV_SEVERITY_WARNING, "UAttack LOC unavailable");
        }
    }
}

void FD_Attack::update_delay_history()
{
    const Vector3f attitude_b_deg(degrees(AP::ahrs().get_roll_rad()),
                                  degrees(AP::ahrs().get_pitch_rad()),
                                  degrees(AP::ahrs().get_yaw_rad()));
    attitude_delay.push(attitude_b_deg);

    const Vector3f gyro_b_rads = AP::ahrs().get_gyro_latest();
    const Vector3f gyro_b_dps(degrees(gyro_b_rads.x),
                              degrees(gyro_b_rads.y),
                              degrees(gyro_b_rads.z));
    gyro_delay.push(gyro_b_dps);
}

// 按"时间戳线性插值"取延迟后的姿态/陀螺（UDelay 内部找相邻样本插值）。
// 注意不能用"硬编码频率 × Δ 折算步数"的写法：本类 update() 的实际调度频率是
// 200Hz（SCHED_TASK 请求 150Hz，AP_Scheduler 整型截断 interval_ticks=400/150=2 tick），
// 频率假设错了 Δ 就只生效一部分，晃动时残差 ω×Δ_err 表现为虚假 LOS rate。
// 缓冲未覆盖目标时刻时回退为当前值并返回 false。
bool FD_Attack::get_delayed_state(Vector3f &attitude_b_deg, Vector3f &gyro_b_dps) const
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t t_target_ms = now_ms - (uint32_t)constrain_float(_dly_lpf, 0.0f, 2000.0f);

    bool attitude_valid = attitude_delay.get_at_time_angle(t_target_ms, attitude_b_deg);
    bool gyro_valid = gyro_delay.get_at_time(t_target_ms, gyro_b_dps);

    if (!attitude_valid) {
        attitude_b_deg = Vector3f(degrees(AP::ahrs().get_roll_rad()),
                                  degrees(AP::ahrs().get_pitch_rad()),
                                  degrees(AP::ahrs().get_yaw_rad()));
    }
    if (!gyro_valid) {
        const Vector3f gyro_b_rads = AP::ahrs().get_gyro_latest();
        gyro_b_dps = Vector3f(degrees(gyro_b_rads.x),
                              degrees(gyro_b_rads.y),
                              degrees(gyro_b_rads.z));
    }
    return attitude_valid && gyro_valid;
}

void FD_Attack::clear_target_output()
{
    // 控制侧清零在派生类的 on_target_lost()
    on_target_lost();
    _target_c_deg.zero();
    _los_e_deg.zero();
    // 两条链的 LOS 角也要清——update_los() 每周期用它俩回写 _los_e_deg，
    // 只清 _los_e_deg 的话，目标丢失期间 UAT1 的 Le 列会显示上一次捕获的旧角度
    _los_e_deg_lpf.zero();
    _los_e_deg_ekf.zero();
    _los_e_unit.zero();
    _los_e_dot.zero();
    _los_e_rate_lpf.zero();
    _gyro_c_dps.zero();
    _los_c_rate_lpf.zero();
    // 目标源自带速率同样清零（LOS_TYPE=2 时它是对外量）
    _src_rate_valid = false;
    _los_e_rate_src.zero();
    _los_c_rate_src.zero();
    // EKF 输出镜像同步清零（滤波器状态本身的复位只在源变化时由 reset_chains() 做一次——
    // 本函数在目标丢失期间每周期都会被调用，任何乘性/累积式复位放这里都会发散）
    _los_e_rate_ekf.zero();
    _los_c_rate_ekf.zero();
    have_last_target = false;
    los_rate_control_active = false;
    last_observation_ms = 0;
}

void FD_Attack::handle_attack_msg(const mavlink_message_t &msg)
{
    if (target_loc_initialized) {
        target_loc->handle_msg(msg);
    }
    if (target_cam_initialized) {
        target_cam->handle_msg(msg);
    }
}

Location FD_Attack::get_target_loc()
{
    if (target_loc == nullptr) {
        return Location();
    }
    return target_loc->get_target_loc();
}

void FD_Attack::set_target_loc(Location &loc_in)
{
    if (target_loc_initialized) {
        target_loc->set_target_loc(loc_in);
    }
}

// 只做记录：常规速率日志每 100ms 一条（无观测时记零阶保持值）；
// 高率日志（UATK_HILOG=1）每次调用都记。
// 日志编号：基类 UAT1~UAT4，高率 UAH1/UAH2，派生类（控制侧）UAT5~UAT7。
void FD_Attack::update_log()
{
    const uint32_t now_ms = AP_HAL::millis();

    // ---- 常规速率日志（10Hz）----
    if (now_ms - last_log_ms >= 100U) {
        last_log_ms = now_ms;
        AP::logger().WriteStreaming(
            "UAT1",
            "TimeUS,Src,LcY,LcP,Fps",
            "s-ddz",
            "F----",
            "QBfff",
            AP_HAL::micros64(),
            (uint8_t)_target_source,
            _target_c_deg.x,
            _target_c_deg.y,
            (target_cam != nullptr) ? target_cam->get_fps() : 0.0f);

        // UAT2：EKF 开关 / LOS 速率来源 / 有效延迟 / θ̇ 的 1σ
        AP::logger().WriteStreaming(
            "UAT2",
            "TimeUS,En,LT,Dly,SgY,SgP",
            "s---kk",
            "F-----",
            "QBBHff",
            AP_HAL::micros64(),
            (uint8_t)ekf.enable.get(),
            (uint8_t)los_type.get(),
            (uint16_t)_dly,
            ekf.get_sigma_theta_dot_dps(FD_EKF::Channel::YAW),
            ekf.get_sigma_theta_dot_dps(FD_EKF::Channel::PITCH));

        // UAT3：归一化新息 + 三路相机系 LOS rate 并排（Lp=差分+低通链 / Ek=EKF / Sr=目标源自带）
        AP::logger().WriteStreaming(
            "UAT3",
            "TimeUS,InY,InP,LpY,LpP,EkY,EkP,SrY,SrP",
            "s--kkkkkk",
            "F--------",
            "Qffffffff",
            AP_HAL::micros64(),
            ekf.get_innovation_norm(FD_EKF::Channel::YAW),
            ekf.get_innovation_norm(FD_EKF::Channel::PITCH),
            _los_c_rate_lpf.z,
            _los_c_rate_lpf.y,
            _los_c_rate_ekf.z,
            _los_c_rate_ekf.y,
            _los_c_rate_src.z,
            _los_c_rate_src.y);

        // UAT4：本周期是否有相机帧 + 本帧相机量测（高率标定用）
        AP::logger().WriteStreaming(
            "UAT4",
            "TimeUS,CamUpd,CamY,CamP,Src",
            "s-dd-",
            "F----",
            "QBffB",
            AP_HAL::micros64(),
            (uint8_t)(_cam_updated_this_cycle ? 1 : 0),
            _hilog_target_c_deg.x,
            _hilog_target_c_deg.y,
            (uint8_t)_target_source);

        // 控制侧日志（UAT5~UAT7）——实现在端侧派生类。
        // 排在基类四条之后，使写入顺序与编号顺序一致（UAT1 → UAT7）
        update_control_log();

        // UAH1/UAH2 常开：UATK_HILOG=0 时按常规速率（10Hz）记在这里
        if (hi_log.get() == 0) {
            write_uah_msgs();
        }
    }

    // ---- 高率日志：UATK_HILOG=1 时每次 update() 都记 ----
    if (hi_log.get() != 0) {
        write_uah_msgs();
    }
}

void FD_Attack::do_print()
{
    const int16_t print_mask = debug_print.get();

    // 实测 update() 调用频率（1Hz 打印窗口内计数）
    const uint32_t now_ms = AP_HAL::millis();
    float update_rate_hz = 0.0f;
    if (update_rate_last_ms != 0) {
        const float dt_s = (now_ms - update_rate_last_ms) * 0.001f;
        if (dt_s > 0.0f) {
            update_rate_hz = update_call_count / dt_s;
        }
    }
    update_call_count = 0;
    update_rate_last_ms = now_ms;

    if (print_mask & (1U << 0)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK Lc Y/P:%.1f/%.1f Le Y/P:%.1f/%.1f",
                        _target_c_deg.x,
                        _target_c_deg.y,
                        _los_e_deg.x,
                        _los_e_deg.y);
    }
    if (print_mask & (1U << 1)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK Le xyz:%.2f/%.2f/%.2f",
                        _los_e_unit.x,
                        _los_e_unit.y,
                        _los_e_unit.z);
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK LeD xyz:%.2f/%.2f/%.2f",
                        _los_e_dot.x,
                        _los_e_dot.y,
                        _los_e_dot.z);
    }
    if (print_mask & (1U << 2)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK LcR xyz:%.1f/%.1f/%.1f",
                        _los_c_rate_lpf.x,
                        _los_c_rate_lpf.y,
                        _los_c_rate_lpf.z);
    }
    // 控制侧打印（bit0 相机姿态、bit3/4/5/7）在派生类
    do_print_control(print_mask);
    if (print_mask & (1U << 6)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK FPS:%.1f UpdHz:%.1f",
                        (target_cam != nullptr) ? target_cam->get_fps() : 0.0f,
                        update_rate_hz);
    }
    if (print_mask & (1U << 8)) {
        // EKF 在线判读：开关与有效延迟、EKF 的 θ̇ 与 1σ、两链相机系 LOS rate 对照
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK EKF En:%u Dly lp/ek/sel:%u/%u/%u",
                        (unsigned)ekf.enable.get(),
                        (unsigned)_dly_lpf,
                        (unsigned)_dly_ekf,
                        (unsigned)_dly);
        // Δ 的 1σ（判 UATK_EKF_QD / SIG0）+ 快照历史是否够长
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK EKF SgD:%.1f HistLim:%u",
                        (double)ekf.get_sigma_delay_ms(),
                        (unsigned)(ekf.is_history_limited() ? 1 : 0));
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK EKF Td Y/P:%.1f/%.1f Sg Y/P:%.1f/%.1f",
                        ekf.get_theta_dot_dps(FD_EKF::Channel::YAW),
                        ekf.get_theta_dot_dps(FD_EKF::Channel::PITCH),
                        ekf.get_sigma_theta_dot_dps(FD_EKF::Channel::YAW),
                        ekf.get_sigma_theta_dot_dps(FD_EKF::Channel::PITCH));
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "ATK EKF In Y/P:%.1f/%.1f",
                        (double)ekf.get_innovation_norm(FD_EKF::Channel::YAW),
                        (double)ekf.get_innovation_norm(FD_EKF::Channel::PITCH));
    }
}

// EKF 链是否用于制导。选择权在 UATK_LOS_TYPE（1=EKF）：除开关外还要求 UATK_EKF_EN≠0
// （EKF 链在跑）且两通道都已初始化，避免 EKF 未收敛时把零值喂进制导环。
bool FD_Attack::ekf_for_control() const
{
    return (los_type.get() == 1) && (ekf.enable.get() != 0) && _ekf_ready;
}

// LPF 链是否运行。UATK_LPF_EN=0 时仍需它在两种情况下跑：
//   ① LOS_TYPE≠0（LPF 不是被选中的那一套）——它是兜底量与 UAT3 的对照量；
//   ② EKF 未接管制导——两条链不能都不跑，否则 LOS 失去来源。
bool FD_Attack::lpf_enabled() const
{
    if (lpf.enable.get() != 0) {
        return true;
    }
    return (los_type.get() != 0) || !ekf_for_control();
}

// 联合 EKF 量测更新——把【原始相机离轴角 b】（未去旋转）送进滤波器。
// 去旋转在滤波器内部用状态 Δ̂ 完成（若先去旋转再量测 θ，∂z/∂Δ=0，Δ 永远不可观）；
// gyro_b_dps 仅用于机动门控（直流 ω 下 Δ 与缓变 LOS 不可分，冻结 Δ 通道增益）。
// 随后把两平面 θ̇ 组装回惯性系 LOS 角速度三向量（口径：ω = e × ė）。
void FD_Attack::ekf_measure(const Vector2f &target_c_deg, uint32_t t_arr_ms, const Vector3f &gyro_b_dps)
{
    ekf.push_observation(target_c_deg, t_arr_ms, attitude_delay, gyro_b_dps, rotation_camera_to_body);

    // 由两平面坐标率闭式组装惯性系 LOS 角速度（ω = e × ė，e 为单位向量）：
    //     ω = θ̇y·cosθp·(sinθp·cosθy, sinθp·sinθy, cosθp) + θ̇p·(−sinθy, cosθy, 0)
    // 关键：几何量 (θy,θp) 取"本帧实测方向"（滤波器内部按状态 Δ̂ 去旋转的结果），
    // 速率量才取滤波器输出——视线方向本身处处良态（极点处只是方位角坐标不可观测），
    // 而滤波器偏航状态在极点会被门控冻结，取它会把 ω 的方向算错。
    const float theta_y_rad = radians(ekf.get_meas_theta_deg(FD_EKF::Channel::YAW));
    const float theta_p_rad = radians(ekf.get_meas_theta_deg(FD_EKF::Channel::PITCH));
    const float sin_tp = sinf(theta_p_rad);
    const float cos_tp = cosf(theta_p_rad);
    const float sin_ty = sinf(theta_y_rad);
    const float cos_ty = cosf(theta_y_rad);

    const float theta_dot_yaw_dps = ekf.get_theta_dot_dps(FD_EKF::Channel::YAW);
    const float theta_dot_pitch_dps = ekf.get_theta_dot_dps(FD_EKF::Channel::PITCH);

    const Vector3f term_yaw(sin_tp * cos_ty, sin_tp * sin_ty, cos_tp);
    // cosθp 因子：偏航平面坐标率对 ω 的贡献在视线指向天顶/天底时自然归零（与门控一致）
    _los_e_rate_ekf = term_yaw * (theta_dot_yaw_dps * cos_tp)
                          + Vector3f(-sin_ty, cos_ty, 0.0f) * theta_dot_pitch_dps;
}

// 高率日志的统一写入点（常规速率与高率共用；由调用方保证互斥）。
//   UAH1：两种方法的惯性系 LOS 角 θ 与角速率 θ̇ 并列（Lp=LPF 链，Ek=EKF），各分
//         横向/纵向共 8 个量；同一行里逐列相减即两法差，无需按时间戳对齐。
//   UAH2：机体相机系角速度 p(绕相机 y)/y(绕相机 z)，deg/s。
//         UAH1 的 θ/θ̇ + UAH2 的 ω_cam 足以在离线端重建两条链的相机系 LOS rate。
// 背景：常规 IMU 消息只有 25Hz/实例（Nyquist 12.5Hz），不足以做"像素速率与陀螺
// 去相关"的 Δ 判据，标定架次才开 UATK_HILOG 提速到每周期。
void FD_Attack::write_uah_msgs()
{
    const Vector3f gyro_b_rads = AP::ahrs().get_gyro_latest();
    const Vector3f gyro_b_dps(degrees(gyro_b_rads.x),
                              degrees(gyro_b_rads.y),
                              degrees(gyro_b_rads.z));
    // 相机系角速度：x=光轴 / y=右 / z=下；只记 p(绕相机 y) 与 y(绕相机 z)
    const Vector3f gyro_c_dps = rotation_body_to_camera * gyro_b_dps;

    AP::logger().WriteStreaming(
        "UAH1",
        "TimeUS,LpThY,LpThP,LpTdY,LpTdP,EkThY,EkThP,EkTdY,EkTdP",
        "sddkkddkk",
        "F--------",
        "Qffffffff",
        AP_HAL::micros64(),
        lpf.get_theta_deg(FD_LPF::Channel::YAW),
        lpf.get_theta_deg(FD_LPF::Channel::PITCH),
        lpf.get_theta_dot_dps(FD_LPF::Channel::YAW),
        lpf.get_theta_dot_dps(FD_LPF::Channel::PITCH),
        ekf.get_theta_deg(FD_EKF::Channel::YAW),
        ekf.get_theta_deg(FD_EKF::Channel::PITCH),
        ekf.get_theta_dot_dps(FD_EKF::Channel::YAW),
        ekf.get_theta_dot_dps(FD_EKF::Channel::PITCH));

    AP::logger().WriteStreaming(
        "UAH2",
        "TimeUS,GP,GY",
        "skk",
        "F--",
        "Qff",
        AP_HAL::micros64(),
        gyro_c_dps.y,
        gyro_c_dps.z);
}

// ==================== update() 骨架化的子步骤 ====================

// ① 每周期参数/系数刷新：延迟历史、1Hz 滤波系数、有效延迟 Δ、EKF 的 Q/R/Δ 标度
void FD_Attack::update_runtime_params()
{
    update_delay_history();

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_filter_config_ms >= 1000U) {
        lpf.set_cutoff_hz(lpf.hz.get());
        last_filter_config_ms = now_ms;
    }

    // UATK_LPF_EN=0 但 LPF 仍被强制保留（LOS_TYPE≠0 时它是对照/兜底；EKF 未接管时它是唯一来源）
    // → 见 lpf_enabled()，警告一次。LPF 被选中且正常启用时不警告。
    if (lpf.enable.get() == 0 && (los_type.get() != 0 || !ekf_for_control()) && !_lpf_en_warned) {
        _lpf_en_warned = true;
        gcs().send_text(MAV_SEVERITY_WARNING, "ATK LPF_EN=0 but LPF required: forced ON");
    }

    // UATK_LOS_TYPE=1（用 EKF）但 EKF 链没跑或还没收敛 → 已回退 LPF，警告一次
    if (los_type.get() == 1 && (ekf.enable.get() == 0 || !_ekf_ready) && !_los_type_warned) {
        _los_type_warned = true;
        gcs().send_text(MAV_SEVERITY_WARNING, "ATK LOS_TYPE=1 but EKF not ready: fallback LPF");
    }

    // 有效延迟每周期刷新。UATK_LPF_EST=1 且 CAM 源时由在线 Δ 估计器接管
    // （估计器内部已钳位 [10,200]ms，冻结期保持最后值）；否则回退参数 UATK_LPF_DLY。
    if (lpf.est_en.get() != 0 && _target_source == TargetSource::CAM) {
        _dly_lpf = _dly_est.get_delay_ms();
    } else {
        _dly_lpf = constrain_float(lpf.dly_ms.get(), 0.0f, 1000.0f);
    }
    ekf.set_process_scale(ekf.q.get());
    // Δ 状态的三项现场可调量也每周期下发（改参即时生效）：
    // set_dly_init_sigma_ms 只影响首帧初始化时的 P[Δ]，周期刷新无副作用；
    // set_dly_max_ms 收紧后在下一次量测更新的 Δ 钳位处生效
    // （快照历史槽数按 0.2s 硬上限预留，不受影响）。
    ekf.set_dly_process_scale(ekf.qd.get());
    ekf.set_dly_max_ms(ekf.dmax_ms.get());
    ekf.set_dly_init_sigma_ms(ekf.sig0_ms.get());
    // EKF 的量测噪声直接用 UATK_EKF_R：延迟 Δ 已进状态空间，不需要再按外挂估计器
    // 的实测散度抬高 R（Δ 残余由 H 的 Δ 列与协方差建模，再抬 R 属双重计数，
    // 且会把两个估计器耦合起来）
    ekf.set_measurement_sigma_deg(ekf.r.get());
    // 本周期相机量测标志清零（供高率标定日志 UAT4 判断"相机是否有更新"）
    _cam_updated_this_cycle = false;
}

// ② 目标源：目标层 update + 外部注入（31011 靶机 / AP_Follow）+ 源判定与切换复位
void FD_Attack::update_targets()
{
    // 目标源管理到"本帧相机离轴角 _target_c_deg"为止（LOS 计算全在 update_los）
    if (target_cam_initialized) {
        target_cam->update();
    }
    if (target_loc_initialized) {
        // 外部注入由端侧派生类实现，经 set_target_loc() 落地
        update_external_targets();
        target_loc->update();
    }

    TargetSource new_source = TargetSource::NONE;
    if (target_cam_initialized && target_cam->is_valid()) {
        new_source = TargetSource::CAM;
    } else if (target_loc_initialized && target_loc->is_valid()) {
        new_source = TargetSource::LOC;
    }

    if (new_source != _target_source) {
        if (new_source == TargetSource::CAM) {
            gcs().send_text(MAV_SEVERITY_INFO, "UAttack source CAM");
        } else if (new_source == TargetSource::LOC) {
            gcs().send_text(MAV_SEVERITY_INFO, "UAttack source LOC");
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "UAttack no target");
        }
        _target_source = new_source;
        have_last_target = false;
        last_observation_ms = 0;
        los_rate_control_active = false;
        // 控制侧复位（速率环/阶段/相机姿态/速度/目标速率/滞后偏置）交给派生类
        on_source_changed();
        // 源发生变化 → 两条链统一复位到初始状态（见 reset_chains()）。
        // 语义：目标为 NONE 时两链都处于初始态、不工作；进入 LOC 或 CAM 就从初始态
        // 重新起算——Δ 回到 UATK_LPF_DLY 预设值重新在线估，接受重捕后 ~1s 收敛。
        // LOC→CAM 必须复位：LOC 量测没有相机管线延迟，Δ 若沿用会被带偏。
        // 不用 ekf.soften()（只放大协方差、保留状态）：LOC/CAM 延迟语义不同，
        // 旧状态反而有害；且 soften 是乘性累积的，放每周期路径上会把 P 爆掉。
        reset_chains();
    }

    if (_target_source == TargetSource::NONE) {
        // 无目标：清输出（控制侧复位由派生类的 on_target_lost() 完成）；日志照记
        clear_target_output();
        _have_observation = false;
        return;
    }

    // 本帧观测 = 目标在相机系下的离轴角（未做任何去旋转）；没有新帧时
    // _have_observation=false，两条链都不推进（只在收到观测帧时更新）。
    _have_observation = false;
    if (_target_source == TargetSource::CAM) {
        _have_observation = target_cam->get_info(_target_c_deg.x, _target_c_deg.y);
    } else if (_target_source == TargetSource::LOC) {
        _have_observation = target_loc->get_info(_target_c_deg.x, _target_c_deg.y);
    }

    // 本帧观测帧簿记：帧间隔/相机更新标志/高率日志量。两条链与派生类控制量读同一份。
    // 无新观测时不刷新（零阶保持）；相机更新标志由 update_runtime_params() 每周期开头清零。
    if (_have_observation) {
        const uint32_t now_ms = AP_HAL::millis();
        _cam_updated_this_cycle = true;
        _hilog_target_c_deg = _target_c_deg;
        _observation_dt_s = last_observation_ms == 0 ?
                            0.01f :
                            (float)(now_ms - last_observation_ms) * 0.001f;
        last_observation_ms = now_ms;
    }
}

// ③ LOS 计算：取延迟状态 → 两条链各自更新 → 按 UATK_EKF_EN 选一套给最终量。
// 两条链各自判"源是否有效 / 本帧有无新观测"，互不干扰，都只用 _target_c_deg 一份原始量测。
// 与调度频率无关的设计：所有 dt 都按时间戳实测（帧间隔、UDelay 插值取数、低通用实测 dt
// 施加），滤波器的历史/窗口按时间跨度取槽数，不写死 Hz。
void FD_Attack::update_los()
{
    // 公共输入：延迟后的机体姿态/陀螺（LPF 链去旋转 + EKF 机动门控 + 控制量都要）
    get_delayed_state(_attitude_b_deg, _gyro_b_dps);

    const uint32_t now_ms = AP_HAL::millis();
    update_lpf(now_ms);     // "差分+低通"链（含在线 Δ 估计）；源无效/无新观测在函数内直接返回
    update_ekf(now_ms);     // 7 状态联合 EKF（含每周期预测步）；同上
    _gyro_c_dps = rotation_body_to_camera * _gyro_b_dps;
    // 目标源自带 LOS rate（LOS_TYPE=2 用；折算要用 _gyro_c_dps，故排在它之后）
    update_source_rate(now_ms);

    // 按 UATK_LOS_TYPE 选一套作为最终量（控制量与常规日志都用它）：
    //   0 = LPF 链  1 = EKF 链（需 UATK_EKF_EN≠0 且两通道已初始化）  2 = 目标源自带 LOS rate
    // 选中链不可用时逐级回退（见下方 warn），绝不出现"没有来源"的中间态。
    const uint8_t los_src = (uint8_t)los_type.get();
    const bool use_ekf = (los_src == 1) && (ekf.enable.get() != 0) && _ekf_ready;
    const bool use_src = (los_src == 2) && _src_rate_valid;
    _los_e_deg  = use_ekf ? _los_e_deg_ekf  : _los_e_deg_lpf;   // 源速率只替代角速度，角度仍取 LPF 链
    if (use_src) {
        _los_e_rate = _los_e_rate_src;
        _los_c_rate = _los_c_rate_src;
    } else if (use_ekf) {
        _los_e_rate = _los_e_rate_ekf;
        _los_c_rate = _los_c_rate_ekf;
    } else {
        _los_e_rate = _los_e_rate_lpf;
        _los_c_rate = _los_c_rate_lpf;
    }
    _dly        = use_ekf ? _dly_ekf        : _dly_lpf;
}

// "差分+低通"链（含只服务本链的在线 Δ 估计）→ _los_*_lpf / _dly_lpf。
// 无新观测（源为 NONE 时 _have_observation 也是 false）直接返回：最终量保持上一帧值。
void FD_Attack::update_lpf(uint32_t now_ms)
{
    if (_target_source == TargetSource::NONE) {
        return;
    }
    if (!_have_observation) {
        return;
    }
    if (!lpf_enabled()) {
        return;     // UATK_LPF_EN=0 且 EKF 已接管制导：本链不跑
    }

    // 去旋转：相机系离轴角 → 相机系单位向量 → 惯性系单位向量（用延迟后的姿态）
    const float yaw_c_rad = radians(_target_c_deg.x);
    const float pitch_c_rad = radians(_target_c_deg.y);
    const Vector3f target_c_unit(cosf(pitch_c_rad) * cosf(yaw_c_rad),
                                 cosf(pitch_c_rad) * sinf(yaw_c_rad),
                                 -sinf(pitch_c_rad));

    Matrix3f rotation_ned_from_body;
    rotation_ned_from_body.from_euler(radians(_attitude_b_deg.x),
                                      radians(_attitude_b_deg.y),
                                      radians(_attitude_b_deg.z));
    const Matrix3f rotation_ned_from_camera =
        rotation_ned_from_body * rotation_camera_to_body;

    if (!have_last_target) {
        have_last_target = true;
        // 换源/重新捕获后首帧清一次 LPF 链滤波器状态（首帧用样本自动初始化）
        lpf.reset();
    }

    // 低通用【本帧实测间隔】施加（_observation_dt_s），与调度/相机帧率无关
    lpf.update(target_c_unit, rotation_ned_from_camera, now_ms, _observation_dt_s);
    _los_e_unit     = lpf.get_los_e_unit();
    _los_e_dot      = lpf.get_los_e_dot();
    _los_e_deg_lpf  = lpf.get_los_e_deg();
    _los_e_rate_lpf = lpf.get_los_e_rate_dps();
    _los_c_rate_lpf = lpf.get_los_c_rate_dps();

    // 在线 Δ 估计：只服务 LPF 链与延迟姿态/陀螺取数——结果在下一周期开头由
    // update_runtime_params() 取用（→ _dly_lpf）；EKF 链的 Δ 在状态空间里，不共享。
    // 只在 CAM 源观测帧更新（LOC 源没有相机管线延迟）。
    if (_target_source == TargetSource::CAM && lpf.est_en.get() != 0) {
        _dly_est.update(_target_c_deg, now_ms, attitude_delay, _gyro_b_dps, rotation_camera_to_body);
    }
}

// 7 状态联合 EKF 链（Δ 在状态空间里）→ _los_*_ekf / _dly_ekf。
// 量测是【原始离轴角 _target_c_deg】（未去旋转）：去旋转在滤波器内部用状态 Δ̂ 完成，
// ∂h/∂Δ≠0，Δ 才可观；量测真实时刻 = 到达时刻 − Δ̂（滤波器内部快照回滚）。
// 预测步每周期都跑（不等相机帧），使 θ̇ 输出在帧间连续。不需要 400Hz 陀螺做帧间传播：
// 惯性系 LOS 的演化与机体转动无关（机体转动只在量测坐标系里体现，已由延迟姿态处理）。
void FD_Attack::update_ekf(uint32_t now_ms)
{
    // 源为 NONE 时本链不工作——连预测步都不跑（NONE 的那次源变化已 reset_chains() 置回初始态）
    if (_target_source == TargetSource::NONE) {
        return;
    }

    if (ekf.enable.get() != 0) {
        ekf.propagate(now_ms);
    }

    if (!_have_observation) {
        return;     // 无新观测帧：不做量测更新，EKF 输出保持上一帧值（零阶保持）
    }

    if (ekf.enable.get() == 0) {
        _los_e_deg_ekf.zero();
        _los_e_rate_ekf.zero();
        _los_c_rate_ekf.zero();
        _ekf_ready = false;
        _dly_ekf = _dly_lpf;      // 未运行 EKF：给判读一个可视值（不参与选择）
        return;
    }

    ekf_measure(_target_c_deg, now_ms, _gyro_b_dps);
    _los_e_deg_ekf = Vector2f(ekf.get_theta_deg(FD_EKF::Channel::YAW),
                              ekf.get_theta_deg(FD_EKF::Channel::PITCH));

    // 相机系输出：用同一套延迟姿态矩阵，与 LPF 链的 _los_c_rate_lpf 同口径
    Matrix3f rotation_ned_from_body;
    rotation_ned_from_body.from_euler(radians(_attitude_b_deg.x),
                                      radians(_attitude_b_deg.y),
                                      radians(_attitude_b_deg.z));
    Matrix3f rotation_camera_from_ned = rotation_ned_from_body * rotation_camera_to_body;
    rotation_camera_from_ned.transpose();
    _los_c_rate_ekf = rotation_camera_from_ned * _los_e_rate_ekf;

    _ekf_ready = ekf.is_initialised(FD_EKF::Channel::YAW) &&
                 ekf.is_initialised(FD_EKF::Channel::PITCH);
    _dly_ekf = ekf.get_delay_ms();
}

// 两条估计链的"初始状态"复位。调用点唯一：update_targets() 的"源发生变化"分支
// （每次源变化各调用一次，含 X→NONE）。语义：
//   · 无目标 = 两条链都处于初始态、不工作：EKF 不跑预测步（update_ekf 的 NONE 门 +
//     未初始化），LPF 链不推进（update_lpf 的 NONE 门）；对外量由 clear_target_output()
//     每周期清零。
//   · 一旦目标进入 LOC 或 CAM，从初始态重新起算：Δ 回到 UATK_LPF_DLY 预设值重新
//     在线估，接受重捕后约 1s 的收敛时间。
void FD_Attack::reset_chains()
{
    // ① LPF 链：清导数/低通滤波器状态（LowPassFilter::reset() 保留截止频率，
    //    不必重新下发 UATK_LPF_HZ；1Hz 刷新本来也会重设）
    lpf.init();

    // ② EKF 链：x 清零、Δ 取 UATK_LPF_DLY、P=单位矩阵、快照历史全部 invalid、
    //    _initialised=false（其后的无观测周期里 propagate() 也会因未初始化直接早退）
    ekf.init(lpf.dly_ms.get());

    // ③ 在线 Δ 估计器：清原始帧窗口与"连续干净窗"回滚基准——否则重捕后头一个
    //    窗口会混入丢失前的旧帧做回归，可能吐出一个假 Δ̂
    _dly_est.init(lpf.dly_ms.get());

    // ④ 对外量与就绪标志。务必清 _ekf_ready：否则 UATK_EKF_EN=2 时 ekf_for_control()
    //    仍为真，会把"已清空但尚未初始化"的 EKF 链（LOS rate=0）选给制导；
    //    清掉后选择部分自动回落到 LPF 链
    _ekf_ready = false;
    _los_e_deg_ekf.zero();
    _los_e_deg_lpf.zero();
    _los_e_rate_ekf.zero();
    _los_c_rate_ekf.zero();
    // ⑤ 目标源自带速率（LOS_TYPE=2）也回到"未收到"状态：否则换源后的头几个周期会用
    //    上一个源留下的速率去制导（角度已清零、速率却是旧的）
    _src_rate_valid = false;
    _src_rate_last_ms = 0;
    _los_e_rate_src.zero();
    _los_c_rate_src.zero();
    _dly_lpf = constrain_float(lpf.dly_ms.get(), 0.0f, 1000.0f);
    _dly_ekf = _dly_lpf;        // 未初始化时给判读一个可视值（不参与选择）
    _dly = _dly_lpf;
}

// ============ 目标源自带 LOS rate（UATK_LOS_TYPE=2） ============
// 取数与超时判定：
//   · 收到新速率帧 → 立即折算成相机系/惯性系分量（与两条链同口径），记时间戳
//   · 没收到       → 先保持上一帧值（零阶保持）；超过 SRC_RATE_TOUT_MS 判为不可用，
//                    由 update_los() 回退 LPF 链（并警告一次）
// 源速率通道由 FD_Target_Base::handle_info_rate() 提供：r1=方位角速率(右+) / r2=高低角速率(上+)
void FD_Attack::update_source_rate(uint32_t now_ms)
{
    if (_target_source == TargetSource::NONE) {
        _src_rate_valid = false;
        _src_rate_last_ms = 0;
        return;
    }

    // 当前源是否带速率通道：HY 这类只有像素测偏的源没有 → LOS_TYPE=2 时一次性告警
    // （只发一次，不刷屏；源切换后也不重复）
    const bool src_has_rate_channel =
        ((_target_source == TargetSource::CAM) && (target_cam != nullptr) && target_cam->has_rate_channel()) ||
        ((_target_source == TargetSource::LOC) && (target_loc != nullptr) && target_loc->has_rate_channel());
    if ((los_type.get() == 2) && !src_has_rate_channel && !_src_no_rate_warned) {
        _src_no_rate_warned = true;
        gcs().send_text(MAV_SEVERITY_WARNING, "ATK LOS_TYPE=2 but source has no LOS rate: fallback LPF");
    }

    float r1 = 0.0f;
    float r2 = 0.0f;
    bool got = false;
    if (_target_source == TargetSource::CAM && target_cam != nullptr) {
        got = target_cam->get_info_rate(r1, r2);
    } else if (_target_source == TargetSource::LOC && target_loc != nullptr) {
        got = target_loc->get_info_rate(r1, r2);
    }

    if (!got) {
        // 无新速率帧：保持上一帧值，直到超时
        if (_src_rate_valid && (now_ms - _src_rate_last_ms) > SRC_RATE_TOUT_MS) {
            _src_rate_valid = false;
            if (!_src_rate_warned) {
                _src_rate_warned = true;
                gcs().send_text(MAV_SEVERITY_WARNING, "ATK source LOS rate lost: fallback LPF");
            }
        }
        return;
    }

    _src_rate_last_ms = now_ms;
    _src_rate_valid = true;
    // 源速率 = 惯性系 LOS 角速率（与 LPF 链由 LOS 角微分得到的量同口径）：
    // 先在地理系组成三维角速度，再旋到相机系；姿态用【当前】姿态（源速率是近实时量）
    _los_e_rate_src = source_rate_earth(r1, r2);

    Matrix3f rotation_camera_from_ned = rotation_ned_from_body_current() * rotation_camera_to_body;
    rotation_camera_from_ned.transpose();
    _los_c_rate_src = rotation_camera_from_ned * _los_e_rate_src;
}

// 当前（未延迟）机体姿态 → NED 旋转矩阵。源速率折算用它：源自带速率是近实时量，
// 配 LPF 链的延迟姿态（默认 50ms，LPF_EST=1 时还会被在线估计器接管）会在机动中引入误差。
Matrix3f FD_Attack::rotation_ned_from_body_current() const
{
    Matrix3f rotation_ned_from_body;
    rotation_ned_from_body.from_euler(AP::ahrs().get_roll_rad(),
                                      AP::ahrs().get_pitch_rad(),
                                      AP::ahrs().get_yaw_rad());
    return rotation_ned_from_body;
}

// 源速率 → 惯性系 LOS 角速度（NED 分量, deg/s）。构造与 FD_LPF::update() 完全一致，
// 区别只是"由 LOS 角微分"换成"目标源直接给的角速率"：
//   ① 相机系 LOS 方向 u_c 经姿态旋到 NED 得 u_e；取其方位 az、高低 el
//   ② du_e/dt = ∂u/∂az·r1 + ∂u/∂el·r2（r 用弧度/秒）——单位向量导数，1/s
//   ③ ω_e = degrees(u_e × du_e/dt)   ← FD_LPF 里同样是 u × ė
// 口径说明：
//  · 源给的是 az/el 的 Euler 角速率（方位 右+ / 高低 上+），与 LPF 由 LOS 角算出的速率同一种量；
//  · 装配用的 ∂u/∂az、∂u/∂el 是由 LOS 方向与竖直轴决定的几何量，所以导引头方位零位与地理北
//    是否重合不影响结果；
//  · 待现场确认：厂家给的是 Euler 角速率还是垂直于视线的横向分量（后者方位通道差 cos(el)）
//    —— 用 UAT3 的 Sr 与 Lp 两列对照即可判定。
Vector3f FD_Attack::source_rate_earth(float r1, float r2) const
{
    // ① 相机系 LOS 方向 → NED 方向
    const float yaw_c_rad = radians(_target_c_deg.x);
    const float pitch_c_rad = radians(_target_c_deg.y);
    const Vector3f u_c(cosf(pitch_c_rad) * cosf(yaw_c_rad),
                       cosf(pitch_c_rad) * sinf(yaw_c_rad),
                       -sinf(pitch_c_rad));

    const Matrix3f rotation_ned_from_camera = rotation_ned_from_body_current() * rotation_camera_to_body;
    const Vector3f u_e = rotation_ned_from_camera * u_c;

    // ② 地理系里的方位/高低角与单位向量导数
    const float az_rad = atan2f(u_e.y, u_e.x);
    const float el_rad = atan2f(-u_e.z, u_e.xy().length());
    const float ca = cosf(az_rad);
    const float sa = sinf(az_rad);
    const float ce = cosf(el_rad);
    const float se = sinf(el_rad);

    const Vector3f du_daz(-ce * sa, ce * ca, 0.0f);     // ∂u/∂az（方位 右+）
    const Vector3f du_del(-se * ca, -se * sa, -ce);     // ∂u/∂el（高低 上+）
    const Vector3f du_e_dt = du_daz * radians(r1) + du_del * radians(r2);

    // ③ 角速度矢量（与 FD_LPF 的 u × ė 同一构造）
    const Vector3f los_rate_rads = u_e % du_e_dt;
    return Vector3f(degrees(los_rate_rads.x),
                    degrees(los_rate_rads.y),
                    degrees(los_rate_rads.z));
}
