#include "FD_Target_DYT2.h"

// 通讯频率修改指令的 B 字段指令码（协议固定值 11H=0x11=17）
static const uint8_t FCMD_SET_REPORT_RATE = 0x11;

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

const AP_Param::GroupInfo FD_Target_DYT2::var_info[] = {
    // TOUT 为基类 FD_Target_Cam 的共用成员，由本类注册（AP_Param 嵌套层数限制，见 FD_Target_Cam.h）
    AP_GROUPINFO("TOUT", 0, FD_Target_DYT2, target_timeout, 2000),

    // @Param: CENT
    // @DisplayName: DYT no-target recentering period
    // @Description: 无目标（含收不到导引头状态帧）时按此周期重发"归中+跟踪模式-自动"
    // @Units: ms
    // @Range: 1000 30000
    // @Increment: 500
    // @User: Advanced
    AP_GROUPINFO("CENT", 1, FD_Target_DYT2, center_time, 5000),

    // @Param: ANG
    // @DisplayName: DYT recentering angle threshold
    // @Description: 无目标且框架高低/方位角绝对值超过该角度时立即发一次"归中+跟踪模式-自动"（1s 内不重复）
    // @Units: deg
    // @Range: 1 90
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ANG", 2, FD_Target_DYT2, center_angle_deg, 30.0f),

    // @Param: STIME
    // @DisplayName: DYT vehicle state frame period
    // @Description: 向导引头发送飞机状态帧(0x03)的周期
    // @Units: ms
    // @Range: 10 200
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("STIME", 3, FD_Target_DYT2, state_time, 20),

    // @Param: RATE
    // @DisplayName: DYT report rate level
    // @Description: 上电时下发的导引头上报频率档位: 1=10Hz 2=25Hz 3=50Hz 4=75Hz 5=100Hz
    // @Range: 1 5
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RATE", 4, FD_Target_DYT2, report_rate, 5),

    AP_GROUPEND
};

FD_Target_DYT2::FD_Target_DYT2() :
    last_telem_ms(0),
    last_send_ms(0),
    last_state_ms(0),
    last_log_ms(0),
    cfg_last_ms(0),
    cfg_send_count(0),
    last_fps_ms(0),
    frame_cnt(0),
    fps(0.0f),
    center_sent_ms(0),
    track_pending(false)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool FD_Target_DYT2::init()
{
    _last_ms = 0;
    _valid = false;
    _new_data = false;
    set_type(0);

    last_telem_ms = 0;
    last_send_ms = 0;
    last_state_ms = 0;
    last_log_ms = 0;
    cfg_last_ms = 0;
    cfg_send_count = 0;
    last_fps_ms = 0;
    frame_cnt = 0;
    fps = 0.0f;
    center_sent_ms = 0;
    track_pending = false;

    if (!init_port()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "UAttack DYT2 UART unavailable");
        return false;
    }

    gcs().send_text(MAV_SEVERITY_WARNING, "UAttack DYT2 UART initialized");
    return true;
}

void FD_Target_DYT2::update()
{
    if (get_port() == nullptr) {
        return;
    }

    while (get_port()->available() > 0) {
        const int16_t read_value = get_port()->read();
        if (read_value < 0) {
            break;
        }
        uart_msg_telem.parse((uint8_t)read_value);
        uart_msg_ack.parse((uint8_t)read_value);
    }

    if (uart_msg_telem._msg_1.updated) {
        uart_msg_telem._msg_1.updated = false;
        handle_telem();
    }
    if (uart_msg_ack._msg_1.updated) {
        uart_msg_ack._msg_1.updated = false;
        handle_ack();
    }

    const uint32_t now_ms = AP_HAL::millis();

    // 帧率统计(1s 窗口；_valid 恒假，基类的 handle_info 帧率统计不参与)
    if (last_fps_ms == 0) {
        last_fps_ms = now_ms;
    } else if (now_ms - last_fps_ms >= 1000U) {
        fps = (float)frame_cnt * 1000.0f / (float)(now_ms - last_fps_ms);
        frame_cnt = 0;
        last_fps_ms = now_ms;
    }

    // 上电配置: 只发导引头上报频率（默认 100Hz）
    run_startup_config(now_ms);

    // 注意: 这里不再用基类 _valid 做判定（失调角不进 LOS，_valid 恒假），
    // 改由状态帧内容 + 状态帧超时判定"是否有目标"。

    // 飞机状态帧: 周期发送（默认 50Hz）
    const uint32_t state_period_ms = (uint32_t)state_time.get();
    if ((state_period_ms > 0) && (now_ms - last_state_ms >= state_period_ms)) {
        send_vehicle_state(now_ms);
        last_state_ms = now_ms;
    }

    // 归中 + 跟踪模式-自动 的两条触发通道：
    //   ① 周期通道: 只在没目标时（未锁定/收不到状态帧）每 UATK_TD_CENT(默认 5s) 一次
    //   ② 事件通道: 框架高低/方位角超 UATK_TD_ANG(默认 30°) 立即发一次（不等 5s 周期——
    //      飞机转弯时角度快速超限）；跟踪态下同样适用（角度还大多半是锁错目标），
    //      1s 内不重复以免归中过程中刷屏
    const bool link_timeout = (target_timeout.get() > 0) &&
                              (now_ms - last_telem_ms > (uint32_t)target_timeout.get());
    const bool locked = target_locked() && !link_timeout;

    const uint32_t period_ms = (uint32_t)center_time.get();
    const bool periodic_due = !locked && (period_ms > 0) &&
                              (now_ms - last_send_ms >= period_ms);

    bool event_due = false;
    if (!link_timeout) {
        const float angle_limit_deg = center_angle_deg.get();
        event_due = ((fabsf(uart_msg_telem.frame_pitch_deg()) > angle_limit_deg) ||
                     (fabsf(uart_msg_telem.frame_yaw_deg()) > angle_limit_deg)) &&
                    (now_ms - last_send_ms >= EVENT_HOLD_MS);
    }

    if (!periodic_due && !event_due) {
        // 归中已发、等 TRACK_DELAY_MS 后补发自动跟踪
        if (track_pending && (now_ms - center_sent_ms >= TRACK_DELAY_MS)) {
            send_auto_track();
            track_pending = false;
        }
        return;
    }

    send_center();
    last_send_ms = now_ms;
    center_sent_ms = now_ms;
    track_pending = true;
}

// 目标有效 = 跟踪态 + 检测到目标（协议: bit0=1 跟踪态 / bit0=1 检测到目标）
bool FD_Target_DYT2::target_locked() const
{
    return uart_msg_telem.tracking() && uart_msg_telem.detected();
}

void FD_Target_DYT2::handle_telem()
{
    last_telem_ms = AP_HAL::millis();
    frame_cnt++;

    // 目标角度 = 框架角 + 失调角：框架角是相对导引头"自身 0 位"的量（不是水平基准），
    // 失调角是目标偏离光轴的角度，两者同属导引头/相机坐标系 → 相加即目标相对 0 位的角度
    // （set_type(0)，与 HY 同口径）
    if (target_locked()) {
        handle_info(uart_msg_telem.frame_yaw_deg() + uart_msg_telem.offset_yaw_deg(),
                    uart_msg_telem.frame_pitch_deg() + uart_msg_telem.offset_pitch_deg());
    } else {
        _valid = false;     // 未锁定/未检测到目标: 本类不作相机源
    }

    // 导引头自带的惯性系 LOS 角速率（协议第 16-19 字节）：方位角速率(右+)、高低角速率(上+)。
    // 与 FD_LPF 链由 LOS 角微分得到的速率同口径（都是惯性量）；角度(框架角+失调角)则是相对
    // 自身 0 位的量 —— 两者参考系不同，由 FD_Attack 在地理系里组向量后旋到相机系消费。
    if (target_locked()) {
        handle_info_rate(uart_msg_telem.los_rate_yaw_dps(), uart_msg_telem.los_rate_pitch_dps());
    }

    update_log();
}

void FD_Target_DYT2::handle_ack()
{
    // 回令只做记录（协议为发送-回令方式，上电只发频率设置）
}

// 上电配置: 只发"通讯频率修改"（默认 100Hz = C 字段 0x0005），发 CFG_SEND_TIMES 次
void FD_Target_DYT2::run_startup_config(uint32_t now_ms)
{
    if (cfg_send_count >= CFG_SEND_TIMES) {
        return;
    }
    if ((cfg_send_count > 0) && (now_ms - cfg_last_ms < CFG_PERIOD_MS)) {
        return;
    }

    const uint8_t level = (uint8_t)constrain_int16((int16_t)report_rate.get(), 1, 5);
    uart_msg_cmd.pack_report_rate(FCMD_SET_REPORT_RATE, level);
    write_bytes(uart_msg_cmd._msg_1.content.data, uart_msg_cmd._msg_1.length);

    cfg_last_ms = now_ms;
    cfg_send_count++;
}

// 飞机状态帧(0x03): 速度/姿态/航向取 AHRS；倾转角无来源置 0；
// 弹目距离(bit0-11)与重新框选次数(bit12-15)无来源置 0
void FD_Target_DYT2::send_vehicle_state(uint32_t now_ms)
{
    Vector3f velocity_ned_ms;
    if (!AP::ahrs().get_velocity_NED(velocity_ned_ms)) {
        velocity_ned_ms.zero();
    }

    FD_msg_DYT2_cmd::State state;
    state.vel_n_ms = velocity_ned_ms.x;
    state.vel_e_ms = velocity_ned_ms.y;
    state.vel_d_ms = velocity_ned_ms.z;
    state.ground_speed_ms = velocity_ned_ms.xy().length();
    state.pitch_deg = degrees(AP::ahrs().get_pitch_rad());
    state.roll_deg = degrees(AP::ahrs().get_roll_rad());
    state.heading_deg = degrees(AP::ahrs().get_yaw_rad());
    state.tilt_deg = 0.0f;
    state.dist_reframe = 0;
    state.timestamp_ms = now_ms;

    uart_msg_cmd.pack_vehicle_state(state);
    write_bytes(uart_msg_cmd._msg_2.content.data, uart_msg_cmd._msg_2.length);
}

// 无目标指令: 归中(D4H) 与 跟踪模式-自动(B4H) 分开发、间隔 TRACK_DELAY_MS
// （协议是发送-回令式，同一毫秒连发两条设备可能只处理一条）
void FD_Target_DYT2::send_center()
{
    uart_msg_cmd.pack_center();
    write_bytes(uart_msg_cmd._msg_1.content.data, uart_msg_cmd._msg_1.length);
}

void FD_Target_DYT2::send_auto_track()
{
    uart_msg_cmd.pack_auto_track();
    write_bytes(uart_msg_cmd._msg_1.content.data, uart_msg_cmd._msg_1.length);
}

void FD_Target_DYT2::update_log()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_log_ms < 100U) {
        return;
    }
    last_log_ms = now_ms;

    // DYT3: 视线角速率(惯性系) + 框架角 + 图像失调角 + 跟踪/检测位
    AP::logger().WriteStreaming(
        "DYT3",
        "TimeUS,LyP,LyY,FrP,FrY,OfP,OfY,Tr,De",
        "skkdddd--",
        "F--------",
        "QffffffBB",
        AP_HAL::micros64(),
        uart_msg_telem.los_rate_pitch_dps(),
        uart_msg_telem.los_rate_yaw_dps(),
        uart_msg_telem.frame_pitch_deg(),
        uart_msg_telem.frame_yaw_deg(),
        uart_msg_telem.offset_pitch_deg(),
        uart_msg_telem.offset_yaw_deg(),
        (uint8_t)(target_locked() ? 1 : 0),
        (uint8_t)(uart_msg_telem.detected() ? 1 : 0));

    // DYT4: 导引头状态（状态码/跟踪/检测/变倍/帧计数/帧率/录像/存储/功率）
    AP::logger().WriteStreaming(
        "DYT4",
        "TimeUS,St,Tk,De,Zoom,FC,Fps,Rec,Stg,Pw",
        "s---------",
        "F---------",
        "QBBBBBfBBB",
        AP_HAL::micros64(),
        uart_msg_telem.dyt_state(),
        (uint8_t)(uart_msg_telem.tracking() ? 1 : 0),
        (uint8_t)(uart_msg_telem.detected() ? 1 : 0),
        uart_msg_telem.zoom(),
        uart_msg_telem.frame_count(),
        fps,
        uart_msg_telem.record(),
        uart_msg_telem.storage_percent(),
        uart_msg_telem.power_state());
}
