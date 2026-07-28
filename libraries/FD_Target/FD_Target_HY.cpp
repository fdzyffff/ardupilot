#include "FD_Target_HY.h"

#include <AP_Logger/AP_Logger.h>

const AP_Param::GroupInfo FD_Target_HY::var_info[] = {
    AP_GROUPINFO("TOUT", 0, FD_Target_HY, target_timeout, 2000),
    AP_GROUPINFO("CENT", 1, FD_Target_HY, center_time, 3000),
    AP_GROUPINFO("TRKT", 2, FD_Target_HY, track_time, 500),
    AP_GROUPINFO("RESX", 3, FD_Target_HY, res_x, 1920.0f),
    AP_GROUPINFO("RESY", 4, FD_Target_HY, res_y, 1080.0f),
    AP_GROUPINFO("FOVX", 5, FD_Target_HY, fov_x, 60.0f),
    AP_GROUPINFO("FOVY", 6, FD_Target_HY, fov_y, 45.0f),
    AP_GROUPINFO("COEF", 7, FD_Target_HY, pix_coef, 0.03f),
    AP_GROUPINFO("TYPE", 8, FD_Target_HY, target_type, -1),
    AP_GROUPINFO("CONF", 9, FD_Target_HY, min_conf, 50),
    AP_GROUPINFO("RLIM", 10, FD_Target_HY, reacq_limit_deg, 15.0f),
    AP_GROUPEND
};

FD_Target_HY::FD_Target_HY() :
    _port(nullptr)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool FD_Target_HY::init()
{
    _last_ms = 0;
    _valid = false;
    set_type(0);   // 脱靶量已是相机系角度(右正/上正)，UAttack 直接使用

    // HY replaces DYT as the camera target source and uses the first NET port.
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_NET, 0);
    if (_port == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "UAttack HY UART unavailable");
        return false;
    }

    gcs().send_text(MAV_SEVERITY_WARNING, "UAttack HY UART initialized");
    return true;
}

void FD_Target_HY::update()
{
    if (_port == nullptr) {
        return;
    }

    // 1. 字节流喂给三个解析器，各自按 CMD1 过滤
    while (_port->available() > 0) {
        const int16_t read_value = _port->read();
        if (read_value < 0) {
            break;
        }
        uart_msg_HY_miss.parse((uint8_t)read_value);
        uart_msg_HY_ai.parse((uint8_t)read_value);
        uart_msg_HY_hb.parse((uint8_t)read_value);
    }

    if (uart_msg_HY_miss._msg_1.updated) {
        handle_miss_msg();
        uart_msg_HY_miss._msg_1.updated = false;
    }
    if (uart_msg_HY_ai._msg_1.updated) {
        handle_ai_msg();
        uart_msg_HY_ai._msg_1.updated = false;
    }
    if (uart_msg_HY_hb._msg_1.updated) {
        handle_heartbeat_msg();
        uart_msg_HY_hb._msg_1.updated = false;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // 2. 超时判定（以最后一次有效脱靶量为准）
    if ((target_timeout.get() > 0) &&
        (now_ms - _last_ms > (uint32_t)target_timeout.get())) {
        _valid = false;
    }

    // // 3. AI 目标新鲜度（仅用于捕获阶段）
    // if (ai_target_valid && (now_ms - last_ai_ms > 1000U)) {
    //     ai_target_valid = false;
    // }

    // if (!_valid) {
    //     // 4a. 无跟踪：周期重发"开启检测 + 全类型 + 自动锁定"
    //     if (now_ms - last_cmd_ms > (uint32_t)center_time.get()) {
    //         uart_msg_HY_control.pack_open_detection();
    //         send_control();
    //         uart_msg_HY_control.pack_detect_all_types();
    //         send_control();
    //         uart_msg_HY_control.pack_auto_lock();
    //         send_control();
    //         last_cmd_ms = now_ms;
    //     }
    //     // 4b. 若AI报文中有满足条件的目标，发 ID 跟踪加速锁定
    //     if (ai_target_valid &&
    //         (now_ms - last_track_ms > (uint32_t)track_time.get())) {
    //         uart_msg_HY_control.pack_track_id(ai_target_id);
    //         send_control();
    //         last_track_ms = now_ms;
    //     }
    //     return;
    // }

    // // 5. 跟踪中越限：脱靶量过大说明可能已漂移到非期望目标，放弃重新捕获
    // if ((fabsf(miss_yaw_deg) > reacq_limit_deg.get()) ||
    //     (fabsf(miss_pitch_deg) > reacq_limit_deg.get())) {
    //     uart_msg_HY_control.pack_stop_track();
    //     send_control();
    //     _valid = false;
    // }
}

// 00 81 测偏数据(脱靶量)
void FD_Target_HY::handle_miss_msg()
{
    const FD1_msg_HY_miss::MSG_Collection &msg = uart_msg_HY_miss._msg_1.content.msg;

    const bool data_valid   = (msg.status & 0x01U) != 0U;   // 稳定跟踪
    const bool algo_stopped = (msg.status & 0x02U) != 0U;   // 跟踪算法停止
    const bool is_angle     = (msg.status & 0x04U) != 0U;   // 角度(float)输出

    if (!data_valid || algo_stopped) {
        return;   // 记忆跟踪/停止状态不更新，靠超时降级
    }

    float yaw_deg, pitch_deg;
    if (is_angle) {
        yaw_deg = msg.offset_x.f;
        pitch_deg = msg.offset_y.f;
    } else {
        // 像素模式按系数换算；建议跟踪板侧配置 4.4.10 系数后直接输出角度
        yaw_deg = (float)msg.offset_x.i * pix_coef.get();
        pitch_deg = (float)msg.offset_y.i * pix_coef.get();
    }

    miss_yaw_deg = yaw_deg;
    miss_pitch_deg = pitch_deg;

    // 脱靶量约定右正/上正，与 UAttack los_c_deg(x右正, y上正)一致，直送
    handle_info(yaw_deg, pitch_deg);

    update_log_miss((int16_t)(yaw_deg * 100.0f),
                    (int16_t)(pitch_deg * 100.0f),
                    msg.width, msg.height, msg.status);
}

// 00 82 AI目标检测
void FD_Target_HY::handle_ai_msg()
{
    const FD1_msg_HY_ai::MSG_Collection &msg = uart_msg_HY_ai._msg_1.content.msg;

    const uint8_t count = msg.count;
    if (count == 0 || count > 22) {
        return;
    }

    // TODO(完善): total>22 时分包发送、需按 frame_id 拼帧；
    // 当前按各报文独立选最佳目标，分包场景会有偶发目标切换。

    int8_t best_idx = -1;
    uint8_t best_conf = 0;
    for (uint8_t i = 0; i < count; i++) {
        const FD1_msg_HY_ai::HY_ai_target &t = msg.targets[i];
        if (target_type.get() >= 0 && t.type != (uint8_t)target_type.get()) {
            continue;   // 类型过滤（15=质心目标可通过 TYPE=15 选中）
        }
        if (t.conf < (uint8_t)constrain_int16(min_conf.get(), 0, 100)) {
            continue;
        }
        if (best_idx < 0 || t.conf > best_conf) {
            best_idx = (int8_t)i;
            best_conf = t.conf;
        }
    }
    if (best_idx < 0) {
        return;
    }

    const FD1_msg_HY_ai::HY_ai_target &t = msg.targets[best_idx];
    float yaw_deg, pitch_deg;
    if (!pixel_to_angle_deg((float)t.x + 0.5f * (float)t.w,
                            (float)t.y + 0.5f * (float)t.h,
                            yaw_deg, pitch_deg)) {
        return;
    }

    ai_target_valid = true;
    ai_target_id   = t.id;
    ai_target_type = t.type;
    ai_target_conf = t.conf;
    ai_yaw_deg     = yaw_deg;
    ai_pitch_deg   = pitch_deg;
    last_ai_ms     = AP_HAL::millis();

    update_log_ai();
}

// 00 83 心跳
void FD_Target_HY::handle_heartbeat_msg()
{
    const FD1_msg_HY_hb::MSG_Collection &msg = uart_msg_HY_hb._msg_1.content.msg;

    selfcheck_code = msg.code;

    const uint32_t now_ms = AP_HAL::millis();
    if ((msg.code != 0U) && (now_ms - last_selfcheck_warn_ms > 10000U)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "HY selfcheck fault: 0x%lx", (unsigned long)msg.code);
        last_selfcheck_warn_ms = now_ms;
    }
    update_log_heartbeat(msg.count, msg.code);
}

bool FD_Target_HY::pixel_to_angle_deg(float px, float py,
                                      float &yaw_deg, float &pitch_deg) const
{
    const float rx = res_x.get();
    const float ry = res_y.get();
    const float fx = fov_x.get();
    const float fy = fov_y.get();
    if (rx < 1.0f || ry < 1.0f || fx < 1.0f || fy < 1.0f) {
        return false;
    }
    // 针孔模型：等效焦距(像素)由视场角推出，逐轴反正切
    const float f_px_x = 0.5f * rx / tanf(0.5f * radians(fx));
    const float f_px_y = 0.5f * ry / tanf(0.5f * radians(fy));
    yaw_deg   =  degrees(atan2f(px - 0.5f * rx, f_px_x));
    pitch_deg = -degrees(atan2f(py - 0.5f * ry, f_px_y));  // y向下为正 -> 角度上正
    return true;
}

void FD_Target_HY::send_control()
{
    if (_port == nullptr) {
        return;
    }
    _port->write(uart_msg_HY_control._msg_1.content.data,
                 uart_msg_HY_control._msg_1.length);
}

void FD_Target_HY::update_log_miss(int16_t yaw_cdeg, int16_t pitch_cdeg,
                                   uint16_t w, uint16_t h, uint8_t status)
{
    AP::logger().WriteStreaming(
        "HYM1",
        "TimeUS,St,Yaw,Pitch,W,H",
        "s-dd--",
        "F-----",
        "QBcccc",
        AP_HAL::micros64(),
        status,
        yaw_cdeg,
        pitch_cdeg,
        w,
        h);
}

void FD_Target_HY::update_log_ai()
{
    AP::logger().WriteStreaming(
        "HYA1",
        "TimeUS,Id,Typ,Conf,Yaw,Pitch",
        "s---dd",
        "F-----",
        "QBBBff",
        AP_HAL::micros64(),
        ai_target_id,
        ai_target_type,
        ai_target_conf,
        ai_yaw_deg,
        ai_pitch_deg);
}

void FD_Target_HY::update_log_heartbeat(uint16_t count, uint32_t code)
{
    AP::logger().WriteStreaming(
        "HYH1",
        "TimeUS,Cnt,Code",
        "s--",
        "F--",
        "QHI",
        AP_HAL::micros64(),
        count,
        code);
}
