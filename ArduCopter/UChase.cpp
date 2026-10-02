#include "Copter.h"
#include "UChase.h"
#include <FD_DATA/FD_DATA.h>

#if AP_EXTERNAL_AHRS_ENABLED
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_GPS/AP_GPS.h>
#endif

// HUOJIAN: 地面指控通信层实现。本模块只做取数与指令分发，
// 11050/11051/11052 的组包与发送统一在 libraries/FD_DATA 完成。

void UChase::init()
{
    last_gcs_msg_ms = 0;
    _target_loc_ms = 0;
    _target_loc_valid = false;
    _target_vel_ned_ms.zero();
    _target_heading_deg = 0.0f;
}

// 10Hz 调度：只刷新上报数据缓存，发送由 FD_DATA 按 GCS 流速率完成
void UChase::update()
{
    update_target_timeout();
    fill_report_data();
}

void UChase::fill_report_data()
{
    FD_DATA::chase_status_t status {};
    status.state = get_mission_state();
    status.armed = copter.motors->armed() ? 1 : 0;
    status.gcs_ok = gcs_ok() ? 1 : 0;
    status.hb_ok = copter.uattack.is_cam_online() ? 1 : 0;
    status.bbox_valid = copter.uattack.is_active_cam() ? 1 : 0;
    status.lla_valid = copter.uattack.is_target_loc_valid() ? 1 : 0;
    status.chase_valid = copter.uattack.is_target_loc_valid() ? 1 : 0;
    status.throttle = copter.uattack.get_throttle();
    status.relay_state = relay_state_bitmap();
    AP::fd_data().set_chase_status(status);

    // HUOJIAN: 11052 探测器距离/个数来自两路 lk 探测器；引信状态字待引信驱动接入后填
    FD_DATA::chase_fuze_t fuze {};
    for (uint8_t i = 0; i < UDetector::REPORT_TARGETS; i++) {
        const float dist = copter.udetector.report_distance_m(i);
        switch (i) {
        case 0:
            fuze.det_dist1 = dist;
            break;
        case 1:
            fuze.det_dist2 = dist;
            break;
        case 2:
            fuze.det_dist3 = dist;
            break;
        default:
            fuze.det_dist4 = dist;
            break;
        }
    }
    fuze.det_target_count = copter.udetector.report_count();
    AP::fd_data().set_chase_fuze(fuze);
}

void UChase::msg_received(const mavlink_message_t &msg)
{
    // HUOJIAN: 只把非本机(地面站/其它分系统)报文算作 gcs_ok，避免自己心跳自满足
    if (msg.sysid == gcs().sysid_this_mav()) {
        return;
    }
    last_gcs_msg_ms = AP_HAL::millis();
}

bool UChase::gcs_ok() const
{
    return (last_gcs_msg_ms != 0) && (AP_HAL::millis() - last_gcs_msg_ms < 2000U);
}

// HUOJIAN: 协议任务阶段映射，CHASE 三态/打击/返航/降落分别对应 state
uint8_t UChase::get_mission_state() const
{
    const Mode *mode = copter.flightmode;
    if (mode == nullptr) {
        return 0;
    }

    switch (mode->mode_number()) {
#if MODE_GUIDED_ENABLED
    case Mode::Number::CHASE:
        switch (copter.mode_chase.get_state()) {
        case ModeChase::Chase_State::Takeoff:
            return 4;   // TAKEOFF 爬升中
        case ModeChase::Chase_State::Standby:
            return copter.motors->armed() ? 7 : 2;   // 7=解锁待命 2=待发射
        case ModeChase::Chase_State::Fly:
            return 5;   // NAV_FLIGHT 跟点飞行
        }
        return 0;
#endif
    case Mode::Number::ATTACK:
        return 6;       // TERMINAL 段制导打击
    case Mode::Number::RTL:
        return 9;       // RTL 返航中
    case Mode::Number::LAND:
        return 8;       // STOPPING 软停/降落
    case Mode::Number::GUIDED:
        return 3;       // ARM
    default:
        return copter.motors->armed() ? 10 : 0;   // 10=PILOT_OVERRIDE 人工模式
    }
}

uint8_t UChase::relay_state_bitmap() const
{
    uint8_t bitmap = 0;
#if AP_RELAY_ENABLED
    // HUOJIAN: 本板无独立 IO 通道，RELAY 未配置引脚时读回即为 0
    for (uint8_t i = 0; i < 4; i++) {
        if (AP::relay()->get(i)) {
            bitmap |= (1U << i);
        }
    }
#endif
    return bitmap;
}

// HUOJIAN: 31010 发射——先切 CHASE，再解锁，最后起飞
void UChase::handle_launch()
{
#if MODE_GUIDED_ENABLED
    if (copter.flightmode->mode_number() != Mode::Number::CHASE) {
        if (!copter.set_mode(Mode::Number::CHASE, ModeReason::GCS_COMMAND)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CHASE: mode fail");
            return;
        }
        // HUOJIAN: 进入模式时的判定在 ModeChase::init 里做（地面->起飞态，飞行中->按目标）
    } else {
        // HUOJIAN: 已在 CHASE 时重复收到起飞指令，只重新登记起飞请求
        copter.mode_chase.start_takeoff();
    }

    // HUOJIAN: 安全要求——31010 只切模式 29，绝不自动解锁；
    // 解锁必须由地面站原来的 ARM 指令完成，未解锁时停等待解锁后自动起飞
    if (!copter.motors->armed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CHASE: wait ARM, no auto arm");
    }
    // HUOJIAN: Copter 的解锁状态以 motors 库为准（AP_Arming::armed() 对 Copter 不适用）
    // if (!copter.motors->armed()) {
    //     if (!copter.arming.arm(AP_Arming::Method::MAVLINK)) {
    //         GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CHASE: arm fail");
    //         return;
    //     }
    // }

    copter.mode_chase.start_takeoff();
#else
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CHASE: guided disabled");
#endif
}

// HUOJIAN: 31016 拦截机信息——作为伪 GPS 注入，提供本机位置/速度（GPS 拒止时供 EKF 使用）
void UChase::handle_interceptor_gps(const mavlink_command_int_t &packet)
{
#if AP_EXTERNAL_AHRS_ENABLED
    // HUOJIAN: 协议给的是相对高度，需要高度基准换算成 MSL。
    // GPS 拒止冷启动时 home/origin 都可能没有——此时按基准 0 注入，
    // EKF 会用第一帧注入的位置自行建立原点，局部 NED 与雷达相对高度保持一致
    int32_t base_alt_cm = 0;
    if (AP::ahrs().home_is_set()) {
        base_alt_cm = AP::ahrs().get_home().alt;
    } else {
        Location origin;
        if (AP::ahrs().get_origin(origin)) {
            base_alt_cm = origin.alt;
        }
    }

    AP_ExternalAHRS::gps_data_message_t gps {};
    gps.gps_week = 2046;
    gps.ms_tow = AP_HAL::millis();
    gps.fix_type = AP_GPS_FixType::FIX_3D;
    gps.satellites_in_view = 12;
    // HUOJIAN: 地面雷达精度按米级给，避免 EKF 门限过紧
    gps.horizontal_pos_accuracy = 2.0f;
    gps.vertical_pos_accuracy = 3.0f;
    gps.horizontal_vel_accuracy = 0.5f;
    gps.hdop = 1.0f;
    gps.vdop = 1.5f;
    gps.latitude = packet.x;      // 1e-7 deg
    gps.longitude = packet.y;     // 1e-7 deg
    gps.msl_altitude = base_alt_cm + (int32_t)(packet.z * 100.0f);   // 相对高度按选定的基准转 MSL(cm)
    gps.ned_vel_north = packet.param1;
    gps.ned_vel_east = packet.param2;
    gps.ned_vel_down = 0.0f;

    // HUOJIAN: 转交 LK 后端，由它交给 AP_GPS
    AP::externalAHRS().handle_ext_gps(gps);
    // uint8_t instance = 0;
    // if (AP::gps().get_first_external_instance(instance)) {
    //     AP::gps().handle_external(gps, instance);
    // }
#endif
}

// HUOJIAN: CHASE 协议下行指令分发（31011 靶机信息由 FD_Target_Loc 在 packetReceived 中落数据）
bool UChase::handle_command_int(const mavlink_command_int_t &packet, MAV_RESULT &result)
{
    result = MAV_RESULT_ACCEPTED;

    switch (packet.command) {
    case MAV_CMD_USER_1:    // 31010 发射
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHASE: launch cmd");
        handle_launch();
        return true;

    case MAV_CMD_USER_2:    // 31011 靶机信息(目标位置)
        handle_target_info(packet);
        return true;

    case MAV_CMD_USER_4:    // 31013 打击：直接进打击模式
        if (copter.set_mode(Mode::Number::ATTACK, ModeReason::GCS_COMMAND)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHASE: strike cmd");
        } else {
            result = MAV_RESULT_FAILED;
        }
        return true;

    case MAV_CMD_USER_5:    // 31014 返航
        if (copter.set_mode(Mode::Number::RTL, ModeReason::GCS_COMMAND)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHASE: rtl cmd");
        } else {
            result = MAV_RESULT_FAILED;
        }
        return true;

    case MAV_CMD_USER_6:    // 31015 导引头控制：只接收不执行
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHASE: seeker cmd %.0f recv", (double)packet.param1);
        return true;

    case MAV_CMD_USER_7:    // 31016 拦截机信息
        handle_interceptor_gps(packet);
        return true;

    case MAV_CMD_USER_8:    // 31017 引信控制：机内无该设备，只接收不执行
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHASE: fuze cmd %.0f recv", (double)packet.param1);
        return true;

    case MAV_CMD_USER_9:    // 31018 探测器控制：机内无该设备，只接收不执行
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHASE: det cmd %.0f recv", (double)packet.param1);
        return true;

    case MAV_CMD_DO_SET_RELAY:   // 181 IO 输出控制：本板无独立 IO，只接收不执行
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CHASE: relay cmd recv (no IO)");
        return true;

    default:
        return false;
    }
}

// HUOJIAN: 31011 靶机信息——协议 param5/6=目标经纬度(1e-7deg)、param7=相对 home 高度(m)
void UChase::handle_target_info(const mavlink_command_int_t &packet)
{
    Location temp_loc;
    temp_loc.lat = packet.x;
    temp_loc.lng = packet.y;
    // HUOJIAN: 协议 z 是相对 home 高度，全局目标统一存海拔(MSL)，需要相对高度时由消费侧换算
    int32_t base_alt_cm = 0;
    if (AP::ahrs().home_is_set()) {
        base_alt_cm = AP::ahrs().get_home().alt;
    } else {
        Location origin;
        if (AP::ahrs().get_origin(origin)) {
            base_alt_cm = origin.alt;
        }
    }
    // temp_loc.set_alt_m(packet.z, Location::AltFrame::ABOVE_HOME);   // HUOJIAN: 旧写法保留——存相对高度会与 MSL 的本机位置帧不一致
    temp_loc.set_alt_cm((int32_t)(packet.z * 100.0f) + base_alt_cm, Location::AltFrame::ABSOLUTE);
    _target_loc = temp_loc;
    // HUOJIAN: param1/param2=靶机北/东向速度(m/s)，param3=航向(deg)，协议未给垂速
    _target_vel_ned_ms.x = packet.param1;
    _target_vel_ned_ms.y = packet.param2;
    _target_vel_ned_ms.z = 0.0f;
    _target_heading_deg = packet.param3;
    _target_loc_ms = AP_HAL::millis();
    _target_loc_valid = true;
}

// HUOJIAN: 目标位置超时维护——update() 里按 10Hz 判，超过 TARGET_TIMEOUT_MS 未更新即丢失
void UChase::update_target_timeout()
{
    if (!_target_loc_valid) {
        return;
    }
    if ((AP_HAL::millis() - _target_loc_ms) > TARGET_TIMEOUT_MS) {
        _target_loc_valid = false;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CHASE: target lost");
    }
}
