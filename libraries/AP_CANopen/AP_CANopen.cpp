/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Oliver Walters / Currawong Engineering Pty Ltd
 */

#include "AP_CANopen.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Param/AP_Param.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <stdio.h>
#include <AP_Redundancy/AP_Redundancy_config.h>
#if ENABLE_REDUNDANCY_CONTROL
#include <AP_Redundancy/AP_Redundancy.h>
#endif

// 前向声明
static bool is_redundancy_in_control();

// 舵机是否处于 save 后 flash 写入等待窗口。窗口期间对该舵机不发 SDO 读请求，
// 避免 KST MCU 忙于写 flash 时返回过渡值或丢失请求。使用有符号差值兼容 millis() 回绕。
static inline bool is_servo_flash_writing(const Servo_t &s)
{
    return (s.flash_write_end_ms != 0) &&
           ((int32_t)(AP_HAL::millis() - s.flash_write_end_ms) < 0);
}

// 计算硬件行程目标：统一为对称 ±max(|N|, |P|)，绕开 KST 反向语义下
// "修改负向限位时正向限位跟变"的歧义（对称时两侧数值相等，任何视角下读回都一致）。
// 软件层 update() 仍按 param_neg/param_pos 做非对称 constrain_float，
// 硬件对称只做物理兜底。最终限值钳到 KST 协议 §2.15 规定的 ±100° 物理上限。
static inline void calc_hw_range(float param_neg, float param_pos,
                                 float &hw_neg, float &hw_pos)
{
    float hw_limit = fmaxf(fabsf(param_neg), fabsf(param_pos));
    if (hw_limit > 100.0f) {
        hw_limit = 100.0f;
    }
    hw_neg = -hw_limit;
    hw_pos =  hw_limit;
}

extern const AP_HAL::HAL &hal;

#if HAL_CANMANAGER_ENABLED
#define debug_can(level_debug, fmt, args...)                     \
    do {                                                         \
        AP::can().log_text(level_debug, "CANopen", fmt, ##args); \
    } while (0)
#else
#define debug_can(level_debug, fmt, args...)
#endif

// 每个舵机3个行程参数的宏：AnN(负向行程), AnP(正向行程), AnT(TRIM中位)
// n: 舵机编号(1-16), base: 起始参数索引
#define CO_ANG_PARAMS(n, base) \
    AP_GROUPINFO("A" #n "N", base,       AP_CANopen, _ang_neg[(n)-1], -20), \
    AP_GROUPINFO("A" #n "P", (base) + 1, AP_CANopen, _ang_pos[(n)-1],  20), \
    AP_GROUPINFO("A" #n "T", (base) + 2, AP_CANopen, _ang_trim[(n)-1],  0)

// table of user-configurable CAN bus parameters
const AP_Param::GroupInfo AP_CANopen::var_info[] = {

    // @Param: SRV_BM
    // @DisplayName: Servo channels
    // @Description: Bitmask defining which servo channels are to be transmitted
    // over Piccolo CAN
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5:
    // Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11,
    // 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16
    // @User: Advanced
    AP_GROUPINFO("SRV_BM", 1, AP_CANopen, _srv_bm, 0xFFFF),

    // @Param: SRV_RT
    // @DisplayName: Servo command output rate
    // @Description: Output rate of servo command messages
    // @Units: Hz
    // @User: Advanced
    // @Range: 1 500
    AP_GROUPINFO("SRV_RT", 2, AP_CANopen, _srv_hz, CANOPEN_MSG_RATE_HZ_DEFAULT),

    // @Param: MAV_RT
    // @DisplayName: MAVLink servo status message rate
    // @Description: Rate at which CAN_SERVO_STATUS messages are sent over MAVLink
    // @Units: Hz
    // @User: Advanced
    // @Range: 0 10
    AP_GROUPINFO("MAV_RT", 3, AP_CANopen, _mavlink_rate, 2),

    // 每个舵机的行程参数（单位：度）
    // AnN: 负向行程限位（范围 -100~0，默认 -20）
    // AnP: 正向行程限位（范围 0~100，默认 20）
    // AnT: 中位偏移（范围 -100~100，默认 0）
    // 参数索引 4~51，共48个参数
    CO_ANG_PARAMS(1,  4),   // A1N, A1P, A1T
    CO_ANG_PARAMS(2,  7),   // A2N, A2P, A2T
    CO_ANG_PARAMS(3,  10),  // A3N, A3P, A3T
    CO_ANG_PARAMS(4,  13),  // A4N, A4P, A4T
    CO_ANG_PARAMS(5,  16),  // A5N, A5P, A5T
    CO_ANG_PARAMS(6,  19),  // A6N, A6P, A6T
    CO_ANG_PARAMS(7,  22),  // A7N, A7P, A7T
    CO_ANG_PARAMS(8,  25),  // A8N, A8P, A8T
    CO_ANG_PARAMS(9,  28),  // A9N, A9P, A9T
    CO_ANG_PARAMS(10, 31),  // A10N, A10P, A10T
    CO_ANG_PARAMS(11, 34),  // A11N, A11P, A11T
    CO_ANG_PARAMS(12, 37),  // A12N, A12P, A12T
    CO_ANG_PARAMS(13, 40),  // A13N, A13P, A13T
    CO_ANG_PARAMS(14, 43),  // A14N, A14P, A14T
    CO_ANG_PARAMS(15, 46),  // A15N, A15P, A15T
    CO_ANG_PARAMS(16, 49),  // A16N, A16P, A16T

    AP_GROUPEND};

AP_CANopen::AP_CANopen() {
    AP_Param::setup_object_defaults(this, var_info);

    for (int i = 0; i < CANOPEN_MAX_NUM_SERVO; i++) {
        _servos[i].enabled = (_srv_bm & (1 << i));
        _servos[i].node.node_ID = i + CANOPEN_SERVO_NODE_ID_START;
        _servos[i].node.SDO_idle_semHandle = true;
        _servos[i].last_real_pos_feedback_timestamp_us = 0;
        _servos[i].got_range_min_setting = false;
        _servos[i].got_range_max_setting = false;
        _servos[i].got_enable_byte = false;
        _servos[i].enable_byte = 0;
        _servos[i].was_online = false;
        _servos[i].range_setup_state = RANGE_SETUP_IDLE;
        _servos[i].flash_write_end_ms = 0;
        _servos[i].reverse_clear_retries = 0;
        _servos[i].reverse_clear_failed = false;
        _servos[i].range_min_deg = 0.0f;
        _servos[i].range_max_deg = 0.0f;
        _servos[i].target_angle_deg = 0;
        _servos[i].real_angle_deg = 0;
        _servos[i].real_current_A = 0;
        _servos[i].real_temperature_dc = 0;
    }

    debug_can(AP_CANManager::LOG_INFO, "CANopen: constructed\n\r");
}

AP_CANopen *AP_CANopen::get_canopen(uint8_t driver_index) {
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) !=
            AP_CAN::Protocol::CANopen) {
        return nullptr;
    }

    return static_cast<AP_CANopen *>(AP::can().get_driver(driver_index));
}

bool AP_CANopen::add_interface(AP_HAL::CANIface *can_iface) {
    if (_can_iface != nullptr) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CANopen: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CANopen: CAN driver not found\n\r");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CANopen: Driver not initialized\n\r");
        return false;
    }

    if (!_can_iface->set_event_handle(&_event_handle)) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CANopen: Cannot add event handle\n\r");
        return false;
    }
    return true;
}

// initialize CANopen bus
void AP_CANopen::init(uint8_t driver_index, bool enable_filters) {
    _driver_index = driver_index;

    debug_can(AP_CANManager::LOG_DEBUG, "CANopen: starting init\n\r");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "CANopen: already initialized\n\r");
        return;
    }
    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_CANopen::loop, void), _thread_name, 4096,
            AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CANopen: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    snprintf(_thread_name, sizeof(_thread_name), "CANopen_%u", driver_index);

    debug_can(AP_CANManager::LOG_DEBUG, "CANopen: init done\n\r");
}

// loop to send output to CAN devices in background thread
void AP_CANopen::loop() {
    AP_HAL::CANFrame rxFrame{};

    // 控制帧使用 micros64 高精度定时，查询帧使用 millis 即可
    uint64_t last_servo_tx_us = 0;
    uint32_t last_range_min_ms = 0;
    uint32_t last_range_max_ms = 0;
    uint32_t last_enable_ms = 0;
    uint32_t last_feedback_ms = 0;
    uint32_t last_config_range_ms = 0;
    uint32_t last_query_status_ms = 0;
    uint32_t last_log_ms = 0;
    uint32_t last_mavlink_ms = 0;
    bool _prev_in_control = false;  // 上一次迭代的余度控制状态，用于检测控制权转移

    while (true) {
        if (!_initialized) {
            debug_can(AP_CANManager::LOG_ERROR, "CANopen: not initialized\n\r");
            hal.scheduler->delay_microseconds(10000);
            continue;
        }

        // ========== 接收CAN帧（最高优先级，最先执行，确保FIFO不溢出） ==========
        while (read_frame(rxFrame, 0)) {
            uint8_t node_id = rxFrame.id & 0x7F;
            if ((node_id < CANOPEN_SERVO_NODE_ID_START) ||
                (node_id > CANOPEN_SERVO_NODE_ID_END)) {
                continue;
            }

            uint8_t servo_index = node_id - CANOPEN_SERVO_NODE_ID_START;

            if (rxFrame.dlc == 4) {  // 舵机位置反馈帧
                int16_t angle_deg_x10 = (rxFrame.data[1] << 8) + rxFrame.data[0];
                _servos[servo_index].real_angle_deg = angle_deg_x10 * 0.1f;
                _servos[servo_index].last_real_pos_feedback_timestamp_us = AP_HAL::micros64();
            } else {
                switch (rxFrame.id & 0xFF80) {
                    case 0x580:  // slave回复的SDO
                        process_SDO_reply(rxFrame);
                        break;
                    default:  // RPDO
                        process_RPDO(rxFrame);
                        break;
                }
            }
        }

        uint64_t now_us = AP_HAL::micros64();
        uint32_t now_ms = AP_HAL::millis();

        bool in_control = is_redundancy_in_control();

        // ========== 检测控制权丢失：重置 NMT 标记，以便重新接管时重新评估 ==========
        if (_prev_in_control && !in_control) {
            _nmt_start_sent = false;
        }
        _prev_in_control = in_control;

        // ========== NMT Start：接管时优先检查舵机是否已在线，实现快速接管 ==========
        if (!_nmt_start_sent && in_control) {
            // 检查是否有舵机最近有PDO反馈（说明备份FMU在监听期间舵机一直在线）
            bool any_servo_recently_online = false;
            for (int i = 0; i < CANOPEN_MAX_NUM_SERVO; i++) {
                if (is_servo_channel_active(i) &&
                    _servos[i].last_real_pos_feedback_timestamp_us != 0 &&
                    (now_us - _servos[i].last_real_pos_feedback_timestamp_us) < SERVO_ONLINE_TIMEOUT_US) {
                    any_servo_recently_online = true;
                    break;
                }
            }

            if (any_servo_recently_online) {
                // 快速接管：舵机已在线，跳过NMT Start，直接开始发送控制帧
                _nmt_start_sent = true;
                gcs().send_text(MAV_SEVERITY_INFO, "CANopen: fast takeover, servos already online");
            } else {
                // 正常接管：发送NMT Start，使舵机进入Operational状态
                for (int i = 0; i < CANOPEN_MAX_NUM_SERVO; i++) {
                    if (is_servo_channel_active(i)) {
                        CANopen_set_slave_node_into_Operational_state(&_servos[i].node);
                    }
                }
                _nmt_start_sent = true;
                gcs().send_text(MAV_SEVERITY_INFO, "CANopen: NMT Start sent to all servos");
            }
        }

        // ========== 刷新在线舵机索引表（每 100ms 重建一次） ==========
        if (now_ms - _last_online_rebuild_ms >= 100) {
            _last_online_rebuild_ms = now_ms;
            rebuild_online_list();
        }

        // ========== 定时发送舵机控制帧（每次迭代仅发1帧，均匀散开避免TX邮箱溢出） ==========
        // 每个舵机的目标帧率 = _srv_hz，总帧率 = _srv_hz × 在线数
        // 帧间隔 = 1000000 / (srv_hz × online_count)，每次只发1帧，绝不争抢邮箱
        _srv_hz.set(constrain_int16(_srv_hz, CANOPEN_MSG_RATE_HZ_MIN,
                                    CANOPEN_MSG_RATE_HZ_MAX));
        bool sent_control = false;

        if (_online_count > 0) {
            uint32_t per_frame_interval_us = 1000000U / ((uint32_t)_srv_hz * _online_count);
            // 最小间隔保护：不低于500μs，避免CAN总线过载
            if (per_frame_interval_us < 500) {
                per_frame_interval_us = 500;
            }

            if (now_us - last_servo_tx_us >= per_frame_interval_us) {
                // 无漂移累加
                last_servo_tx_us += per_frame_interval_us;
                if (now_us - last_servo_tx_us > per_frame_interval_us) {
                    last_servo_tx_us = now_us;
                }
                send_servo_target_angle();
                sent_control = true;
            }
        }

        // ========== 查询帧仅在未发送控制帧的迭代中执行，避免争抢TX邮箱 ==========
        if (!sent_control) {
            // 读取舵机运动范围最小值
            if (now_ms - last_range_min_ms >= QUERY_SERVO_RANGE_PERIOD_MS) {
                last_range_min_ms = now_ms;
                query_servos_range_min();
            }

            // 读取舵机运动范围最大值
            if (now_ms - last_range_max_ms >= QUERY_SERVO_RANGE_PERIOD_MS) {
                last_range_max_ms = now_ms;
                query_servos_range_max();
            }

            // 读取舵机功能使能字节（含反向位），用于 configure_servos_range 中强制清零
            if (now_ms - last_enable_ms >= QUERY_SERVO_RANGE_PERIOD_MS) {
                last_enable_ms = now_ms;
                query_servos_enable_byte();
            }

            // 定时重新初始化离线舵机
            if (now_ms - last_feedback_ms >= SET_ENABLE_SERVO_FEEDBACK_PERIOD_MS) {
                last_feedback_ms = now_ms;
                enable_servos_feedback();
            }

            // 定时检查并配置舵机行程
            if (now_ms - last_config_range_ms >= CONFIGURE_SERVO_RANGE_PERIOD_MS) {
                last_config_range_ms = now_ms;
                configure_servos_range();
            }

            // 定时查询舵机状态
            if (now_ms - last_query_status_ms >= QUERY_SERVO_STATUS_PERIOD_MS) {
                last_query_status_ms = now_ms;
                query_servos_status();
            }
        }

        // ========== 定时将舵机状态存入日志 ==========
        if (now_ms - last_log_ms >= SERVO_STATUS_LOG_PERIOD_MS) {
            last_log_ms = now_ms;
            log_servos_status();
        }

        // ========== MAVLink舵机状态上报 ==========
#if HAL_GCS_ENABLED
        if (_mavlink_rate > 0) {
            uint16_t mavlink_rate_ms = 1000 / _mavlink_rate;
            if (now_ms - last_mavlink_ms >= mavlink_rate_ms) {
                last_mavlink_ms = now_ms;
                gcs().send_message(MSG_CAN_SERVO_STATUS);
            }
        }
#endif

        // ========== 定时输出发送统计 ==========
        if (_tx_fail_count > 0 && now_ms - _last_stats_ms >= CANOPEN_TX_STATS_PERIOD_MS) {
            gcs().send_text(MAV_SEVERITY_WARNING, "CANopen: TX fail %lu/%lu",
                           (unsigned long)_tx_fail_count, (unsigned long)_tx_total_count);
            _tx_fail_count = 0;
            _tx_total_count = 0;
            _last_stats_ms = now_ms;
        }

        // 刚发完控制帧时跳过等待，立即进入下一次循环（接收+发下一帧）
        // 未发控制帧时正常等待，有CAN帧到达立即唤醒，否则最多等1ms
        if (!sent_control) {
            _event_handle.wait(1000);
        }
    }
}

/// @brief 处理从机发来的SDO返回帧
/// @param frame 返回帧
void AP_CANopen::process_SDO_reply(AP_HAL::CANFrame &frame) {
    uint8_t node_id = frame.id & 0x7F;

    if ((node_id < CANOPEN_SERVO_NODE_ID_START) ||
        (node_id > CANOPEN_SERVO_NODE_ID_END)) {
        return;
    }

    uint8_t servo_index = node_id - CANOPEN_SERVO_NODE_ID_START;

    uint16_t index = (frame.data[2] << 8) + frame.data[1];
    uint8_t subindex = frame.data[3];

    uint8_t cmd_byte = frame.data[0];

    // SDO abort 帧（byte[0]=0x80）：释放信号量，重置行程配置状态机
    if (cmd_byte == 0x80) {
        _servos[servo_index].node.SDO_idle_semHandle = true;
        if (_servos[servo_index].range_setup_state != RANGE_SETUP_IDLE &&
            _servos[servo_index].range_setup_state != RANGE_SETUP_DONE) {
            _servos[servo_index].range_setup_state = RANGE_SETUP_IDLE;
            _servos[servo_index].got_range_min_setting = false;
            _servos[servo_index].got_range_max_setting = false;
            _servos[servo_index].got_enable_byte = false;
            if (_config_target == (int8_t)servo_index) {
                _config_target = -1;
            }
        }
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "CANopen: Servo %u SDO abort idx=0x%04X",
                        servo_index + 1, index);
        return;
    }

    // 写入确认帧（byte[0]=0x60）：推进行程配置状态机，每步推进时重置超时计时器
    if (cmd_byte == 0x60) {
        switch (index) {
            case CANOPEN_INDEX_KST_SERVO_READ_RANGE_MIN:
                if (_servos[servo_index].range_setup_state == RANGE_SETUP_SET_MIN) {
                    _servos[servo_index].range_setup_state = RANGE_SETUP_SET_MAX;
                    _servos[servo_index].range_setup_start_ms = AP_HAL::millis();
                }
                break;
            case CANOPEN_INDEX_KST_SERVO_READ_RANGE_MAX:
                if (_servos[servo_index].range_setup_state == RANGE_SETUP_SET_MAX) {
                    _servos[servo_index].range_setup_state = RANGE_SETUP_SAVE;
                }
                break;
            case CANOPEN_INDEX_KST_SERVO_ENABLE_BYTE:
                if (_servos[servo_index].range_setup_state == RANGE_SETUP_CLEAR_REVERSE) {
                    _servos[servo_index].range_setup_state = RANGE_SETUP_SAVE_REVERSE;
                    _servos[servo_index].range_setup_start_ms = AP_HAL::millis();
                }
                break;
            default:
                break;
        }
        // 释放SDO信号量
        _servos[servo_index].node.SDO_idle_semHandle = true;
        return;
    }

    // 读取响应帧（byte[0]=0x43/0x4B/0x4F）
    switch (index) {
        case CANOPEN_INDEX_KST_SERVO_READ_STATUS:
            if (subindex == CANOPEN_SUBINDEX_KST_SERVO_READ_STATUS) {
                uint16_t current_A_x100 = (frame.data[5] << 8) + frame.data[4];
                _servos[servo_index].real_current_A = current_A_x100 * 0.01f;
                int8_t temperature_dc = frame.data[6];
                _servos[servo_index].real_temperature_dc =
                    (float)temperature_dc;
            }
            break;

        case CANOPEN_INDEX_KST_SERVO_READ_RANGE_MIN:
            if (subindex == CANOPEN_SUBINDEX_KST_SERVO_READ_RANGE_MIN) {
                int16_t angle_deg_x10 = (frame.data[5] << 8) + frame.data[4];
                _servos[servo_index].range_min_deg = angle_deg_x10 * 0.1f;
                _servos[servo_index].got_range_min_setting = true;
            }
            break;

        case CANOPEN_INDEX_KST_SERVO_READ_RANGE_MAX:
            if (subindex == CANOPEN_SUBINDEX_KST_SERVO_READ_RANGE_MAX) {
                int16_t angle_deg_x10 = (frame.data[5] << 8) + frame.data[4];
                _servos[servo_index].range_max_deg = angle_deg_x10 * 0.1f;
                _servos[servo_index].got_range_max_setting = true;
            }
            break;

        case CANOPEN_INDEX_KST_SERVO_ENABLE_BYTE:
            if (subindex == CANOPEN_SUBINDEX_KST_SERVO_ENABLE_BYTE) {
                _servos[servo_index].enable_byte = frame.data[4];
                _servos[servo_index].got_enable_byte = true;
            }
            break;

        default:
            break;
    }
}

/// @brief 处理从机发来的PDO帧
/// @param frame 数据帧
void AP_CANopen::process_RPDO(AP_HAL::CANFrame &frame) {
    // uint8_t node_id = frame.id & 0x7F;
}

// write frame on CAN bus, returns true on success
bool AP_CANopen::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout) {
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CANopen: Driver not initialized for write_frame\n\r");
        return false;
    }

    _tx_total_count++;

    bool read_select = false;
    bool write_select = true;

    bool ret =
        _can_iface->select(read_select, write_select, &out_frame, timeout);

    if (!ret || !write_select) {
        _tx_fail_count++;
        return false;
    }

    // 不使用 AbortOnError，让CAN控制器在总线错误时自动重发
    if (_can_iface->send(out_frame, timeout, 0) != 1) {
        _tx_fail_count++;
        return false;
    }

    return true;
}

// read frame on CAN bus, returns true on succses
bool AP_CANopen::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout) {
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CANopen: Driver not initialized for read_frame\n\r");
        return false;
    }
    bool read_select = true;
    bool write_select = false;
    bool ret = _can_iface->select(read_select, write_select, nullptr, timeout);

    if (!ret || !read_select) {
        // No frame available
        return false;
    }

    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags{};

    return (_can_iface->receive(recv_frame, time, flags) == 1);
}

// called from SRV_Channels
void AP_CANopen::update() {
    /* Read out the servo commands from the channel mixer */
    for (uint8_t ii = 0; ii < CANOPEN_MAX_NUM_SERVO; ii++) {
        if (is_servo_channel_active(ii)) {
            SRV_Channel::Aux_servo_function_t function = SRV_Channels::channel_function(ii);
            float norm = SRV_Channels::get_output_norm(function);

            // get_output_norm(function) 内部会对 PWM 先算 norm 再用首匹配通道的
            // reversed 去反向，返回的是"未反向的逻辑信号"（与 SERVOx_REVERSED 无关）。
            // CANopen 按角度下发，没有 PWM 通路二次处理，必须在这里用本通道自己
            // 的 SERVOx_REVERSED 重新施加一次方向，否则：
            //   1) SERVOx_REVERSED 对 CAN 舵机完全不起作用；
            //   2) 多通道共享同一 function 时（如 SERVO2/SERVO6 同为升降舵）
            //      所有通道永远同向，无法独立反向。
            SRV_Channel *c = SRV_Channels::srv_channel(ii);
            if (c != nullptr && c->get_reversed()) {
                norm = -norm;
            }

            float angn = _ang_neg[ii].get();   // 负向行程限位（度，负值）
            float angp = _ang_pos[ii].get();   // 正向行程限位（度，正值）
            float trim = _ang_trim[ii].get();  // 中位偏移（度）

            // 分段线性映射：norm=0 → trim，norm=+1 → angp，norm=-1 → angn
            // 当 trim 设为行程几何中心时，两侧增益完全对称
            float angle;
            if (norm >= 0) {
                angle = trim + norm * (angp - trim);
            } else {
                angle = trim + norm * (trim - angn);
            }

            // 钳位到行程范围内
            _servos[ii].target_angle_deg = constrain_float(angle, angn, angp);
        }
    }
}

/// @brief 查询舵机的状态，温度、电流（轮询式，每次最多处理 CANOPEN_SERVOS_PER_QUERY 个舵机）
/// @param  无
void AP_CANopen::query_servos_status(void) {
    if (_srv_bm == 0x00) {
        return;
    }

    if (!is_redundancy_in_control()) {
        return;
    }

    uint8_t sent = 0;
    for (uint8_t n = 0; n < CANOPEN_MAX_NUM_SERVO; n++) {
        uint8_t i = _rr_query_idx;
        _rr_query_idx = (_rr_query_idx + 1) % CANOPEN_MAX_NUM_SERVO;
        if (is_servo_online(i)) {
            CANopen_read_by_SDO(&_servos[i].node,
                                CANOPEN_INDEX_KST_SERVO_READ_STATUS,
                                CANOPEN_SUBINDEX_KST_SERVO_READ_STATUS, 0, 4);
            if (++sent >= CANOPEN_SERVOS_PER_QUERY) break;
        }
    }
}

/// @brief 查询舵机的运动范围最小值（轮询式，每次最多处理 CANOPEN_SERVOS_PER_QUERY 个舵机）
/// @param  无
void AP_CANopen::query_servos_range_min(void) {
    if (_srv_bm == 0x00) {
        return;
    }

    if (!is_redundancy_in_control()) {
        return;
    }

    uint8_t sent = 0;
    for (uint8_t n = 0; n < CANOPEN_MAX_NUM_SERVO; n++) {
        uint8_t i = _rr_range_min_idx;
        _rr_range_min_idx = (_rr_range_min_idx + 1) % CANOPEN_MAX_NUM_SERVO;
        if (is_servo_channel_active(i) && !_servos[i].got_range_min_setting &&
            !is_servo_flash_writing(_servos[i])) {
            CANopen_read_by_SDO(&_servos[i].node,
                                CANOPEN_INDEX_KST_SERVO_READ_RANGE_MIN,
                                CANOPEN_SUBINDEX_KST_SERVO_READ_RANGE_MIN, 0, 4);
            if (++sent >= CANOPEN_SERVOS_PER_QUERY) break;
        }
    }
}

/// @brief 查询舵机的运动范围最大值（轮询式，每次最多处理 CANOPEN_SERVOS_PER_QUERY 个舵机）
/// @param  无
void AP_CANopen::query_servos_range_max(void) {
    if (_srv_bm == 0x00) {
        return;
    }

    if (!is_redundancy_in_control()) {
        return;
    }

    uint8_t sent = 0;
    for (uint8_t n = 0; n < CANOPEN_MAX_NUM_SERVO; n++) {
        uint8_t i = _rr_range_max_idx;
        _rr_range_max_idx = (_rr_range_max_idx + 1) % CANOPEN_MAX_NUM_SERVO;
        if (is_servo_channel_active(i) && _servos[i].got_range_min_setting &&
            !_servos[i].got_range_max_setting && !is_servo_flash_writing(_servos[i])) {
            CANopen_read_by_SDO(&_servos[i].node,
                                CANOPEN_INDEX_KST_SERVO_READ_RANGE_MAX,
                                CANOPEN_SUBINDEX_KST_SERVO_READ_RANGE_MAX, 0, 4);
            if (++sent >= CANOPEN_SERVOS_PER_QUERY) break;
        }
    }
}

/// @brief 查询舵机的功能使能字节（含反向位，§2.14 协议），仅在未取到时读取
/// @note  读回的 bit 5 若为 1，则 configure_servos_range() 会触发 CLEAR_REVERSE 流程强制清零
void AP_CANopen::query_servos_enable_byte(void) {
    if (_srv_bm == 0x00) {
        return;
    }

    if (!is_redundancy_in_control()) {
        return;
    }

    uint8_t sent = 0;
    for (uint8_t n = 0; n < CANOPEN_MAX_NUM_SERVO; n++) {
        uint8_t i = _rr_enable_idx;
        _rr_enable_idx = (_rr_enable_idx + 1) % CANOPEN_MAX_NUM_SERVO;
        if (is_servo_channel_active(i) && !_servos[i].got_enable_byte &&
            !is_servo_flash_writing(_servos[i])) {
            CANopen_read_by_SDO(&_servos[i].node,
                                CANOPEN_INDEX_KST_SERVO_ENABLE_BYTE,
                                CANOPEN_SUBINDEX_KST_SERVO_ENABLE_BYTE, 0, 1);
            if (++sent >= CANOPEN_SERVOS_PER_QUERY) break;
        }
    }
}

/// @brief 定时检查离线舵机并重新初始化：发送NMT Start，检测在线→离线切换并重置range查询状态
/// @note  不再依赖 got_range_max_setting，打破了 "NMT→range查询→enable_feedback" 的循环死锁
void AP_CANopen::enable_servos_feedback(void)
{
    if (_srv_bm == 0x00) {
        return;
    }

    if (!is_redundancy_in_control()) {
        return;
    }

    for (int i = 0; i < CANOPEN_MAX_NUM_SERVO; i++) {
        if (!is_servo_channel_active(i)) {
            continue;
        }

        bool online = is_servo_online(i);

        // 检测在线→离线切换：重置range查询状态，使舵机重新走完整初始化流程
        if (_servos[i].was_online && !online) {
            _servos[i].got_range_min_setting = false;
            _servos[i].got_range_max_setting = false;
            _servos[i].got_enable_byte = false;
            _servos[i].range_setup_state = RANGE_SETUP_IDLE;
            // 重连后应重新尝试反向清除（上次失败可能是临时原因，重启舵机后可能恢复）
            _servos[i].reverse_clear_retries = 0;
            _servos[i].reverse_clear_failed = false;
            _servos[i].flash_write_end_ms = 0;
            gcs().send_text(MAV_SEVERITY_WARNING,
                "CANopen: Servo %u offline, will re-init", i + 1);
        }
        _servos[i].was_online = online;

        if (online) {
            continue;
        }

        // 向离线舵机发送NMT Start，使其进入Operational状态并开始反馈位置
        // 如果CAN TX邮箱满，write_frame会返回false，下次周期重试
        CANopen_set_slave_node_into_Operational_state(&_servos[i].node);
    }
}

/// @brief 检查并配置舵机硬件行程：对比参数与硬件读回值，不匹配则写入新行程并保存
/// @note  单目标策略：一次只跟踪一个舵机的配置流程直到完成，避免 round-robin 散开导致超时
///        每步（SET_MIN/SET_MAX）有独立的超时计时，由 process_SDO_reply 推进状态并重置计时器
void AP_CANopen::configure_servos_range(void) {
    if (_srv_bm == 0x00) {
        return;
    }

    if (!is_redundancy_in_control()) {
        return;
    }

    // 如果当前有目标舵机，优先处理它
    if (_config_target >= 0) {
        uint8_t i = (uint8_t)_config_target;

        // 目标舵机已失效（被禁用或配置完成/回到IDLE），释放目标
        if (!is_servo_channel_active(i) ||
            _servos[i].range_setup_state == RANGE_SETUP_DONE ||
            _servos[i].range_setup_state == RANGE_SETUP_IDLE) {
            _config_target = -1;
            // fall through 去扫描下一个需要配置的舵机
        } else {
            // 处理当前目标舵机的配置步骤
            float param_neg = _ang_neg[i].get();
            float param_pos = _ang_pos[i].get();
            // 硬件行程统一写对称 ±max(|N|,|P|)（见 calc_hw_range 注释）
            float hw_neg, hw_pos;
            calc_hw_range(param_neg, param_pos, hw_neg, hw_pos);

            switch (_servos[i].range_setup_state) {
                case RANGE_SETUP_SET_MIN: {
                    if (AP_HAL::millis() - _servos[i].range_setup_start_ms > RANGE_SETUP_STEP_TIMEOUT_MS) {
                        _servos[i].range_setup_state = RANGE_SETUP_IDLE;
                        _servos[i].got_range_min_setting = false;
                        _servos[i].got_range_max_setting = false;
                        _config_target = -1;
                        gcs().send_text(MAV_SEVERITY_WARNING,
                            "CANopen: Servo %u range SET_MIN timeout", i + 1);
                        return;
                    }
                    int16_t val = (int16_t)(hw_neg * 10.0f);
                    uint32_t data = (uint32_t)val & 0x0000FFFF;
                    CANopen_write_by_SDO(&_servos[i].node,
                            CANOPEN_INDEX_KST_SERVO_READ_RANGE_MIN,
                            CANOPEN_SUBINDEX_KST_SERVO_READ_RANGE_MIN,
                            data, true);
                    return;
                }

                case RANGE_SETUP_SET_MAX: {
                    if (AP_HAL::millis() - _servos[i].range_setup_start_ms > RANGE_SETUP_STEP_TIMEOUT_MS) {
                        _servos[i].range_setup_state = RANGE_SETUP_IDLE;
                        _servos[i].got_range_min_setting = false;
                        _servos[i].got_range_max_setting = false;
                        _config_target = -1;
                        gcs().send_text(MAV_SEVERITY_WARNING,
                            "CANopen: Servo %u range SET_MAX timeout", i + 1);
                        return;
                    }
                    int16_t val = (int16_t)(hw_pos * 10.0f);
                    uint32_t data = (uint32_t)val & 0x0000FFFF;
                    CANopen_write_by_SDO(&_servos[i].node,
                            CANOPEN_INDEX_KST_SERVO_READ_RANGE_MAX,
                            CANOPEN_SUBINDEX_KST_SERVO_READ_RANGE_MAX,
                            data, true);
                    return;
                }

                case RANGE_SETUP_SAVE:
                    CANopen_write_by_SDO(&_servos[i].node,
                        CANOPEN_INDEX_KST_SERVO_SAVE,
                        CANOPEN_SUBINDEX_KST_SERVO_SAVE,
                        CANOPEN_SAVE_MAGIC, false);
                    _servos[i].range_setup_state = RANGE_SETUP_DONE;
                    _servos[i].range_min_deg = hw_neg;
                    _servos[i].range_max_deg = hw_pos;
                    // save 无应答 + KST 内部 flash 写入期间 SDO 不可靠，设等待窗口
                    _servos[i].flash_write_end_ms = AP_HAL::millis() + KST_SAVE_FLASH_WAIT_MS;
                    _config_target = -1;
                    gcs().send_text(MAV_SEVERITY_INFO,
                        "CANopen: Servo %u hw range set sym[%.1f, %.1f] (param[%.1f,%.1f]), saved (flash wait %ums)",
                        i + 1, hw_neg, hw_pos, param_neg, param_pos, (unsigned)KST_SAVE_FLASH_WAIT_MS);
                    return;

                case RANGE_SETUP_CLEAR_REVERSE: {
                    if (AP_HAL::millis() - _servos[i].range_setup_start_ms > RANGE_SETUP_STEP_TIMEOUT_MS) {
                        _servos[i].range_setup_state = RANGE_SETUP_IDLE;
                        _servos[i].got_enable_byte = false;
                        _config_target = -1;
                        gcs().send_text(MAV_SEVERITY_WARNING,
                            "CANopen: Servo %u clear reverse timeout", i + 1);
                        return;
                    }
                    uint8_t new_byte = _servos[i].enable_byte & ~CANOPEN_KST_ENABLE_BIT_REVERSE;
                    uint32_t data = (uint32_t)new_byte & 0xFFu;
                    CANopen_write_by_SDO(&_servos[i].node,
                            CANOPEN_INDEX_KST_SERVO_ENABLE_BYTE,
                            CANOPEN_SUBINDEX_KST_SERVO_ENABLE_BYTE,
                            data, true);
                    return;
                }

                case RANGE_SETUP_SAVE_REVERSE:
                    // 持久化反向清除：save 命令无需回复（协议 §2.10）
                    CANopen_write_by_SDO(&_servos[i].node,
                        CANOPEN_INDEX_KST_SERVO_SAVE,
                        CANOPEN_SUBINDEX_KST_SERVO_SAVE,
                        CANOPEN_SAVE_MAGIC, false);
                    // 不再乐观写本地 enable_byte——清掉 got_enable_byte 强制下一轮重读验证 bit5
                    // 真的被清零（协议 §2.14 改写后立刻生效，但 KST 实测可能存在写失败/flash 未固化
                    // 重启回滚等情况）
                    _servos[i].got_enable_byte = false;
                    // 反向语义变化后，此前读到的 0x300A/0x300B 可能在旧约定下，强制重读
                    _servos[i].got_range_min_setting = false;
                    _servos[i].got_range_max_setting = false;
                    // save 无应答 + KST 内部 flash 写入期间 SDO 不可靠，设等待窗口
                    _servos[i].flash_write_end_ms = AP_HAL::millis() + KST_SAVE_FLASH_WAIT_MS;
                    _servos[i].range_setup_state = RANGE_SETUP_IDLE;
                    _config_target = -1;
                    gcs().send_text(MAV_SEVERITY_INFO,
                        "CANopen: Servo %u reverse clear sent, verifying after flash wait",
                        i + 1);
                    return;

                default:
                    _servos[i].range_setup_state = RANGE_SETUP_IDLE;
                    _config_target = -1;
                    return;
            }
        }
    }

    // 无当前目标，扫描下一个需要配置的舵机
    for (uint8_t n = 0; n < CANOPEN_MAX_NUM_SERVO; n++) {
        uint8_t i = _rr_config_idx;
        _rr_config_idx = (i + 1) % CANOPEN_MAX_NUM_SERVO;

        if (!is_servo_channel_active(i)) {
            continue;
        }

        float param_neg = _ang_neg[i].get();
        float param_pos = _ang_pos[i].get();
        // 硬件行程期望值：对称 ±max(|N|,|P|)，硬件读回与之对比
        float hw_neg, hw_pos;
        calc_hw_range(param_neg, param_pos, hw_neg, hw_pos);

        if (_servos[i].range_setup_state == RANGE_SETUP_IDLE) {
            // flash 写入窗口期跳过本舵机（save 后等 KST 内部 flash 写完再动）
            if (is_servo_flash_writing(_servos[i])) {
                continue;
            }
            // 前置：必须先取到功能使能字节，否则无法判断反向状态
            if (!_servos[i].got_enable_byte) {
                continue;
            }
            // 优先清除 KST 内部反向位：反向语义下 0x300A/0x300B 的约定与飞控假设不一致，
            // 必须先清零再处理行程；用户如需反向请使用 ArduPilot 的 SERVOx_REVERSED
            if (_servos[i].enable_byte & CANOPEN_KST_ENABLE_BIT_REVERSE) {
                // 前次已判定失败则不再重试，等在线→离线重连再试
                if (_servos[i].reverse_clear_failed) {
                    continue;
                }
                // 读回仍是 1：要么写未生效，要么重启后回滚。超上限则放弃并拒绝 pre-arm
                if (_servos[i].reverse_clear_retries >= KST_REVERSE_CLEAR_MAX_RETRIES) {
                    _servos[i].reverse_clear_failed = true;
                    gcs().send_text(MAV_SEVERITY_CRITICAL,
                        "CANopen: Servo %u reverse clear FAILED after %u retries, pre-arm blocked",
                        i + 1, (unsigned)KST_REVERSE_CLEAR_MAX_RETRIES);
                    continue;
                }
                _servos[i].reverse_clear_retries++;
                _servos[i].range_setup_state = RANGE_SETUP_CLEAR_REVERSE;
                _servos[i].range_setup_start_ms = AP_HAL::millis();
                _config_target = (int8_t)i;
                gcs().send_text(MAV_SEVERITY_INFO,
                    "CANopen: Servo %u internal reverse detected (enable=0x%02X), clear attempt %u/%u",
                    i + 1, _servos[i].enable_byte,
                    (unsigned)_servos[i].reverse_clear_retries,
                    (unsigned)KST_REVERSE_CLEAR_MAX_RETRIES);
                return;
            }
            // 读回 bit5=0：验证成功，重置计数供后续可能的误触发重新计数
            _servos[i].reverse_clear_retries = 0;
            if (!_servos[i].got_range_min_setting || !_servos[i].got_range_max_setting) {
                continue;
            }
            // 与期望的硬件对称值对比，不匹配则触发重写（容差 0.15°）
            if (fabsf(_servos[i].range_min_deg - hw_neg) > 0.15f ||
                fabsf(_servos[i].range_max_deg - hw_pos) > 0.15f) {
                _servos[i].range_setup_state = RANGE_SETUP_SET_MIN;
                _servos[i].range_setup_start_ms = AP_HAL::millis();
                _config_target = (int8_t)i;
                gcs().send_text(MAV_SEVERITY_INFO,
                    "CANopen: Servo %u range mismatch hw[%.1f,%.1f] expect sym[%.1f,%.1f] (param[%.1f,%.1f])",
                    i + 1, _servos[i].range_min_deg, _servos[i].range_max_deg,
                    hw_neg, hw_pos, param_neg, param_pos);
                return;
            }
            _servos[i].range_setup_state = RANGE_SETUP_DONE;
            continue;
        }

        if (_servos[i].range_setup_state == RANGE_SETUP_DONE) {
            // 检查参数是否在运行中被修改（仍以硬件对称值为基准）
            if (fabsf(_servos[i].range_min_deg - hw_neg) > 0.15f ||
                fabsf(_servos[i].range_max_deg - hw_pos) > 0.15f) {
                _servos[i].range_setup_state = RANGE_SETUP_SET_MIN;
                _servos[i].range_setup_start_ms = AP_HAL::millis();
                _config_target = (int8_t)i;
                gcs().send_text(MAV_SEVERITY_INFO,
                    "CANopen: Servo %u param changed, reconfiguring range", i + 1);
                return;
            }
            continue;
        }
    }
}

/// @brief 保存各个舵机状态到日志中，应该以10Hz的频率调用
/// @param  无
void AP_CANopen::log_servos_status(void) {
    AP_Logger *logger = AP_Logger::get_singleton();
    uint64_t timestamp = AP_HAL::micros64();

    if (logger && logger->logging_enabled()) {
        for (int i = 0; i < CANOPEN_MAX_NUM_SERVO; i++) {
            if (is_servo_channel_active(i)) {
                logger->Write_CAN_ServoStatus(
                    timestamp,
                    i,
                    _servos[i].real_angle_deg,
                    _servos[i].target_angle_deg,
                    _servos[i].real_current_A*10,
                    _servos[i].real_temperature_dc);
            }
        }
    }
}

/// @brief 检查当前余度是否处于控制状态
/// @return true表示当前余度处于控制状态，应该发送控制帧；false表示不应该发送控制帧
static bool is_redundancy_in_control()
{
#if ENABLE_REDUNDANCY_CONTROL
    auto *red = AP_Redundancy::get_singleton();
    return red ? red->is_in_control() : false;
#else
    return true;
#endif
}

/// @brief 重建在线舵机索引表，确保控制帧在在线舵机间均匀分配
void AP_CANopen::rebuild_online_list(void) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < CANOPEN_MAX_NUM_SERVO; i++) {
        if (is_servo_online(i)) {
            _online_list[count++] = i;
        }
    }
    _online_count = count;
    // 如果发送位置超出新列表，重置
    if (_online_send_pos >= _online_count) {
        _online_send_pos = 0;
    }
}

/// @brief 通过CAN总线发送舵机的角度控制信息（每次仅发1帧，由调用频率保证总帧率）
/// @note  使用预建的在线索引表顺序轮询，每次1帧 → 1个TX邮箱 → 不会溢出
///        如果目标舵机有 SDO 事务进行中（如行程配置），跳过该舵机避免并发 SDO 冲突
void AP_CANopen::send_servo_target_angle() {
    if (_srv_bm == 0x00 || _online_count == 0) {
        return;
    }

    if (!is_redundancy_in_control()) {
        return;
    }

    // 遍历在线列表，跳过 SDO 忙的舵机
    for (uint8_t attempt = 0; attempt < _online_count; attempt++) {
        uint8_t i = _online_list[_online_send_pos];
        _online_send_pos = (_online_send_pos + 1) % _online_count;

        // 如果该舵机有 SDO 事务进行中（信号量被占用且未超时），跳过
        if (!_servos[i].node.SDO_idle_semHandle &&
            AP_HAL::micros64() < _servos[i].node.SDO_timeout_time_us) {
            continue;
        }

        uint32_t data = (uint32_t)((int16_t)(_servos[i].target_angle_deg * 10.0f)) & 0x0000FFFF;
        CANopen_write_by_SDO(
            &_servos[i].node,
            CANOPEN_INDEX_KST_SERVO_SET_TARGET_ANGLE,
            CANOPEN_SUBINDEX_KST_SERVO_SET_TARGET_ANGLE,
            data,
            false);
        return;
    }
}

bool AP_CANopen::handle_servo_message(AP_HAL::CANFrame &frame) {
    return true;
}

/// @brief 查看此舵机是否被使能
/// @param chan 舵机编号，0~15对应全部参数表中的SERVO1~SERVO16
/// @return 是否使能
bool AP_CANopen::is_servo_channel_active(uint8_t chan) {
    // 首先检查此舵机是否是通过CAN总线控制的
    if (((_srv_bm >> chan) & 0x01) == 0x00) {
        return false;
    }

    // 取出此舵机在飞机上的功能（副翼、升降舵、方向舵等）
    SRV_Channel::Aux_servo_function_t function =
        SRV_Channels::channel_function(chan);

    // 如果舵机没有设置功能，则立即返回false
    if (function <= SRV_Channel::k_none) {
        return false;
    }

    // 如果这个通道是电机相关的功能，则立即返回false
    if (SRV_Channel::is_motor(function)) {
        return false;
    }

    // 运行到此处，说明此舵机被使能了
    return true;
}

/// @brief 舵机是否在线
/// @param chan 舵机编号，0~15对应全部参数表中的SERVO1~SERVO16
/// @return true表示舵机在线
bool AP_CANopen::is_servo_online(uint8_t chan) {
    if (is_servo_channel_active(chan) == false) {
        return false;
    }

    if (chan >= CANOPEN_MAX_NUM_SERVO) {
        return false;
    }

    // No messages received from this servo
    if (_servos[chan].last_real_pos_feedback_timestamp_us == 0) {
        return false;
    }

    uint64_t now = AP_HAL::micros64();

    if (now > (_servos[chan].last_real_pos_feedback_timestamp_us +
               SERVO_ONLINE_TIMEOUT_US)) {
        return false;
    }

    return true;
}

bool AP_CANopen::pre_arm_check(char *reason, uint8_t reason_len) {
    // 检查舵机是否在线
    for (uint8_t ii = 0; ii < CANOPEN_MAX_NUM_SERVO; ii++) {
        if (is_servo_channel_active(ii)) {
            if (!is_servo_online(ii)) {
                snprintf(reason, reason_len, "Servo %u not detected", ii + 1);
                return false;
            }
            // 反向清除重试耗尽仍失败：KST 内部 0x300F bit5 没法稳定清零，
            // 若放行，SERVOx_REVERSED 与 KST 内部反向双重作用，方向不可预测
            if (_servos[ii].reverse_clear_failed) {
                snprintf(reason, reason_len, "Servo %u reverse clear failed", ii + 1);
                return false;
            }
        }
    }

    return true;
}

void AP_CANopen::CANopen_set_slave_node_into_Operational_state(
    CANopen_slave_node_t *p_node) {
    AP_HAL::CANFrame txFrame{};
    txFrame.id = p_node->node_ID;
    txFrame.dlc = 2;
    txFrame.data[0] = 0x01;
    txFrame.data[1] = 0x00;
    write_frame(txFrame, 0);
}

void AP_CANopen::CANopen_set_slave_node_into_Stop_state(
    CANopen_slave_node_t *p_node) {
    AP_HAL::CANFrame txFrame{};
    txFrame.id = p_node->node_ID;
    txFrame.dlc = 2;
    txFrame.data[0] = 0x02;
    txFrame.data[1] = 0x00;
    write_frame(txFrame, 0);
}

bool AP_CANopen::CANopen_read_by_SDO(CANopen_slave_node_t *p_node,
                                     uint16_t index, uint8_t sub_index,
                                     void *data, uint8_t data_size) {
    if ((p_node->SDO_idle_semHandle == false) &&
        (AP_HAL::micros64() <
         p_node->SDO_timeout_time_us))  // SDO被占用且没有超时
    {
        return false;
    }

    // 占用SDO并计算超时的时间
    p_node->SDO_idle_semHandle = false;
    p_node->SDO_timeout_time_us = AP_HAL::micros64() + CANOPEN_SDO_TIMEOUT_US;

    // 填充CAN帧
    AP_HAL::CANFrame txFrame{};
    txFrame.id = 0x600 + p_node->node_ID;
    txFrame.dlc = 8;
    txFrame.data[0] = 0x40;  // 0x40是SDO读取命令
    txFrame.data[1] = index & 0xFF;
    txFrame.data[2] = (index >> 8) & 0xFF;
    txFrame.data[3] = sub_index;
    txFrame.data[4] = 0x00;
    txFrame.data[5] = 0x00;
    txFrame.data[6] = 0x00;
    txFrame.data[7] = 0x00;

    // 发送出去
    return write_frame(txFrame, AP_HAL::micros64() + 1000ULL);
}

bool AP_CANopen::CANopen_write_by_SDO(CANopen_slave_node_t *p_node,
                                      uint16_t index, uint8_t sub_index,
                                      uint32_t data, bool need_reply) {
    // need_reply=false（控制帧）跳过信号量检查，确保控制帧永不被查询帧阻塞
    if (need_reply) {
        if ((p_node->SDO_idle_semHandle == false) &&
            (AP_HAL::micros64() < p_node->SDO_timeout_time_us))
        {
            return false;  // SDO被占用且没有超时，仅对需要回复的帧生效
        }
        // 占用SDO并计算超时的时间
        p_node->SDO_idle_semHandle = false;
        p_node->SDO_timeout_time_us =
            AP_HAL::micros64() + CANOPEN_SDO_TIMEOUT_US;
    }

    // 填充CAN帧
    AP_HAL::CANFrame txFrame{};
    txFrame.id = 0x600 + p_node->node_ID;
    txFrame.dlc = 8;
    txFrame.data[0] =
        0x22;  // KST舵机的协议并非标准的CANopen协议，此处为0x22，而不是根据数据长度取不同的值
    txFrame.data[1] = index & 0xFF;
    txFrame.data[2] = (index >> 8) & 0xFF;
    txFrame.data[3] = sub_index;
    txFrame.data[4] = data & 0xFF;
    txFrame.data[5] = (data >> 8) & 0xFF;
    txFrame.data[6] = (data >> 16) & 0xFF;
    txFrame.data[7] = (data >> 24) & 0xFF;

    // 每次只发1帧，不存在邮箱争抢，统一使用1ms超时
    return write_frame(txFrame, AP_HAL::micros64() + 1000ULL);
}


