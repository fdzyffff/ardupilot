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

// 前向声明和包含，用于类型转换调用 is_redundancy_in_control()
#if defined(ENABLE_REDUNDANCY_CONTROL) && ENABLE_REDUNDANCY_CONTROL
// 使用条件编译包含正确的头文件
#if defined(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#include "Plane.h"
#elif defined(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#include "Copter.h"
#endif
#endif

extern const AP_HAL::HAL &hal;

#if HAL_CANMANAGER_ENABLED
#define debug_can(level_debug, fmt, args...)                     \
    do {                                                         \
        AP::can().log_text(level_debug, "CANopen", fmt, ##args); \
    } while (0)
#else
#define debug_can(level_debug, fmt, args...)
#endif

// table of user-configurable Piccolo CAN bus parameters
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
        _servos[i].range_min_deg = -20.0f;
        _servos[i].range_max_deg = 20.0f;
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
    AP_HAL::CANFrame txFrame{};
    AP_HAL::CANFrame rxFrame{};
    uint16_t servo_tx_counter = 0;
    uint16_t log_servos_status_counter = 0;
    uint32_t query_servo_status_counter = 0;
    // uint32_t debug_out_counter = 0;
    uint32_t set_servo_start_feedback_counter = 0;
    uint32_t query_servo_range_min_counter = 0;
    uint32_t query_servo_range_max_counter = 0;
    uint32_t mavlink_send_counter = 0;

    while (true) {
        if (!_initialized) {
            debug_can(AP_CANManager::LOG_ERROR, "CANopen: not initialized\n\r");
            hal.scheduler->delay_microseconds(10000);
            continue;
        }

        // 读取舵机运动范围最小值
        if (query_servo_range_min_counter >= QUERY_SERVO_RANGE_PERIOD_MS) {
            query_servo_range_min_counter = 0;
            query_servos_range_min();
        }
        query_servo_range_min_counter++;

        // 读取舵机运动范围最大值
        if (query_servo_range_max_counter >= QUERY_SERVO_RANGE_PERIOD_MS) {
            query_servo_range_max_counter = 0;
            query_servos_range_max();
        }
        query_servo_range_max_counter++;

        // 设置舵机自动输出当前实时位置
        if (set_servo_start_feedback_counter >= SET_ENABLE_SERVO_FEEDBACK_PERIOD_MS) {
            set_servo_start_feedback_counter = 0;
            enable_servos_feedback();
        }
        set_servo_start_feedback_counter++;

        // 定时发送舵机控制帧
        _srv_hz.set(constrain_int16(_srv_hz, CANOPEN_MSG_RATE_HZ_MIN,
                                    CANOPEN_MSG_RATE_HZ_MAX));
        uint16_t servoCmdRateMs = 1000 / _srv_hz;
        if (servo_tx_counter >= servoCmdRateMs) {
            servo_tx_counter = 0;
            send_servo_target_angle();
        }
        servo_tx_counter++;

        // 定时查询舵机状态
        if (query_servo_status_counter >= QUERY_SERVO_STATUS_PERIOD_MS) {
            query_servo_status_counter = 0;
            query_servos_status();
        }
        query_servo_status_counter++;

        // // 定时输出调试信息
        // if (debug_out_counter >= 1000) {
        //     debug_out_counter = 0;
        //     gcs().send_text(MAV_SEVERITY_EMERGENCY, "min: %.1f",
        //                     _servos[0].range_min_deg);
        // }
        // debug_out_counter++;

        while (read_frame(rxFrame, 0)) {
            uint8_t node_id = rxFrame.id & 0x7F;
            if ((node_id < CANOPEN_SERVO_NODE_ID_START) ||
                (node_id > CANOPEN_SERVO_NODE_ID_END)) {
                continue;
            }

            uint8_t servo_index = node_id - CANOPEN_SERVO_NODE_ID_START;  // 计算舵机编号

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

        // 定时将舵机状态存入日志，10Hz
        if (log_servos_status_counter >= SERVO_STATUS_LOG_PERIOD_MS) {
            log_servos_status_counter = 0;
            log_servos_status();
        }
        log_servos_status_counter++;

        // 定时发送MAVLink消息
#if HAL_GCS_ENABLED
        if (_mavlink_rate > 0) {
            uint16_t mavlink_rate_ms = 1000 / _mavlink_rate;
            if (mavlink_send_counter >= mavlink_rate_ms) {
                mavlink_send_counter = 0;
                // Send to all GCS channels
                for (uint8_t i = 0; i < gcs().num_gcs(); i++) {
                    send_servo_status_mavlink(i);
                }
            }
            mavlink_send_counter++;
        }
#endif

        // 1ms loop delay
        hal.scheduler->delay_microseconds(
            1000);  // 延时1ms，从而此线程以1KHz的频率执行
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

    bool read_select = false;
    bool write_select = true;

    bool ret =
        _can_iface->select(read_select, write_select, &out_frame, timeout);

    if (!ret || !write_select) {
        return false;
    }

    return (_can_iface->send(out_frame, timeout,
                             AP_HAL::CANIface::AbortOnError) == 1);
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
            float servo_angel_range = MIN(fabsf(_servos[ii].range_min_deg), fabsf(_servos[ii].range_max_deg));
            _servos[ii].target_angle_deg = SRV_Channels::get_output_norm(function) * servo_angel_range;
        }
    }
}

/// @brief 查询舵机的状态，温度、电流
/// @param  无
void AP_CANopen::query_servos_status(void) {
    // 如果没有一个舵机使能，则直接退出
    if (_srv_bm == 0x00) {
        return;
    }

    for (int i = 0; i < CANOPEN_MAX_NUM_SERVO; i++) {
        if (is_servo_online(i)) {
            CANopen_read_by_SDO(&_servos[i].node,
                                CANOPEN_INDEX_KST_SERVO_READ_STATUS,
                                CANOPEN_SUBINDEX_KST_SERVO_READ_STATUS, 0, 4);
        }
    }
}

/// @brief 查询舵机的运动范围最小值
/// @param  无
void AP_CANopen::query_servos_range_min(void) {
    // 如果没有一个舵机使能，则直接退出
    if (_srv_bm == 0x00) {
        return;
    }

    for (int i = 0; i < CANOPEN_MAX_NUM_SERVO; i++) {
        if (is_servo_channel_active(i)) {
            if (_servos[i].got_range_min_setting) {  // 已经取得设置值则不再重新查询设置值
                continue;
            }
            
            CANopen_read_by_SDO(&_servos[i].node,
                                CANOPEN_INDEX_KST_SERVO_READ_RANGE_MIN,
                                CANOPEN_SUBINDEX_KST_SERVO_READ_RANGE_MIN, 0, 4);
        }
    }
}

/// @brief 查询舵机的运动范围最大值
/// @param  无
void AP_CANopen::query_servos_range_max(void) {
    // 如果没有一个舵机使能，则直接退出
    if (_srv_bm == 0x00) {
        return;
    }

    for (int i = 0; i < CANOPEN_MAX_NUM_SERVO; i++) {
        if (is_servo_channel_active(i)) {
            if (_servos[i].got_range_min_setting == false) {  // 还未取得位置最小值，则先不取最大值
                continue;
            }

            if (_servos[i].got_range_max_setting) {  // 已经取得设置值则不再重新查询设置值
                continue;
            }
            CANopen_read_by_SDO(&_servos[i].node,
                                CANOPEN_INDEX_KST_SERVO_READ_RANGE_MAX,
                                CANOPEN_SUBINDEX_KST_SERVO_READ_RANGE_MAX, 0, 4);
        }
    }
}

/// @brief 使能舵机位置的反馈输出
/// @param  
void AP_CANopen::enable_servos_feedback(void)
{
    // 如果没有一个舵机使能，则直接退出
    if (_srv_bm == 0x00) {
        return;
    }

    for (int i = 0; i < CANOPEN_MAX_NUM_SERVO; i++) {
        if (_servos[i].got_range_max_setting == false) {  // 还未取得位置最大值，则先不让舵机反馈位置
            continue;
        }

        if (is_servo_online(i)) {  // 舵机已经开始输出位置信息（舵机是否在线是通过判断一定时间内是否收到位置反馈帧来实现）
            continue;
        }

        // 填充CAN帧
        AP_HAL::CANFrame txFrame{};
        txFrame.id = i + CANOPEN_SERVO_NODE_ID_START;
        txFrame.dlc = 2;
        txFrame.data[0] = 0x01;  // 0x40是SDO读取命令
        txFrame.data[1] = 0x00;

        // 发送出去
        write_frame(txFrame, AP_HAL::micros64() + 1000ULL);
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
                    _servos[i].real_current_A,
                    _servos[i].real_temperature_dc);
            }
        }
    }
}

/// @brief 检查当前余度是否处于控制状态
/// @return true表示当前余度处于控制状态，应该发送控制帧；false表示不应该发送控制帧
static bool is_redundancy_in_control()
{
#if defined(ENABLE_REDUNDANCY_CONTROL) && ENABLE_REDUNDANCY_CONTROL
    AP_Vehicle* vehicle = AP::vehicle();
    if (vehicle != nullptr) {
        // 注意：is_redundancy_in_control() 不再是虚函数，需要通过类型转换调用
#if defined(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        Plane* plane_ptr = static_cast<Plane*>(vehicle);
        if (plane_ptr != nullptr) {
            return plane_ptr->is_redundancy_in_control();
        }
#elif defined(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduCopter)
        Copter* copter_ptr = static_cast<Copter*>(vehicle);
        if (copter_ptr != nullptr) {
            return copter_ptr->is_redundancy_in_control();
        }
#endif
    }
#endif
    // 如果三余度控制未启用或无法获取车辆实例，默认允许发送控制帧
    return true;
}

/// @brief 通过CAN总线发送舵机的角度控制信息
void AP_CANopen::send_servo_target_angle() {
    // 如果没有一个舵机使能，则直接退出
    if (_srv_bm == 0x00) {
        return;
    }

    // 对于多旋翼和固定翼固件，如果启动了三余度控制，当当前余度没有处于控制状态时，则不发送控制帧
    if (!is_redundancy_in_control()) {
        return;
    }

    for (int i = 0; i < CANOPEN_MAX_NUM_SERVO; i++) {
        if (is_servo_online(i)) {
            uint32_t data = (uint32_t)((int16_t)(_servos[i].target_angle_deg * 10.0f)) & 0x0000FFFF;
            CANopen_write_by_SDO(
                &_servos[i].node,
                CANOPEN_INDEX_KST_SERVO_SET_TARGET_ANGLE,
                CANOPEN_SUBINDEX_KST_SERVO_SET_TARGET_ANGLE,
                data,
                false);
        }
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
    if ((p_node->SDO_idle_semHandle == false) &&
        (AP_HAL::micros64() <
         p_node->SDO_timeout_time_us))  // SDO被占用且没有超时
    {
        return false;
    }

    if (need_reply) {
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

    // 发送出去
    return write_frame(txFrame, AP_HAL::micros64() + 1000ULL);
}

#if HAL_GCS_ENABLED
// send servo status messages over MAVLink
void AP_CANopen::send_servo_status_mavlink(uint8_t mav_chan)
{
    if (_mavlink_rate <= 0) {
        return;
    }

    // Prepare arrays for all 16 servos
    int16_t target_angle_deg[CANOPEN_MAX_NUM_SERVO];
    int16_t real_angle_deg[CANOPEN_MAX_NUM_SERVO];
    uint16_t real_current_A[CANOPEN_MAX_NUM_SERVO];
    int8_t real_temperature_dc[CANOPEN_MAX_NUM_SERVO];

    WITH_SEMAPHORE(_telem_sem);

    // Fill arrays with servo data
    for (uint8_t i = 0; i < CANOPEN_MAX_NUM_SERVO; i++) {
        if (is_servo_channel_active(i)) {
            // Convert degrees to 0.1 degrees (ddeg)
            target_angle_deg[i] = (int16_t)(_servos[i].target_angle_deg * 10.0f);
            real_angle_deg[i] = (int16_t)(_servos[i].real_angle_deg * 10.0f);
            // Convert Amperes to 0.01 Amperes (cA)
            real_current_A[i] = (uint16_t)(_servos[i].real_current_A * 100.0f);
            // Temperature in degrees Celsius
            real_temperature_dc[i] = (int8_t)_servos[i].real_temperature_dc;
        } else {
            // Fill with zeros for inactive servos
            target_angle_deg[i] = 0;
            real_angle_deg[i] = 0;
            real_current_A[i] = 0;
            real_temperature_dc[i] = 0;
        }
    }

    // Check if we have payload space
    if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t)mav_chan, CAN_SERVO_STATUS)) {
        return;
    }

    // Send MAVLink message
    mavlink_msg_can_servo_status_send((mavlink_channel_t)mav_chan,
                                      target_angle_deg,
                                      real_angle_deg,
                                      real_current_A,
                                      real_temperature_dc);
}
#endif // HAL_GCS_ENABLED

