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

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANDriver.h>

#include <AP_Param/AP_Param.h>

#define CANOPEN_MSG_RATE_HZ_DEFAULT 50
#define CANOPEN_MSG_RATE_HZ_MIN 1
#define CANOPEN_MSG_RATE_HZ_MAX 500

#define CANOPEN_SDO_TIMEOUT_US 3000  // SDO超时时间，3000us

#define CANOPEN_MAX_NUM_SERVO 16
#define CANOPEN_SERVO_NODE_ID_START 31  // 舵机节点ID起点，servo1 - 31，servo2 - 32，以此类推（注意是十进制！）
#define CANOPEN_SERVO_NODE_ID_END 46

#define CANOPEN_INDEX_KST_SERVO_SET_TARGET_ANGLE 0x6003
#define CANOPEN_SUBINDEX_KST_SERVO_SET_TARGET_ANGLE 0x00

#define CANOPEN_INDEX_KST_SERVO_READ_STATUS 0x6005
#define CANOPEN_SUBINDEX_KST_SERVO_READ_STATUS 0x00

#define CANOPEN_INDEX_KST_SERVO_READ_RANGE_MIN 0x300A
#define CANOPEN_SUBINDEX_KST_SERVO_READ_RANGE_MIN 0x00

#define CANOPEN_INDEX_KST_SERVO_READ_RANGE_MAX 0x300B
#define CANOPEN_SUBINDEX_KST_SERVO_READ_RANGE_MAX 0x00

#define SERVO_ONLINE_TIMEOUT_US 500000ULL  // 舵机掉线超时时间，此值为500ms，超过此值时间依然没有收到舵机发来的CAN帧，则认为其掉线

#define SERVO_DEFAULT_ANGLE_RANGE 100.0f  // 舵机默认最大角度，从而范围为：±100度
#define SERVO_STATUS_LOG_PERIOD_MS 100  // 舵机状态保存到日志中的周期
#define QUERY_SERVO_STATUS_PERIOD_MS 100  // 定时查询舵机状态（温度、电流）的周期
#define QUERY_SERVO_RANGE_PERIOD_MS 300  // 定时查询舵机运动范围的周期
#define SET_ENABLE_SERVO_FEEDBACK_PERIOD_MS 300  // 定时使能舵机反馈输出的周期
typedef struct _CANopen_slave_node_t
{
  uint8_t node_ID;  // 此从节点在总线上的节点ID
  bool SDO_idle_semHandle;  // 此从节点的SDO服务空闲标志（没有被其他任务占用）
  uint64_t SDO_timeout_time_us;  // SDO占用超时时间
}CANopen_slave_node_t;

typedef struct _Servo_t
{
    bool enabled;  // 飞控参数中是否使能了此舵机
    CANopen_slave_node_t node;  // 此舵机对应的CANopen节点
    uint64_t last_real_pos_feedback_timestamp_us;  // 最后一次收到舵机发来的位置反馈信息的时间
    bool got_range_min_setting;  // 已经取得位置范围最小值的设置值
    bool got_range_max_setting;  // 已经取得位置范围最大值的设置值
    float range_min_deg;  // 位置范围最小值，范围：-100~0°
    float range_max_deg;  // 位置范围最大值，取值范围：0~100°
    float real_angle_deg;  // 舵机反馈过来的当前的实际角度值，单位：度
    float target_angle_deg;  // 此舵机的目标角度值，单位：度
    float real_current_A;  // 舵机反馈过来的当前的实际电流值
    float real_temperature_dc;  // 舵机反馈过来的当前的实际温度值
}Servo_t;

class AP_CANopen : public AP_CANDriver
{
public:
    AP_CANopen();
    ~AP_CANopen();

    /* Do not allow copies */
    AP_CANopen(const AP_CANopen &other) = delete;
    AP_CANopen &operator=(const AP_CANopen&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    // Return CANopen from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_CANopen *get_canopen(uint8_t driver_index);

    // initialize CANopen bus
    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // called from SRV_Channels
    void update();

    // return true if a particular servo is 'active' on the Piccolo interface
    bool is_servo_channel_active(uint8_t chan);

    // return true if a particular servo has been detected on the CAN interface
    bool is_servo_online(uint8_t chan);

    // send servo status messages over MAVLink
    void send_servo_status_mavlink(uint8_t mav_chan);

    // test if the CAN driver is ready to be armed
    bool pre_arm_check(char* reason, uint8_t reason_len);

private:

    // loop to send output to ESCs in background thread
    void loop();

    // write frame on CAN bus, returns true on success
    bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout);

    // read frame on CAN bus, returns true on succses
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout);

    void send_servo_target_angle(void);

    // interpret a servo message received over CAN
    bool handle_servo_message(AP_HAL::CANFrame &frame);

    void CANopen_set_slave_node_into_Operational_state(CANopen_slave_node_t *p_node);
    void CANopen_set_slave_node_into_Stop_state(CANopen_slave_node_t *p_node);
    bool CANopen_read_by_SDO(CANopen_slave_node_t *p_node, uint16_t index,
                             uint8_t sub_index, void *data, uint8_t data_size);
    bool CANopen_write_by_SDO(CANopen_slave_node_t *p_node, uint16_t index,
                             uint8_t sub_index, uint32_t data, bool need_reply);

    void log_servos_status(void);

    void query_servos_status(void);

    void query_servos_range_min(void);

    void query_servos_range_max(void);

    void enable_servos_feedback(void);

    void process_SDO_reply(AP_HAL::CANFrame &frame);  // 处理从机发来的SDO返回帧
    void process_RPDO(AP_HAL::CANFrame &frame);  // 处理从机发来的PDO

    bool _initialized;
    char _thread_name[16];
    uint8_t _driver_index;
    AP_HAL::CANIface* _can_iface;
    HAL_BinarySemaphore _event_handle;

    Servo_t _servos[CANOPEN_MAX_NUM_SERVO];

    AP_Int32 _srv_bm;       //! Servo selection bitmask
    AP_Int16 _srv_hz;       //! Servo update rate (Hz)
    AP_Int8 _mavlink_rate;  //! MAVLink servo status message rate (Hz, default 2Hz)

    HAL_Semaphore _telem_sem;
};

