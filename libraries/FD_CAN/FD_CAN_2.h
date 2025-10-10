#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANDriver.h>

#include <AP_Param/AP_Param.h>

#include <FD_CAN/FD_BMS.h>
#include <FD_CAN/FD_BLOWER.h>
#include <FD_CAN/FD_COLLECTOR.h>

#define FD_CAN_2_MAX_SERVO_NUM 16

class FD_SERVO;
class FD_COLLECTOR;

class FD_CAN_2 : public AP_CANDriver
{
public:
    friend class FD_SERVO;
    friend class FD_COLLECTOR;

    FD_CAN_2();
    ~FD_CAN_2();

    /* Do not allow copies */
    FD_CAN_2(const FD_CAN_2 &other) = delete;
    FD_CAN_2 &operator=(const FD_CAN_2&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    // Return CAN_ESC from @driver_index or nullptr if it's not ready or doesn't exist
    static FD_CAN_2 *get_can_fd(uint8_t driver_index);

    // initialize CAN_ESC bus
    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // called from high level code
    void update();

    // test if the CAN driver is ready to be armed
    bool pre_arm_check(char* reason, uint8_t reason_len);

    FD_BMS *_bms;
    FD_BLOWER *_blower;
    FD_COLLECTOR *_collector;

    AP_Int32 _print;    //.控制是否打印调试信息（0 禁用，非 0 启用）       
    AP_Int8 _enable_srv;    //.控制是否启用舵机（SERVO）控制（1 启用，0 禁用）
    AP_Int8 _enable_mot;    //.控制是否启用电机（MOT）控制（1 启用，0 禁用）
    AP_Int32 _interval_srv; //.舵机控制命令的发送间隔（单位：毫秒，范围 1-1000）
    AP_Int32 _interval_mot; //.电机控制命令的发送间隔（单位：毫秒，范围 1-1000）     

private:

    // loop to send output to ESCs in background thread
    void loop();

    // write frame on CAN bus, returns true on success
    bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout);

    // read frame on CAN bus, returns true on succses
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout);

    void log_status(void);

    bool _initialized;  //.标识 CAN FD 驱动是否已初始化
    char _thread_name[16];
    uint8_t _driver_index;  //.CAN 驱动的索引
    AP_HAL::CANIface* _can_iface;
    HAL_BinarySemaphore sem_handle;

};
