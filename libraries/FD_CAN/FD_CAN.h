#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANDriver.h>

#include <AP_Param/AP_Param.h>

#include <FD_CAN/FD_SERVO.h>
#include <FD_CAN/FD_COLLECTOR.h>

#define FD_CAN_MAX_SERVO_NUM 16

class FD_SERVO;
class FD_COLLECTOR;

class FD_CAN : public AP_CANDriver
{
public:
    friend class FD_SERVO;
    friend class FD_COLLECTOR;

    FD_CAN();
    ~FD_CAN();

    /* Do not allow copies */
    FD_CAN(const FD_CAN &other) = delete;
    FD_CAN &operator=(const FD_CAN&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    // Return CAN_ESC from @driver_index or nullptr if it's not ready or doesn't exist
    static FD_CAN *get_can_fd(uint8_t driver_index);

    // initialize CAN_ESC bus
    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // called from high level code
    void update();

    // test if the CAN driver is ready to be armed
    bool pre_arm_check(char* reason, uint8_t reason_len);

    FD_SERVO *_servo_ptr[FD_CAN_MAX_SERVO_NUM];
    FD_COLLECTOR *_collector;

    AP_Int32 _print;       
    AP_Int8 _enable_srv;
    AP_Int8 _enable_mot;   
    AP_Int32 _interval_srv;       
    AP_Int32 _interval_mot;      

private:

    // loop to send output to ESCs in background thread
    void loop();

    // write frame on CAN bus, returns true on success
    bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout);

    // read frame on CAN bus, returns true on succses
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout);

    void log_status(void);

    bool _initialized;
    char _thread_name[16];
    uint8_t _driver_index;
    AP_HAL::CANIface* _can_iface;
    HAL_BinarySemaphore sem_handle;

};
