#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANDriver.h>

#include <AP_Param/AP_Param.h>

#include <FD_CAN/FD_BATT.h>
#include <FD_CAN/FD_BMS.h>
#include <FD_DATA/FD_DATA.h>


class FD_BATT;
class FD_BMS;

class FD_CAN : public AP_CANDriver
{
public:
    friend class FD_BATT;
    friend class FD_BMS;

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

    FD_BATT *_batt_ptr;
    FD_BMS*_bms_ptr;

    AP_Int32 _print;       
    AP_Int8 _batt_enable;  
    AP_Int8 _bms_enable;       

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
