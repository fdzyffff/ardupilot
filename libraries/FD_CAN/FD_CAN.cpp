#include "FD_CAN.h"

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
#include <stdio.h>

extern const AP_HAL::HAL &hal;

#if HAL_CANMANAGER_ENABLED
#define debug_can(level_debug, fmt, args...)                     \
    do {                                                         \
        AP::can().log_text(level_debug, "CAN_FD", fmt, ##args); \
    } while (0)
#else
#define debug_can(level_debug, fmt, args...)
#endif

// table of user-configurable Piccolo CAN bus parameters
const AP_Param::GroupInfo FD_CAN::var_info[] = {

    // No use, reserved
    AP_GROUPINFO("P1", 1, FD_CAN, _p1, 0),

    // No use, reserved
    AP_GROUPINFO("P2", 2, FD_CAN, _p2, 0),

    AP_GROUPEND};

FD_CAN::FD_CAN() {
    AP_Param::setup_object_defaults(this, var_info);

    debug_can(AP_CANManager::LOG_INFO, "CAN_FD: constructed\n\r");
}

FD_CAN *FD_CAN::get_can_fd(uint8_t driver_index) {
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) !=
            AP_CAN::Protocol::FDCAN) {
        return nullptr;
    }

    return static_cast<FD_CAN *>(AP::can().get_driver(driver_index));
}

bool FD_CAN::add_interface(AP_HAL::CANIface *can_iface) {
    if (_can_iface != nullptr) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CAN_FD: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "FDCAN: CAN driver not found\n\r");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "FDCAN: Driver not initialized\n\r");
        return false;
    }

    if (!_can_iface->set_event_handle(&sem_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "FDCAN: Cannot add event handle\n\r");
        return false;
    }
    return true;
}

// initialize CAN_FD bus
void FD_CAN::init(uint8_t driver_index, bool enable_filters) {
    _driver_index = driver_index;

    debug_can(AP_CANManager::LOG_DEBUG, "CAN_FD: starting init\n\r");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "CAN_FD: already initialized\n\r");
        return;
    }

    hal.util->snprintf(_thread_name, sizeof(_thread_name), "FD_CAN_%u", driver_index);
    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&FD_CAN::loop, void), _thread_name, 4096,
            AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CAN_FD: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    debug_can(AP_CANManager::LOG_DEBUG, "CAN_FD: init done\n\r");
}

// loop to send output to CAN devices in background thread
void FD_CAN::loop() {
    AP_HAL::CANFrame txFrame{};
    AP_HAL::CANFrame rxFrame{};
    uint32_t last_log_ms = AP_HAL::millis();

    while (true) {
        if (!_initialized) {
            debug_can(AP_CANManager::LOG_ERROR, "CAN_FD: not initialized\n\r");
            hal.scheduler->delay_microseconds(10000);
            continue;
        }

        while (read_frame(rxFrame, 0)) {
            switch (rxFrame.id) {

            }
        }
            
        // 定时将电调状态存入日志，10Hz
        if (AP_HAL::millis() -  last_log_ms >= 100) {
            last_log_ms = AP_HAL::millis();
            log_status();
        }

        // 1ms loop delay
        hal.scheduler->delay_microseconds(1000);  // 延时1ms，从而此线程以1KHz的频率执行
    }
}

// write frame on CAN bus, returns true on success
bool FD_CAN::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout) {
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CAN_FD: Driver not initialized for write_frame\n\r");
        return false;
    }

    bool read_select = false;
    bool write_select = true;

    bool ret = _can_iface->select(read_select, write_select, &out_frame, timeout);

    if (!ret || !write_select) {
        return false;
    }

    return (_can_iface->send(out_frame, timeout,
                             AP_HAL::CANIface::AbortOnError) == 1);
}

// read frame on CAN bus, returns true on succses
bool FD_CAN::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout) {
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CAN_FD: Driver not initialized for read_frame\n\r");
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

// called from high level code
void FD_CAN::update() {
    ;
}

void FD_CAN::log_status(void) {
    AP::logger().WriteStreaming("TEC3","TimeUS,p1",
                                "Qf",
                                AP_HAL::micros64(),
                                (float)_p1.get());
}

bool FD_CAN::pre_arm_check(char *reason, uint8_t reason_len) {
    snprintf(reason, reason_len, "FD CAN");
    return true;
}
