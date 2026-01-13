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
    AP_GROUPINFO("PRINT", 1, FD_CAN, _print, 0),

    // No use, reserved
    AP_GROUPINFO("BATT", 2, FD_CAN, _batt_enable, 0),
    AP_GROUPINFO("MOT",  3, FD_CAN, _mot_enable,  0),
    AP_GROUPINFO("BMS",  4, FD_CAN, _bms_enable,  0),

    AP_GROUPEND};

FD_CAN::FD_CAN() {
    AP_Param::setup_object_defaults(this, var_info);

    _batt_ptr = new FD_BATT(this);

    for (uint8_t i_mot = 0; i_mot < FD_CAN_MAX_MOT_NUM; i_mot++)
    {
        _mot_ptr[i_mot] = new FD_MOT(this);
        if (_mot_ptr[i_mot] != nullptr)
        {
            _mot_ptr[i_mot]->set_id(i_mot+1);
        }
    }

    _bms_ptr = new FD_BMS(this);

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
    gcs().send_text(MAV_SEVERITY_INFO, "add_interface");
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
    gcs().send_text(MAV_SEVERITY_INFO, "CAN_FD: starting init\n\r");
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
    gcs().send_text(MAV_SEVERITY_INFO, "CAN_FD: init done\n\r");
}

// loop to send output to CAN devices in background thread
void FD_CAN::loop() {
    AP_HAL::CANFrame txFrame{};
    AP_HAL::CANFrame rxFrame{};

    while (true) {
        if (!_initialized) {
            debug_can(AP_CANManager::LOG_ERROR, "CAN_FD: not initialized\n\r");
            hal.scheduler->delay_microseconds(10000);
            continue;
        }

        while (read_frame(rxFrame, 0)) {
            // gcs().send_text(MAV_SEVERITY_INFO, "rxFrame.id %ld", rxFrame.id);
                // for (uint8_t i = 0; i<sizeof(rxFrame.data); i++) {
                //     gcs().send_text(MAV_SEVERITY_INFO, "%d, %x", i, rxFrame.data[i]);
                // }
            // }
            if (_batt_ptr != nullptr) {
                _batt_ptr->handle_info(rxFrame, _print.get());
            }

            for (uint8_t i_mot = 0; i_mot < FD_CAN_MAX_MOT_NUM; i_mot++)
            {
                if (_mot_ptr[i_mot] != nullptr)
                {
                    _mot_ptr[i_mot]->handle_info(rxFrame, _print.get());    //.调用电机的 handle_info 处理接收帧
                }
            }

            if (_bms_ptr != nullptr) {
                _bms_ptr->handle_info(rxFrame, _print.get());
            }

            for (uint8_t i_mot = 0; i_mot < FD_CAN_MAX_MOT_NUM; i_mot++)
            {
                if (_mot_ptr[i_mot] != nullptr)
                {
                    _mot_ptr[i_mot]->handle_info(rxFrame, _print.get());    // 调用电机的 handle_info 处理接收帧
                }
            }
        }


        if (_mot_enable.get()) {   
            for (uint8_t i_mot = 0; i_mot < FD_CAN_MAX_MOT_NUM; i_mot++) { 
                uint16_t mot_output = 0;
                if (SRV_Channels::get_output_pwm(SRV_Channel::k_motor1, mot_output)) {
                    // gcs().send_text(MAV_SEVERITY_INFO, "motor 1 : %d",mot_output);
                    ;
                }
                _mot_ptr[i_mot]->set_pwm(mot_output);
                _mot_ptr[i_mot]->update();
            }
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

    return (_can_iface->send(out_frame, timeout, 0));
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
    if (_batt_ptr == nullptr) {return;}
    AP::logger().WriteStreaming("HBA1","TimeUS,vfc,vout,I,T1,T2,P,PWM1,PWM2",
                                "s--------",
                                "F--------",
                                "Qffffffff",
                                AP_HAL::micros64(),
                                (float)_batt_ptr->status.vfc,
                                (float)_batt_ptr->status.vout,
                                (float)_batt_ptr->status.I,
                                (float)_batt_ptr->status.T1,
                                (float)_batt_ptr->status.T2,
                                (float)_batt_ptr->status.P,
                                (float)_batt_ptr->status.PWM1,
                                (float)_batt_ptr->status.PWM2);
    AP::logger().WriteStreaming("HBA2","TimeUS,vli,vhy,vbus,power,HPWM1,HPWM2,error,run",
                                "s--------",
                                "F--------",
                                "Qffffffff",
                                AP_HAL::micros64(),
                                (float)_batt_ptr->status.vli,
                                (float)_batt_ptr->status.vhy,
                                (float)_batt_ptr->status.vbus,
                                (float)_batt_ptr->status.power,
                                (float)_batt_ptr->status.HPWM1,
                                (float)_batt_ptr->status.HPWM2,
                                (float)_batt_ptr->status.error,
                                (float)_batt_ptr->status.run);
}

bool FD_CAN::pre_arm_check(char *reason, uint8_t reason_len) {
    snprintf(reason, reason_len, "FD CAN");
    return true;
}
