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
    AP_GROUPINFO("SRV", 2, FD_CAN, _enable_srv, 1),
    AP_GROUPINFO("MOT", 3, FD_CAN, _enable_mot, 1),

    AP_GROUPEND};

FD_CAN::FD_CAN() {
    AP_Param::setup_object_defaults(this, var_info);

    _batt_ptr = new FD_BATT(this);

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
    // uint32_t last_log_ms = AP_HAL::millis();
    uint32_t last_servo_ms = AP_HAL::millis();
    uint32_t last_mot_ms = AP_HAL::millis();
    uint32_t last_print_ms = AP_HAL::millis();
    bool should_print_servo = false;
    bool should_print_mot = false;

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
            // if (_batt_ptr != nullptr) {
            //     _batt_ptr->handle_info(rxFrame, _print.get());
            // }
            // if (rxFrame.id == 0xFF) {
            //     txFrame.id = 0xEE;
            //     txFrame.data[0] = 0x11;
            //     for (uint8_t i = 0; i<sizeof(txFrame.data); i++) {
            //         // gcs().send_text(MAV_SEVERITY_INFO, "%d, %x", i, rxFrame.data[i]);
            //         txFrame.data[i] = i;
            //     }
            //     txFrame.dlc = 8;//txFrame.dataLengthToDlc(64);
            //     if (write_frame(txFrame, 0)) {
            //         gcs().send_text(MAV_SEVERITY_INFO, "Send %d", sizeof(txFrame.data));
            //     } else {
            //         gcs().send_text(MAV_SEVERITY_INFO, "Send Fail");
            //     }
            // }
        }
        
        if (_print.get()) {
            if (AP_HAL::millis() -  last_print_ms >= 5000) {
                last_print_ms = AP_HAL::millis();
                should_print_servo = true;
                should_print_mot = true;
            }
        }

        if (_enable_srv.get()) {    
            if (AP_HAL::millis() -  last_servo_ms >= 20) {
                last_servo_ms = AP_HAL::millis();
                for (uint8_t i_servo = 1; i_servo <=10; i_servo++) {
                    SRV_Channel *this_channel = SRV_Channels::srv_channel(i_servo-1);
                    if (this_channel == nullptr) {
                        if (should_print_servo) {
                            gcs().send_text(MAV_SEVERITY_INFO, "%d nullptr", i_servo);
                        }
                        continue;
                    }
                    uint16_t pwm = this_channel->get_output_pwm();
                    if (pwm == 0) {
                        pwm = 1500;
                    }
                    float pwm_value = constrain_float((float)pwm, 1000.f, 2000.f);
                    int16_t servo_angle = (pwm_value - 1500.f)*9.f;//+-4500
                    
                    txFrame.id = i_servo;
                    txFrame.data[0] = (uint8_t)(servo_angle&0xFF);
                    txFrame.data[1] = (uint8_t)((servo_angle>>8)&0xFF);
                    txFrame.data[2] = i_servo;
                    txFrame.dlc = 8;
                    if (write_frame(txFrame, 0)) {
                        if (should_print_servo) {
                            gcs().send_text(MAV_SEVERITY_INFO, "Send %x- %d", (uint16_t)txFrame.id, servo_angle);
                        }
                    } else {
                        gcs().send_text(MAV_SEVERITY_INFO, "Send %x- %d Fail", (uint16_t)txFrame.id, servo_angle);
                    }
                }
                should_print_servo = false;
            }
        }

        if (_enable_mot.get()) {    
            if (AP_HAL::millis() -  last_mot_ms >= 20) {
                last_mot_ms = AP_HAL::millis();

                uint16_t thr_left = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleLeft)*10.f;//100
                uint16_t thr_right = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleRight)*10.f;

                txFrame.id = 0x11;
                txFrame.data[0] = (uint8_t)(thr_left&0xFF);
                txFrame.data[1] = (uint8_t)((thr_left>>8)&0xFF);
                txFrame.dlc = 8;
                if (write_frame(txFrame, 0)) {
                    if (should_print_mot) {
                        gcs().send_text(MAV_SEVERITY_INFO, "Send %x- %d", (uint16_t)txFrame.id, thr_left);
                    }
                } else {
                    // gcs().send_text(MAV_SEVERITY_INFO, "Send Fail");
                }

                txFrame.id = 0x12;
                txFrame.data[0] = (uint8_t)(thr_left&0xFF);
                txFrame.data[1] = (uint8_t)((thr_left>>8)&0xFF);
                txFrame.dlc = 8;
                if (write_frame(txFrame, 0)) {
                    if (should_print_mot) {
                        gcs().send_text(MAV_SEVERITY_INFO, "Send %x- %d", (uint16_t)txFrame.id, thr_left);
                    }
                } else {
                    // gcs().send_text(MAV_SEVERITY_INFO, "Send Fail");
                }

                txFrame.id = 0x13;
                txFrame.data[0] = (uint8_t)(thr_right&0xFF);
                txFrame.data[1] = (uint8_t)((thr_right>>8)&0xFF);
                txFrame.dlc = 8;
                if (write_frame(txFrame, 0)) {
                    if (should_print_mot) {
                        gcs().send_text(MAV_SEVERITY_INFO, "Send %x- %d", (uint16_t)txFrame.id, thr_right);
                    }
                } else {
                    // gcs().send_text(MAV_SEVERITY_INFO, "Send Fail");
                }

                txFrame.id = 0x14;
                txFrame.data[0] = (uint8_t)(thr_right&0xFF);
                txFrame.data[1] = (uint8_t)((thr_right>>8)&0xFF);
                txFrame.dlc = 8;
                if (write_frame(txFrame, 0)) {
                    if (should_print_mot) {
                        gcs().send_text(MAV_SEVERITY_INFO, "Send %x- %d", (uint16_t)txFrame.id, thr_right);
                    }
                } else {
                    // gcs().send_text(MAV_SEVERITY_INFO, "Send Fail");
                }
                should_print_mot = false;
            }
        }



        // // 测试数据，10Hz
        // if (AP_HAL::millis() -  last_log_ms >= 100) {
        //     last_log_ms = AP_HAL::millis();
        //     log_status();
        //     if (_out.get() > 0) {
        //         txFrame.id = 0xEE;
        //         txFrame.data[0] = 0x11;
        //         for (uint8_t i = 0; i<sizeof(txFrame.data); i++) {
        //             // gcs().send_text(MAV_SEVERITY_INFO, "%d, %x", i, rxFrame.data[i]);
        //             txFrame.data[i] = i;
        //         }
        //         txFrame.dlc = 8;//txFrame.dataLengthToDlc(64);
        //         if (write_frame(txFrame, 0)) {
        //             gcs().send_text(MAV_SEVERITY_INFO, "Send %d", (uint16_t)sizeof(txFrame.data));
        //         } else {
        //             gcs().send_text(MAV_SEVERITY_INFO, "Send Fail");
        //         }
        //     }
        // }

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
