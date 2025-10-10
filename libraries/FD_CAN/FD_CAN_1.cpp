#include "FD_CAN_1.h"
#include <RC_Channel/RC_Channel.h>

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
        AP::can().log_text(level_debug, "CAN_FD_1", fmt, ##args); \
    } while (0)
#else
#define debug_can(level_debug, fmt, args...)
#endif

// table of user-configurable Piccolo CAN bus parameters
const AP_Param::GroupInfo FD_CAN_1::var_info[] = {

    // No use, reserved
    AP_GROUPINFO("PRINT", 1, FD_CAN_1, _print, 0),

    // No use, reserved
    AP_GROUPINFO("SRV", 2, FD_CAN_1, _enable_srv, 1),
    AP_GROUPINFO("MOT", 3, FD_CAN_1, _enable_mot, 1),
    AP_GROUPINFO("MRV", 4, FD_CAN_1, _rev_mot, 0),

    AP_GROUPEND};

FD_CAN_1::FD_CAN_1() {  //.构造函数
    AP_Param::setup_object_defaults(this, var_info);

    for (uint8_t i_servo = 0; i_servo < FD_CAN_1_MAX_SERVO_NUM; i_servo++)
    {
        _servo_ptr[i_servo] = new FD_SERVO(this);
        if (_servo_ptr[i_servo] != nullptr)
        {
            _servo_ptr[i_servo]->set_id(i_servo+1);
        }
    }

    for (uint8_t i_mot = 0; i_mot < FD_CAN_1_MAX_MOT_NUM; i_mot++)
    {
        _mot_ptr[i_mot] = new FD_mot(this);
        if (_mot_ptr[i_mot] != nullptr)
        {
            _mot_ptr[i_mot]->set_id(i_mot+1);
        }
    }

    debug_can(AP_CANManager::LOG_INFO, "CAN_FD_1: constructed\n\r");
}

FD_CAN_1 *FD_CAN_1::get_can_fd(uint8_t driver_index) {  //.从 CAN 管理器中获取指定索引（driver_index）的FD_CAN_1实例
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) !=
            AP_CAN::Protocol::FDCAN) {
        return nullptr;
    }

    return static_cast<FD_CAN_1 *>(AP::can().get_driver(driver_index));
}

bool FD_CAN_1::add_interface(AP_HAL::CANIface *can_iface) {
    gcs().send_text(MAV_SEVERITY_INFO, "add_interface");
    if (_can_iface != nullptr) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CAN_FD_1: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "CAN_FD_1: CAN driver not found\n\r");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "CAN_FD_1: Driver not initialized\n\r");
        return false;
    }

    if (!_can_iface->set_event_handle(&sem_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "CAN_FD_1: Cannot add event handle\n\r");
        return false;
    }
    return true;
}

// initialize CAN_FD_1 bus
void FD_CAN_1::init(uint8_t driver_index, bool enable_filters) {
    gcs().send_text(MAV_SEVERITY_INFO, "CAN_FD_1: starting init\n\r");
    _driver_index = driver_index;

    debug_can(AP_CANManager::LOG_DEBUG, "CAN_FD_1: starting init\n\r");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "CAN_FD_1: already initialized\n\r");
        return;
    }

    hal.util->snprintf(_thread_name, sizeof(_thread_name), "FD_CAN_1_%u", driver_index);
    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&FD_CAN_1::loop, void), _thread_name, 4096,
            AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CAN_FD_1: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    debug_can(AP_CANManager::LOG_DEBUG, "CAN_FD_1: init done\n\r");
    gcs().send_text(MAV_SEVERITY_INFO, "CAN_FD_1: init done\n\r");
}

// loop to send output to CAN devices in background thread
void FD_CAN_1::loop() {
    AP_HAL::CANFrame txFrame{}; //. 发送用的 CAN 帧对象
    AP_HAL::CANFrame rxFrame{};
    // uint32_t last_log_ms = AP_HAL::millis();
    uint32_t last_servo_ms = AP_HAL::millis();  //. 舵机命令上一次发送时间（毫秒）
    uint32_t last_mot_ms = AP_HAL::millis();
    uint32_t last_print_ms = AP_HAL::millis();
    bool should_print_servo = false;
    bool should_print_mot = false;
    uint64_t timeout = AP_HAL::micros64() + 10000ULL;

    while (true) {
        if (!_initialized) {
            debug_can(AP_CANManager::LOG_ERROR, "CAN_FD_1: not initialized\n\r");
            hal.scheduler->delay_microseconds(10000);
            continue;
        }

        while (read_frame(rxFrame, 0)) {    //.循环调用read_frame读取 CAN 帧
            for (uint8_t i_servo = 0; i_servo < FD_CAN_1_MAX_SERVO_NUM; i_servo++)
            {
                if (_servo_ptr[i_servo] != nullptr)
                {
                    _servo_ptr[i_servo]->handle_info(rxFrame, _print.get());    //.调用舵机的 handle_info 处理接收帧
                }
            }
            for (uint8_t i_mot = 0; i_mot < FD_CAN_1_MAX_MOT_NUM; i_mot++)
            {
                if (_mot_ptr[i_mot] != nullptr)
                {
                    _mot_ptr[i_mot]->handle_info(rxFrame, _print.get());    //.调用电机的 handle_info 处理接收帧
                }
            }
        }

        if (_print.get()) {
            if (AP_HAL::millis() -  last_print_ms >= 5000) {    //.通过 last_print_ms 限制打印频率为 5 秒一次
                last_print_ms = AP_HAL::millis();
                should_print_servo = true;
                should_print_mot = true;
            }
        }

        if (_enable_srv.get()) {    //. 若启用舵机控制 
            for (uint8_t i_servo = 0; i_servo <=FD_CAN_1_MAX_SERVO_NUM; i_servo++) {
                SRV_Channel *this_channel = SRV_Channels::srv_channel(i_servo);
                // bool is_flap = false;   //. 标记当前舵机是否为襟翼
                if (this_channel == nullptr) {
                    if (should_print_servo) {
                        gcs().send_text(MAV_SEVERITY_INFO, "%d nullptr", i_servo);
                    }
                    continue;
                }
                // if (this_channel->get_function() == SRV_Channel::Aux_servo_function_t::k_flap) {    //.判断当前通道是否配置为“襟翼”（k_flap是襟翼功能枚举）
                //     is_flap = true;
                // }
                uint16_t pwm = this_channel->get_output_pwm();
                if (pwm == 0) {
                    pwm = 1500;
                }
                float pwm_value = constrain_float((float)pwm, 1000.f, 2000.f);
                int16_t servo_angle = (pwm_value - 1500.f)*12.f;//+-4500

                if (_servo_ptr[i_servo] != nullptr) {
                    // _servo_ptr[i_servo]->enable_brake(is_flap);//. 襟翼舵机启用刹车
                    _servo_ptr[i_servo]->set_pos(servo_angle/100.f);//. 设置舵机目标角度
                }
            }
            should_print_servo = false;

            for (uint8_t i_servo = 0; i_servo < FD_CAN_1_MAX_SERVO_NUM; i_servo++)    //.遍历所有舵机，发送控制命令（将set_pos设置的角度转换为CAN帧）
            {
                if (_servo_ptr[i_servo] != nullptr)
                {
                    _servo_ptr[i_servo]->update();  //.核心：生成CAN帧并调用write_frame发送
                }
            }
        }

        if (_enable_mot.get()) {    
            //-桨距控制，0~65535对应-90°到90°范围桨距角
            float pitch_left = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleLeft)*0.4;
            float pitch_right = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleRight)*0.4;

            if (_mot_ptr[0] != nullptr) {
                _mot_ptr[0]->set_pitch(pitch_left);
                if (should_print_mot) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Send %x- %d", (uint16_t)_mot_ptr[0]->status.id, (uint16_t)pitch_left);
                }
            }
            if (_mot_ptr[1] != nullptr) {
                _mot_ptr[1]->set_pitch(pitch_left);
                if (should_print_mot) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Send %x- %d", (uint16_t)_mot_ptr[1]->status.id, (uint16_t)pitch_left);
                }
            }
            if (_mot_ptr[2] != nullptr) {
                _mot_ptr[2]->set_pitch(pitch_right);
                if (should_print_mot) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Send %x- %d", (uint16_t)_mot_ptr[2]->status.id, (uint16_t)pitch_right);
                }
            }
            if (_mot_ptr[3] != nullptr) {
                _mot_ptr[3]->set_pitch(pitch_right);
                if (should_print_mot) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Send %x- %d", (uint16_t)_mot_ptr[3]->status.id, (uint16_t)pitch_right);
                }
            }

            //-电机转速
            // uint16_t thr = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*10.f;
            int16_t ch6_pwm = 0;
            int16_t thr = 0;
            RC_Channel* ch6 = RC_Channels::rc_channel(5);
            if (ch6 != nullptr) {
                ch6_pwm = ch6->get_radio_in(); //. 返回PWM值（微秒）数据类型为int16_t
            }

            if (ch6_pwm < 1100) {
                thr = 0;
            } else if(ch6_pwm < 1600){
                thr = 500;
            }else{
                thr = 1000;
            }

            for (uint8_t i_mot = 1; i_mot < 5; i_mot++){
                if (_rev_mot & (1<<i_mot)) {
                    thr = -thr;
                }
                _mot_ptr[i_mot]->set_rpm(thr)
            }

            for (uint8_t i_mot = 1; i_mot < 5; i_mot++){
                if (_mot_ptr[i_mot] != nullptr) {
                    _mot_ptr[i_mot]->update();
                }
            }
            
            should_print_mot = false;
        }

        // 1ms loop delay
        hal.scheduler->delay_microseconds(1000);  // 延时1ms，从而此线程以1KHz的频率执行
    }
}

// write frame on CAN bus, returns true on success
bool FD_CAN_1::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout) {
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CAN_FD_1: Driver not initialized for write_frame\n\r");
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
bool FD_CAN_1::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout) {
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR,
                  "CAN_FD_1: Driver not initialized for read_frame\n\r");
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
void FD_CAN_1::update() {
    ;
}

void FD_CAN_1::log_status(void) {
    // if (_batt_ptr == nullptr) {return;}
    // AP::logger().WriteStreaming("HBA1","TimeUS,vfc,vout,I,T1,T2,P,PWM1,PWM2",
    //                             "s--------",
    //                             "F--------",
    //                             "Qffffffff",
    //                             AP_HAL::micros64(),
    //                             (float)_batt_ptr->status.vfc,
    //                             (float)_batt_ptr->status.vout,
    //                             (float)_batt_ptr->status.I,
    //                             (float)_batt_ptr->status.T1,
    //                             (float)_batt_ptr->status.T2,
    //                             (float)_batt_ptr->status.P,
    //                             (float)_batt_ptr->status.PWM1,
    //                             (float)_batt_ptr->status.PWM2);
    // AP::logger().WriteStreaming("HBA2","TimeUS,vli,vhy,vbus,power,HPWM1,HPWM2,error,run",
    //                             "s--------",
    //                             "F--------",
    //                             "Qffffffff",
    //                             AP_HAL::micros64(),
    //                             (float)_batt_ptr->status.vli,
    //                             (float)_batt_ptr->status.vhy,
    //                             (float)_batt_ptr->status.vbus,
    //                             (float)_batt_ptr->status.power,
    //                             (float)_batt_ptr->status.HPWM1,
    //                             (float)_batt_ptr->status.HPWM2,
    //                             (float)_batt_ptr->status.error,
    //                             (float)_batt_ptr->status.run);
}

bool FD_CAN_1::pre_arm_check(char *reason, uint8_t reason_len) {
    snprintf(reason, reason_len, "FD CAN");
    return true;
}
