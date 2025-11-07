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

#include <FD1_DATA/FD1_DATA.h>

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
    AP_GROUPINFO("MRV", 4, FD_CAN_1, _rev_mot, 10),//. 第2个电机和第4个电机反转

    AP_GROUPEND};

FD_CAN_1::FD_CAN_1() {  //.构造函数
    AP_Param::setup_object_defaults(this, var_info);

    for (uint8_t i_servo = 0; i_servo < FD_CAN_1_MAX_SERVO_NUM; i_servo++)
    {
        _servo_ptr[i_servo] = new FD_SERVO(this);
        if (_servo_ptr[i_servo] != nullptr)
        {
            _servo_ptr[i_servo]->set_id(i_servo+1-2); //.+1是因为数组0-12对应servo1-13，-2是因为前两路servo给刹车
        }
    }

    for (uint8_t i_mot = 0; i_mot < FD_CAN_1_MAX_MOT_NUM; i_mot++)
    {
        _mot_ptr[i_mot] = new FD_MOT(this);
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
            AP_CAN::Protocol::FDCAN_1) {
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
    uint32_t last_print_ms = AP_HAL::millis();
    uint32_t last_print_time = AP_HAL::millis();
    bool should_print_servo = false;
    bool should_print_mot = false;

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

        int16_t mot1_rpm = AP::fd1_data().get_mot1_rpm();
        int16_t mot2_rpm = AP::fd1_data().get_mot2_rpm();
        int16_t mot3_rpm = AP::fd1_data().get_mot3_rpm();
        int16_t mot4_rpm = AP::fd1_data().get_mot4_rpm();

        uint16_t mot1_temperature = AP::fd1_data().get_mot1_temperature();
        uint16_t mot2_temperature = AP::fd1_data().get_mot2_temperature();
        uint16_t mot3_temperature = AP::fd1_data().get_mot3_temperature();
        uint16_t mot4_temperature = AP::fd1_data().get_mot4_temperature();

        uint16_t controller1_temperature = AP::fd1_data().get_controller1_temperature();
        uint16_t controller2_temperature = AP::fd1_data().get_controller2_temperature();
        uint16_t controller3_temperature = AP::fd1_data().get_controller3_temperature();
        uint16_t controller4_temperature = AP::fd1_data().get_controller4_temperature();

        float propeller1_angle = AP::fd1_data().get_propeller1_angle();
        float propeller2_angle = AP::fd1_data().get_propeller2_angle();
        float propeller3_angle = AP::fd1_data().get_propeller3_angle();
        float propeller4_angle = AP::fd1_data().get_propeller4_angle();

        uint16_t mot1_error = AP::fd1_data().get_mot1_error();
        uint16_t mot2_error = AP::fd1_data().get_mot2_error();
        uint16_t mot3_error = AP::fd1_data().get_mot3_error();
        uint16_t mot4_error = AP::fd1_data().get_mot4_error();

        uint16_t propeller1_error = AP::fd1_data().get_propeller1_error();
        uint16_t propeller2_error = AP::fd1_data().get_propeller2_error();
        uint16_t propeller3_error = AP::fd1_data().get_propeller3_error();
        uint16_t propeller4_error = AP::fd1_data().get_propeller4_error();

        if (AP_HAL::millis() - last_print_time >= 1000) {
            // gcs().send_text(MAV_SEVERITY_INFO, "P%04d%04d%04d%04d", mot1_rpm, mot2_rpm, mot3_rpm, mot4_rpm);
            // gcs().send_text(MAV_SEVERITY_INFO, "%03d%03d%03d%03d", mot1_temperature, mot2_temperature, mot3_temperature, mot4_temperature);
            // gcs().send_text(MAV_SEVERITY_INFO, "%02d%02d%02d%02d", controller1_temperature, controller2_temperature, controller3_temperature, controller4_temperature);
            // gcs().send_text(MAV_SEVERITY_INFO, "%03ld%03ld%03ld%03ld", (int32_t)(propeller1_angle*10), (int32_t)(propeller2_angle*10),(int32_t) (propeller3_angle*10), (int32_t)(propeller4_angle)*10);
            // if (mot1_error != 0){
            //     gcs().send_text(MAV_SEVERITY_INFO, "MOT1%02X", mot1_error);
            // }
            // if (mot2_error != 0){
            //     gcs().send_text(MAV_SEVERITY_INFO, "MOT2%02X", mot2_error);
            // }
            // if (mot3_error != 0){
            //     gcs().send_text(MAV_SEVERITY_INFO, "MOT3%02X", mot3_error);
            // }
            // if (mot4_error != 0){
            //     gcs().send_text(MAV_SEVERITY_INFO, "MOT4%02X", mot4_error);
            // }
            // if (propeller1_error != 0){
            //     gcs().send_text(MAV_SEVERITY_INFO, "PIT1%02X", propeller1_error);
            // }
            // if (propeller2_error != 0){
            //     gcs().send_text(MAV_SEVERITY_INFO, "PIT2%02X", propeller2_error);
            // }
            // if (propeller3_error != 0){
            //     gcs().send_text(MAV_SEVERITY_INFO, "PIT3%02X", propeller3_error);
            // }
            // if (propeller4_error != 0){
            //     gcs().send_text(MAV_SEVERITY_INFO, "PIT4%02X", propeller4_error);
            // }

            gcs().send_text(MAV_SEVERITY_INFO, "P%04d%04d%04d%04d%03d%03d%03d%03d%02d%02d%02d%02d%03ld%03ld%03ld%03ld",
                mot1_rpm, mot2_rpm, mot3_rpm, mot4_rpm,
                mot1_temperature, mot2_temperature, mot3_temperature, mot4_temperature,
                controller1_temperature, controller2_temperature, controller3_temperature, controller4_temperature,
                (int32_t)(propeller1_angle*10), (int32_t)(propeller2_angle*10),(int32_t) (propeller3_angle*10), (int32_t)(propeller4_angle*10));
            // gcs().send_text(MAV_SEVERITY_INFO, "P%04d%04d%04d%04d%03d%03d%03d%03d",
            //     mot1_rpm, mot2_rpm, mot3_rpm, mot4_rpm,
            //     mot1_temperature, mot2_temperature, mot3_temperature, mot4_temperature);
            // gcs().send_text(MAV_SEVERITY_INFO, "W%02d%02d%02d%02d%03ld%03ld%03ld%03ld",
            // controller1_temperature, controller2_temperature, controller3_temperature, controller4_temperature,
            // (int32_t)(propeller1_angle*10), (int32_t)(propeller2_angle*10),(int32_t) (propeller3_angle*10), (int32_t)(propeller4_angle*10));
            gcs().send_text(MAV_SEVERITY_INFO, "MOT1%02X", mot1_error);
            gcs().send_text(MAV_SEVERITY_INFO, "MOT2%02X", mot2_error);
            gcs().send_text(MAV_SEVERITY_INFO, "MOT3%02X", mot3_error);
            gcs().send_text(MAV_SEVERITY_INFO, "MOT4%02X", mot4_error);
            gcs().send_text(MAV_SEVERITY_INFO, "PIT1%02X", propeller1_error);
            gcs().send_text(MAV_SEVERITY_INFO, "PIT2%02X", propeller2_error);
            gcs().send_text(MAV_SEVERITY_INFO, "PIT3%02X", propeller3_error);
            gcs().send_text(MAV_SEVERITY_INFO, "PIT4%02X", propeller4_error);
            last_print_time = AP_HAL::millis();
        }

        if (_print.get()) {
            if (AP_HAL::millis() -  last_print_ms >= 5000) {    //.通过 last_print_ms 限制打印频率为 5 秒一次
                last_print_ms = AP_HAL::millis();
                should_print_servo = true;
                should_print_mot = true;
            }
        }

        if (_enable_srv.get()) {    //. 若启用舵机控制 
            for (uint8_t i_servo = 0; i_servo < FD_CAN_1_MAX_SERVO_NUM; i_servo++) {
                SRV_Channel *this_channel = SRV_Channels::srv_channel(i_servo);
                bool is_flap = false;   //. 标记当前舵机是否为襟翼
                if (this_channel == nullptr) {
                    if (should_print_servo) {
                        gcs().send_text(MAV_SEVERITY_INFO, "%d nullptr", i_servo);
                    }
                    continue;
                }
                if (this_channel->get_function() == SRV_Channel::Aux_servo_function_t::k_flap_tz605) {    //.判断当前通道是否配置为“襟翼”（k_flap是襟翼功能枚举）
                    is_flap = true;
                }
                uint16_t pwm = this_channel->get_output_pwm();
                if (pwm == 0) {
                    pwm = 1500;
                }
                float pwm_value = constrain_float((float)pwm, 1000.f, 2000.f);
                int16_t servo_angle = (pwm_value - 1500.f)*12.f;//+-6000

                if (_servo_ptr[i_servo] != nullptr) {
                    if (is_flap) {
                        bool flap_lock = false;
                        RC_Channel* tmp_ch_flap = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FLAP_LOCK);
                        if (tmp_ch_flap != nullptr) {
                            int16_t tmp_ch_pwm = tmp_ch_flap->get_radio_in(); //. 返回PWM值（微秒）数据类型为int16_t

                            if (tmp_ch_pwm < 1500){
                                flap_lock = true;//.低PWM上锁，即默认上锁
                            }else{
                                flap_lock = false;
                            }
                        }
                        int16_t flap_angle = 0;
                        RC_Channel* tmp_ch_flap_pos = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FLAP_POS);
                        if (tmp_ch_flap_pos != nullptr) {
                            int16_t flap_pwm = tmp_ch_flap_pos->get_radio_in(); //. 返回PWM值（微秒）数据类型为int16_t
                          
                            if (flap_pwm < 1300) {
                                flap_angle = 3000;//.舵机30°对应舵面10°
                            } else if(flap_pwm < 1600){
                                flap_angle = -600;//.舵机-6°对应舵面50°
                            }else{
                                flap_angle = -1500;//.舵机-15°对应舵面60°
                            }
                        }
                        _servo_ptr[i_servo]->enable_brake(is_flap);//. 襟翼舵机启用刹车
                        _servo_ptr[i_servo]->set_brake(flap_lock);//. 襟翼舵机启用刹车
                        _servo_ptr[i_servo]->set_pos(flap_angle/100.f);//. 设置舵机目标角度
                    } else {
                        _servo_ptr[i_servo]->enable_brake(false);
                        _servo_ptr[i_servo]->set_brake(false);
                        _servo_ptr[i_servo]->set_pos(servo_angle/100.f);
                    }

                    _servo_ptr[i_servo]->update();  //.核心：生成CAN帧并调用write_frame发送
                }
            }
            should_print_servo = false;
        }

        if (_enable_mot.get()) {    
            //-桨距控制，0~65535对应-90°到90°范围桨距角
            float pitch_left = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleLeft)*0.25;//.桨距角限制0-25
            float pitch_right = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleRight)*0.25;

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
            int16_t mot_rpm = 0;
            uint8_t mot_mode = 0;
            RC_Channel* tmp_ch_mot = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOT_RPM);
            if (tmp_ch_mot != nullptr) {
                int16_t ch_pwm = tmp_ch_mot->get_radio_in(); //. 返回PWM值（微秒）数据类型为int16_t

                if (ch_pwm < 1100) {
                    mot_mode = 0;
                    mot_rpm = 0;
                } else if(ch_pwm < 1600){
                    mot_mode = 2;
                    mot_rpm = 0;
                }else{
                    mot_mode = 2;
                    mot_rpm = 100;
                }
            }

            for (uint8_t i_mot = 0; i_mot < FD_CAN_1_MAX_MOT_NUM; i_mot++){
                if (_mot_ptr[i_mot] != nullptr) {
                    if (_rev_mot & (1<<i_mot)) {
                        mot_rpm = -mot_rpm;
                    }
                    _mot_ptr[i_mot]->set_mode(mot_mode);
                    _mot_ptr[i_mot]->set_rpm(mot_rpm);
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
