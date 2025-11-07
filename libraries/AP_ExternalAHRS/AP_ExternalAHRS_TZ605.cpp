#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_TZ605_ENABLED

#include "AP_ExternalAHRS_TZ605.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Common/Bitmask.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include <FD1_DATA/FD1_DATA.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_TZ605::AP_ExternalAHRS_TZ605(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)//.构造函数，初始化 TZ605 后端
{
    auto &sm = AP::serialmanager();
    uart_ins = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);

    bool has_ins = frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::GPS) || frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::IMU);
    if (!uart_ins) {
        if (has_ins) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no INS");
            return;
        }
    } else if (!has_ins) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS Set INS but not use");
    }
    baudrate_ins = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num_ins = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    uart_air = sm.find_serial(AP_SerialManager::SerialProtocol_AIR, 0);

    bool has_air = frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::BARO);
    if (!uart_air) {
        if (has_air) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no Air");
            return;
        }
    } else if (!has_air) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS Set AIR but not use baro");
    }

    baudrate_air = sm.find_baudrate(AP_SerialManager::SerialProtocol_AIR, 0);
    port_num_air = sm.find_portnum(AP_SerialManager::SerialProtocol_AIR, 0);


    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_TZ605::update_thread, void), "TZ605", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Failed to allocate ExternalAHRS update thread");
    }

    hal.scheduler->delay(5000);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TZ605 ExternalAHRS initialised");
}

void AP_ExternalAHRS_TZ605::update_thread(void)//-无限循环后台线程，负责实时读取 UART 数据
{
    hal.scheduler->delay(5000);
    if (uart_ins) {
        if (!port_open_ins) {
            port_open_ins = true;
            uart_ins->begin(baudrate_ins, 1024, 512);//.打开串口，配置波特率
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SerialProtocol_AHRS %d", int(baudrate_ins));
        }
    }

    if (uart_air) {
        if (!port_open_air) {
            port_open_air = true;
            uart_air->begin(baudrate_air, 1024, 512);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SerialProtocol_AIR %d", int(baudrate_air));
        }
    }

    // uint32_t _last_post = AP_HAL::millis();
    bool do_print = false;

    while (true) {
        do_print = false;
        // if (AP_HAL::millis() - _last_post > 5000) {
        //     _last_post = AP_HAL::millis();
        //     do_print = true;
        // }
        if (port_open_ins) {
            build_packet_ins();//.读取并解析
            if (do_print) {
                if (frontend.debug_print.get()>0) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "build_packet_ins");
                }
            }
        }
        if (port_open_air) {
            build_packet_air();
            if (do_print) {
                if (frontend.debug_print.get()>0) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "build_packet_air");
                }
            }
        }
        hal.scheduler->delay_microseconds(100);
    }
}

// Builds packets by looking at each individual byte, once a full packet has been read in it checks the checksum then handles the packet.
void AP_ExternalAHRS_TZ605::build_packet_ins()
{
    // uint32_t nbytes = MIN(uart_ins->available(), 2048u);

    // static uint32_t _last_post = AP_HAL::millis();

    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "nbytes %ld", nbytes);
    
    while (uart_ins->available() > 0) { //. 检查缓冲区是否有数据
        uint8_t temp = uart_ins->read();    //. 逐字节读取原始 uint8_t 数据
        _msg_ins.parse(temp);   //. 传入解析器

        if (_msg_ins._msg_1.updated) {
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "updated");
            handle_imu();
            handle_gps();
            if (frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::IMU)) {
                post_imu();
            }
            if (frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::GPS)) {
                post_gps();
            }
            handle_ahrs();
            _msg_ins._msg_1.updated = false;
        }

        // if (AP_HAL::millis() - _last_post > 1000) {
        //     _last_post = AP_HAL::millis();
        //     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PORT IN : %x", b);
        // }
    }
}

void AP_ExternalAHRS_TZ605::build_packet_air()
{
    // uint32_t nbytes = MIN(uart_air->available(), 2048u);

    // static uint32_t _last_post = AP_HAL::millis();

    while (uart_air->available() > 0) {
        uint8_t temp = uart_air->read();
        _msg_air.parse(temp);

        if (_msg_air._msg_1.updated) {
            handle_baro();
            handle_airspeed();
            if (frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::BARO)) {
                post_baro();
            }
            post_airspeed();
            _msg_air._msg_1.updated = false;
        }

        // if (AP_HAL::millis() - _last_post > 1000) {
        //     _last_post = AP_HAL::millis();
        //     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PORT IN : %x", b);
        // }
    }
}

// Collects data from an imu packet into `imu_data`
void AP_ExternalAHRS_TZ605::handle_imu()
{
    last_ins_pkt = AP_HAL::millis();

    Vector3f gravity = AP::ahrs().earth_to_body(Vector3f(0.0f, 0.0f, -GRAVITY_MSS));


    imu_data.accel = gravity + Vector3f(_msg_ins._msg_1.content.msg.acc_x_mss,
                                        _msg_ins._msg_1.content.msg.acc_y_mss,
                                        -_msg_ins._msg_1.content.msg.acc_z_mss);
    // imu_data.accel = Vector3f(_msg_ins._msg_1.content.msg.acc_x_mss,
    //                                     _msg_ins._msg_1.content.msg.acc_y_mss,
    //                                     -_msg_ins._msg_1.content.msg.acc_z_mss);
                                        // m/s^2
    imu_data.gyro = Vector3f(radians(_msg_ins._msg_1.content.msg.rate_n_degrees),
                             radians(_msg_ins._msg_1.content.msg.rate_e_degrees),
                             -radians(_msg_ins._msg_1.content.msg.rate_u_degrees));
    // imu_data.gyro = Vector3f(radians(1.f),
    //                          radians(1.f),
    //                          radians(1.f));
                             // rad/s
    imu_data.temperature = 0.0f;

    static uint32_t _last_post = AP_HAL::millis();
    static float count = 0.0f;
    count += 1.0f;
    if (AP_HAL::millis() - _last_post > 5000) {
        float dt = (float)(AP_HAL::millis() - _last_post) * 0.001f;
        _last_post = AP_HAL::millis();
        if (frontend.debug_print.get()>0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS accel : (%f, %f, %f)", _msg_ins._msg_1.content.msg.acc_x_mss, _msg_ins._msg_1.content.msg.acc_y_mss, _msg_ins._msg_1.content.msg.acc_z_mss);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS gyro : (%f, %f, %f)", _msg_ins._msg_1.content.msg.rate_n_degrees, _msg_ins._msg_1.content.msg.rate_e_degrees, _msg_ins._msg_1.content.msg.rate_u_degrees);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS Rate [%0.1f Hz]", count/dt);
            count = 0.0f;
        }
    }

    static uint32_t _last_error_post = 0;
    if (AP_HAL::millis() - _last_error_post > 5000 && (_msg_ins._msg_1.content.msg.error_code != 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS ERROR1: %d ", int(_msg_ins._msg_1.content.msg.error_code>>16));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS ERROR2: %d ", int(_msg_ins._msg_1.content.msg.error_code&0x0000ffff));
        _last_error_post = AP_HAL::millis();
    }
}

// Posts data from an imu packet to `state` and `handle_external` methods
void AP_ExternalAHRS_TZ605::post_imu()
{
    {
        AP_ExternalAHRS::ins_data_message_t ins {
            accel        : imu_data.accel,
            gyro         : imu_data.gyro,
            temperature  : imu_data.temperature
        };
        AP::ins().handle_external(ins);
    }
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "acc (%f, %f, %f)", state.accel.x, state.accel.y, state.accel.z);
}

// Collects data from an imu packet into `gps_data`
void AP_ExternalAHRS_TZ605::handle_gps()
{
    last_gps_pkt = AP_HAL::millis();

    gps_data.gps_week                    = (0XFF);
    gps_data.ms_tow                      = (AP_HAL::millis());
    gps_data.fix_type                    = (_msg_ins._msg_1.content.msg.gps_fix_state);
    gps_data.satellites_in_view          = ((uint8_t)_msg_ins._msg_1.content.msg.gps_numstat);
    gps_data.horizontal_pos_accuracy     = (1.0f);
    gps_data.vertical_pos_accuracy       = (1.0f);
    gps_data.horizontal_vel_accuracy     = (1.0f);
    gps_data.hdop                        = (0.01f*(float)_msg_ins._msg_1.content.msg.gps_hdop);
    gps_data.vdop                        = (0.01f*(float)_msg_ins._msg_1.content.msg.gps_vdop);
    gps_data.longitude                   = (_msg_ins._msg_1.content.msg.gps_lng);
    gps_data.latitude                    = (_msg_ins._msg_1.content.msg.gps_lat);
    gps_data.msl_altitude                = (_msg_ins._msg_1.content.msg.gps_alt_mm/10);
    gps_data.ned_vel_north               = (_msg_ins._msg_1.content.msg.gps_vel_n_ms_o4/100);
    gps_data.ned_vel_down                = (-_msg_ins._msg_1.content.msg.gps_vel_u_ms_o2);
    gps_data.ned_vel_east                = (_msg_ins._msg_1.content.msg.gps_vel_e_ms_o4/100);
    gps_data.gps_yaw                     = (_msg_ins._msg_1.content.msg.yaw_micro_deg/1000);
    gps_data.gps_yaw_time_ms             = (AP_HAL::millis());
    gps_data.gps_yaw_configured          = (true);
    gps_data.gps_yaw_accuracy            = (5.0f);
    gps_data.have_gps_yaw                = (true);
    gps_data.have_gps_yaw_accuracy       = (true);

    static uint32_t _last_post = AP_HAL::millis();
    if (AP_HAL::millis() - _last_post > 5000) {
        _last_post = AP_HAL::millis();
        if (frontend.debug_print.get()>0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "gps fix %d", _msg_ins._msg_1.content.msg.gps_fix_state);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "gps lng %d | lat %d", int(_msg_ins._msg_1.content.msg.gps_lng), int(_msg_ins._msg_1.content.msg.gps_lat));
        }
    }
    
    AP::fd1_data().set_climb_rate(gps_data.ned_vel_down);//.升降速度
}

// Posts data from an gps packet to `state` and `handle_external` methods
void AP_ExternalAHRS_TZ605::post_gps()
{
    {
        AP_ExternalAHRS::gps_data_message_t gps {
            gps_week                :gps_data.gps_week,
            ms_tow                  :gps_data.ms_tow,
            fix_type                :gps_data.fix_type,
            satellites_in_view      :gps_data.satellites_in_view,
            horizontal_pos_accuracy :gps_data.horizontal_pos_accuracy,
            vertical_pos_accuracy   :gps_data.vertical_pos_accuracy,
            horizontal_vel_accuracy :gps_data.horizontal_vel_accuracy,
            hdop                    :gps_data.hdop,
            vdop                    :gps_data.vdop,
            longitude               :gps_data.longitude,
            latitude                :gps_data.latitude,
            msl_altitude            :gps_data.msl_altitude,
            ned_vel_north           :gps_data.ned_vel_north,
            ned_vel_east            :gps_data.ned_vel_east,
            ned_vel_down            :gps_data.ned_vel_down,
            gps_yaw                 :gps_data.gps_yaw,
            gps_yaw_time_ms         :gps_data.gps_yaw_time_ms,
            gps_yaw_configured      :gps_data.gps_yaw_configured,
            gps_yaw_accuracy        :gps_data.gps_yaw_accuracy,
            have_gps_yaw            :gps_data.have_gps_yaw,
            have_gps_yaw_accuracy   :gps_data.have_gps_yaw_accuracy
        };
        AP::gps().handle_external(gps, 0);
    }
}

void AP_ExternalAHRS_TZ605::handle_ahrs()
{
    {
        WITH_SEMAPHORE(state.sem);
        last_ahrs_pkt = AP_HAL::millis();
        state.accel = imu_data.accel;
        state.gyro = imu_data.gyro;
        state.quat.from_euler(0.000001f*radians((float)_msg_ins._msg_1.content.msg.roll_micro_deg), 0.000001f*radians((float)_msg_ins._msg_1.content.msg.pitch_micro_deg), -0.000001f*radians((float)_msg_ins._msg_1.content.msg.yaw_micro_deg));
        state.location.lng = _msg_ins._msg_1.content.msg.lng;
        state.location.lat = _msg_ins._msg_1.content.msg.lat;
        state.location.set_alt_cm(_msg_ins._msg_1.content.msg.alt_mm/10, Location::AltFrame::ABSOLUTE);
        state.velocity = Vector3f(0.01f*(float)(_msg_ins._msg_1.content.msg.vel_n/100), 0.01f*(float)(_msg_ins._msg_1.content.msg.vel_e/100), -0.01f*(float)(_msg_ins._msg_1.content.msg.vel_u/100));
        // state.velocity = Vector3f(100.0f, 100.0f, 100.0f);

        state.have_quaternion = true;
        state.have_location = _msg_ins._msg_1.content.msg.gps_ok || (_msg_ins._msg_1.content.msg.state == 5);
        state.have_velocity = _msg_ins._msg_1.content.msg.gps_ok || (_msg_ins._msg_1.content.msg.state == 5);

        state.last_location_update_us = AP_HAL::micros();
        
        if (!frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::GPS)) {
            //fake gps
            gps_data.gps_week                    = (0XFF);
            gps_data.ms_tow                      = (AP_HAL::millis());
            gps_data.fix_type                    = (state.have_location?3:1);
            gps_data.satellites_in_view          = (99);
            gps_data.horizontal_pos_accuracy     = (1.0f);
            gps_data.vertical_pos_accuracy       = (1.0f);
            gps_data.horizontal_vel_accuracy     = (1.0f);
            gps_data.hdop                        = (1.0f);
            gps_data.vdop                        = (1.0f);
            gps_data.longitude                   = (state.location.lng);
            gps_data.latitude                    = (state.location.lat);
            gps_data.msl_altitude                = (state.location.alt);
            gps_data.ned_vel_north               = (state.velocity.x);
            gps_data.ned_vel_down                = (state.velocity.z);
            gps_data.ned_vel_east                = (state.velocity.y);
            gps_data.gps_yaw                     = (-0.000001f*radians((float)_msg_ins._msg_1.content.msg.yaw_micro_deg));
            gps_data.gps_yaw_time_ms             = (AP_HAL::millis());
            gps_data.gps_yaw_configured          = (true);
            gps_data.gps_yaw_accuracy            = (5.0f);
            gps_data.have_gps_yaw                = (true);
            gps_data.have_gps_yaw_accuracy       = (true);
            post_gps();
        }
    }

    if (!state.have_origin && _msg_ins._msg_1.content.msg.gps_ok) {
        state.origin.lng = _msg_ins._msg_1.content.msg.lng;
        state.origin.lat = _msg_ins._msg_1.content.msg.lat;
        state.origin.set_alt_cm(_msg_ins._msg_1.content.msg.alt_mm/10, Location::AltFrame::ABSOLUTE);
        state.have_origin = true;
    }
    static uint32_t _last_post = AP_HAL::millis();
    if (AP_HAL::millis() - _last_post > 3000) {
        _last_post = AP_HAL::millis();
        if (frontend.debug_print.get()>0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS fix %d | ok %d", _msg_ins._msg_1.content.msg.gps_fix_state, _msg_ins._msg_1.content.msg.gps_ok);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS lng %d | lat %d", int(_msg_ins._msg_1.content.msg.lng), int(_msg_ins._msg_1.content.msg.lat));
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS alt %d", int(_msg_ins._msg_1.content.msg.alt_mm));
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS roll %d | pitch %d | yaw %d", int(_msg_ins._msg_1.content.msg.roll_micro_deg), int(_msg_ins._msg_1.content.msg.pitch_micro_deg), int(_msg_ins._msg_1.content.msg.yaw_micro_deg));
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS vn %d | ve %d | vu %d", int(_msg_ins._msg_1.content.msg.vel_n), int(_msg_ins._msg_1.content.msg.vel_e), int(_msg_ins._msg_1.content.msg.vel_u));
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS gyror : (%f, %f, %f)", imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z);
        }
    }

    AP::fd1_data().set_alt(_msg_ins._msg_1.content.msg.alt_mm/10);//.mm转cm
    AP::fd1_data().set_roll(_msg_ins._msg_1.content.msg.roll_micro_deg/1000000);//.转deg
    AP::fd1_data().set_yaw(_msg_ins._msg_1.content.msg.yaw_micro_deg/1000000);//.转deg
    AP::fd1_data().set_rate_x(state.gyro.x);//.deg/s
    AP::fd1_data().set_rate_y(state.gyro.y);
    AP::fd1_data().set_rate_z(state.gyro.z);
    AP::fd1_data().set_acc_x(state.accel.x);//.m/s/s
    AP::fd1_data().set_acc_y(state.accel.y);
    AP::fd1_data().set_acc_z(state.accel.z);
    AP::fd1_data().set_gps_utc(_msg_ins._msg_1.content.msg.gps_utc);//.ms
}

// Collects data from an imu packet into `baro_data`
void AP_ExternalAHRS_TZ605::handle_baro()
{
    // last_baro_pkt = AP_HAL::millis();

    baro_data.instance = 0;
    baro_data.pressure_pa = ((float)_msg_air._msg_1.content.msg.ps/1024.f *1000.f);
    baro_data.temperature = ((float)_msg_air._msg_1.content.msg.ts/16.f);

    // bara_alt.instance = 0;
    // bara_alt.altitude = 

    static uint32_t _last_post = AP_HAL::millis();
    static float count = 0.0f;
    count += 1.0f;
    if (AP_HAL::millis() - _last_post > 5000) {
        float dt = (float)(AP_HAL::millis() - _last_post) * 0.001f;
        _last_post = AP_HAL::millis();
        if (frontend.debug_print.get()>0) {
            gcs().send_text(MAV_SEVERITY_INFO, "baro ps: %f | %f", baro_data.pressure_pa, (float)_msg_air._msg_1.content.msg.ps);
            gcs().send_text(MAV_SEVERITY_INFO, "baro ts: %f | %f", baro_data.temperature, (float)_msg_air._msg_1.content.msg.ts);
            gcs().send_text(MAV_SEVERITY_INFO, "baro AOAt1: %f, AOAt2: %f ", ((float)_msg_air._msg_1.content.msg.aoat1/128.f), ((float)_msg_air._msg_1.content.msg.aoat2/128.f));
            gcs().send_text(MAV_SEVERITY_INFO, "baro AOSt1: %f, AOSt2: %f ", ((float)_msg_air._msg_1.content.msg.aost1/128.f), ((float)_msg_air._msg_1.content.msg.aost2/128.f));

            gcs().send_text(MAV_SEVERITY_INFO, "baro Rate [%0.1f Hz]", count/dt);
            count = 0.0f;
        }
    }

    AP::fd1_data().set_aoa(_msg_air._msg_1.content.msg.aoat1/128.f);//.迎角deg
    AP::fd1_data().set_ssa(_msg_air._msg_1.content.msg.aost1/128.f);//.侧滑角deg


    static uint32_t _last_error_post = 0;
    if (AP_HAL::millis() - _last_error_post > 5000 && (_msg_air._msg_1.content.msg.faultword != 0)) {
        gcs().send_text(MAV_SEVERITY_INFO, "baro ERROR: %d ", _msg_air._msg_1.content.msg.faultword);
        _last_error_post = AP_HAL::millis();
    }
}

// Posts data from an baro packet to `state` and `handle_external` methods
void AP_ExternalAHRS_TZ605::post_baro()
{
    {
        AP_ExternalAHRS::baro_data_message_t baro {
            instance     : baro_data.instance,
            pressure_pa  : baro_data.pressure_pa,
            temperature  : baro_data.temperature,
        };
        AP::baro().handle_external(baro);
    }
}

// Collects data from an imu packet into `baro_data`
void AP_ExternalAHRS_TZ605::handle_airspeed()
{
    // last_airspeed_pkt = AP_HAL::millis();
    float ps = ((float)_msg_air._msg_1.content.msg.ps/1024.f*1000.f);
    float qc = ((float)_msg_air._msg_1.content.msg.qc/1024.f*1000.f);
    float rev_airspeed = ((float)_msg_air._msg_1.content.msg.vi/64.f)/3.6f;
    airspeed_data.differential_pressure = ps-qc;
    airspeed_data.temperature = ((float)_msg_air._msg_1.content.msg.ts/16.f);
    airspeed_data.airspeed = rev_airspeed;


    static uint32_t _last_post = AP_HAL::millis();
    if (AP_HAL::millis() - _last_post > 5000) {
        _last_post = AP_HAL::millis();
        if (frontend.debug_print.get()>0) {
            gcs().send_text(MAV_SEVERITY_INFO, "airspeed ps: %f", (float)_msg_air._msg_1.content.msg.ps);
            gcs().send_text(MAV_SEVERITY_INFO, "airspeed qc: %f", (float)_msg_air._msg_1.content.msg.qc);
            gcs().send_text(MAV_SEVERITY_INFO, "airspeed vi: %f | %f", rev_airspeed, (float)_msg_air._msg_1.content.msg.vi);
        }
    }

    AP::fd1_data().set_arspd_tas(airspeed_data.airspeed);//.指示空速
}

// Posts data from an airspeed packet to `state` and `handle_external` methods
void AP_ExternalAHRS_TZ605::post_airspeed()
{
    {
        AP_ExternalAHRS::airspeed_data_message_t airspeed {
            differential_pressure     : airspeed_data.differential_pressure,
            temperature               : airspeed_data.temperature,
            airspeed                  : airspeed_data.airspeed
        };
        AP::airspeed()->handle_external(airspeed);
    }
}

int8_t AP_ExternalAHRS_TZ605::get_port(void) const
{
    if (uart_ins) {
        return port_num_ins;
    }
    if (uart_air) {
        return port_num_air;
    }
    return -1;
};

// Get model/type name
const char* AP_ExternalAHRS_TZ605::get_name() const
{
    return "TZ605";
}

bool AP_ExternalAHRS_TZ605::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return (now - last_ahrs_pkt < 140);
}

bool AP_ExternalAHRS_TZ605::initialised(void) const
{
    return last_ins_pkt != 0;
}

bool AP_ExternalAHRS_TZ605::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "TZ605 unhealthy");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_TZ605::get_filter_status(nav_filter_status &status) const
{
    uint32_t now = AP_HAL::millis();
    memset(&status, 0, sizeof(status));
    if (last_ins_pkt != 0) {
        status.flags.initalized = 1;
    }
    if (healthy() && last_ins_pkt != 0) {
        status.flags.attitude = 1;
    }
    if (now - last_ahrs_pkt < 140 && state.have_location) {
        status.flags.horiz_pos_rel = 1;
        status.flags.horiz_pos_abs = 1;
        status.flags.using_gps = 1;
    }
    if (now - last_ahrs_pkt < 140 && state.have_velocity) {
        status.flags.horiz_vel = 1;
    }
    if (now - last_ahrs_pkt < 140) {
        status.flags.vert_pos = 1;
        status.flags.vert_vel = 1;
    }
}

void AP_ExternalAHRS_TZ605::send_status_report(GCS_MAVLINK &link) const
{
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }

    // send message
    const float mag_var = 0; //we may need to change this to be like the other gates, set to 0 because mag is ignored by the ins filter in vectornav
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags,
                                       0, 0, 0,
                                       mag_var, 0, 0);

}

#endif // AP_EXTERNAL_AHRS_TZ605_ENABLED
