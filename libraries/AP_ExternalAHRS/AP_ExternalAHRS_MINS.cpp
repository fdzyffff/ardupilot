#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_MINS_ENABLED

#include "AP_ExternalAHRS_MINS.h"
#include <AP_AHRS/AP_AHRS.h>
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


extern const AP_HAL::HAL &hal;


AP_ExternalAHRS_MINS::AP_ExternalAHRS_MINS(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)//.构造函数，初始化 MINS 后端
{
    auto &sm = AP::serialmanager();
    uart_ins = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!uart_ins) {
        gcs().send_text(MAV_SEVERITY_INFO, "MINS ExternalAHRS no UART");
        return;
    }

    baudrate_ins = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num_ins = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_MINS::update_thread, void), "MINS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Failed to allocate ExternalAHRS update thread");
    }

    gcs().send_text(MAV_SEVERITY_INFO, "MINS ExternalAHRS initialised");
}

void AP_ExternalAHRS_MINS::update_thread(void)//-无限循环后台线程，负责实时读取 UART 数据
{
    hal.scheduler->delay(5000);
    if (uart_ins) {
        if (!port_open_ins) {
            port_open_ins = true;
            uart_ins->begin(baudrate_ins, 1024, 512);//.打开串口，配置波特率
            gcs().send_text(MAV_SEVERITY_INFO, "SerialProtocol_AHRS %d", int(baudrate_ins));
        }
    }

    // uint32_t _last_post = AP_HAL::millis();
    bool do_print = false;

    while (true) {
        do_print = false;

        if (port_open_ins) {
            build_packet_ins();//.读取并解析
            if (!set_ins) {
                send_ins_setting();
                set_ins = true;
            }
            if (do_print) {
                if (frontend.debug_print.get() & (1<<6)) {
                    gcs().send_text(MAV_SEVERITY_INFO, "build_packet_ins");
                }
            }
        }

        update_mag_cal();

        update_log();

        update_print();

        update_airspeed();

        hal.scheduler->delay_microseconds(100);
    }
}

// Builds packets by looking at each individual byte, once a full packet has been read in it checks the checksum then handles the packet.
void AP_ExternalAHRS_MINS::build_packet_ins()
{
    if (uart_ins == nullptr) {
        return;
    }
    
    while (uart_ins->available() > 0) {     //. 检查缓冲区是否有数据
        uint8_t temp = uart_ins->read();    //. 逐字节读取原始 uint8_t 数据
        _msg_0XD1.parse(temp);               //. 传入解析器

        if (_msg_0XD1._msg_1.updated) {
            print_ahrs_state();
            handle_ahrs();
            _msg_0XD1._msg_1.updated = false;
        }

        _msg_0XA1.parse(temp); 
        if (_msg_0XA1._msg_1.updated) {
            handle_mag_cal();
            _msg_0XA1._msg_1.updated = false;
        }

        // if (AP_HAL::millis() - _last_post > 1000) {
        //     _last_post = AP_HAL::millis();
        //     gcs().send_text(MAV_SEVERITY_INFO, "PORT IN : %x", b);
        // }
    }
}

void AP_ExternalAHRS_MINS::print_ahrs_state()
{
    static uint32_t _last_state_ms = 0;
    if (_msg_0XD1._msg_1.content.msg.state == 1) {
        return;
    }
    _last_state_ms = AP_HAL::millis();

    if (AP_HAL::millis() - _last_state_ms > 5000) {
        if ((_msg_0XD1._msg_1.content.msg.state & 0b00000111) == 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "MINS: initializing ...");
        }
        if ((_msg_0XD1._msg_1.content.msg.state & 0b00000111) == 2) {
            gcs().send_text(MAV_SEVERITY_INFO, "MINS: Error!");
        }
        if (_msg_0XD1._msg_1.content.msg.state & 0b00001000) {
            gcs().send_text(MAV_SEVERITY_INFO, "MINS: Mag calibration is needed");
        }
        if (_msg_0XD1._msg_1.content.msg.state & 0b00010000) {
            gcs().send_text(MAV_SEVERITY_INFO, "MINS: Mag error");
        }
        if (_msg_0XD1._msg_1.content.msg.state & 0b00100000) {
            gcs().send_text(MAV_SEVERITY_INFO, "MINS: Gyro error");
        }
        if (_msg_0XD1._msg_1.content.msg.state & 0b01000000) {
            gcs().send_text(MAV_SEVERITY_INFO, "MINS: Accel error");
        }
        if (_msg_0XD1._msg_1.content.msg.state & 0b10000000) {
            gcs().send_text(MAV_SEVERITY_INFO, "MINS: Baro error");
        }
        _last_state_ms = AP_HAL::millis();
    }
}

void AP_ExternalAHRS_MINS::handle_ahrs()
{

    if ((_msg_0XD1._msg_1.content.msg.state & 0b00000111) == 0) {
        return;
    }

    {
        WITH_SEMAPHORE(state.sem);
        last_ahrs_pkt = AP_HAL::millis();

        frontend.imu_data.accel = Vector3f(_msg_0XD1._msg_1.content.msg.accel_x,
                                  _msg_0XD1._msg_1.content.msg.accel_y,
                                  _msg_0XD1._msg_1.content.msg.accel_z);

        frontend.imu_data.gyro = Vector3f(_msg_0XD1._msg_1.content.msg.roll_rate,
                                 _msg_0XD1._msg_1.content.msg.pitch_rate,
                                 _msg_0XD1._msg_1.content.msg.yaw_rate);
                                 // rad/s
        frontend.imu_data.temperature = _msg_0XD1._msg_1.content.msg.temperature;

        // Vector3f gravity = AP::ahrs().earth_to_body(Vector3f(0.0f, 0.0f, -GRAVITY_MSS));

        // Vector3f accel = gravity + Vector3f(_msg_0XD1._msg_1.content.msg.accel_x,
        //                                     _msg_0XD1._msg_1.content.msg.accel_y,
        //                                     _msg_0XD1._msg_1.content.msg.accel_z);

        // Vector3f accel =  Vector3f(_msg_0XD1._msg_1.content.msg.accel_x,
        //                            _msg_0XD1._msg_1.content.msg.accel_y,
        //                            _msg_0XD1._msg_1.content.msg.accel_z);

        // Vector3f gyro = Vector3f(_msg_0XD1._msg_1.content.msg.roll_rate,
        //                          _msg_0XD1._msg_1.content.msg.pitch_rate,
        //                          _msg_0XD1._msg_1.content.msg.yaw_rate);

        state.accel = frontend.imu_data.accel;
        state.gyro = frontend.imu_data.gyro;
        state.quat.from_euler(_msg_0XD1._msg_1.content.msg.roll, _msg_0XD1._msg_1.content.msg.pitch, _msg_0XD1._msg_1.content.msg.yaw);
        state.location.lng = _msg_0XD1._msg_1.content.msg.lon;
        state.location.lat = _msg_0XD1._msg_1.content.msg.lat;
        state.location.set_alt_cm(_msg_0XD1._msg_1.content.msg.alt, Location::AltFrame::ABSOLUTE);
        state.velocity = Vector3f(_msg_0XD1._msg_1.content.msg.velocity_x, _msg_0XD1._msg_1.content.msg.velocity_y, _msg_0XD1._msg_1.content.msg.velocity_z);

        state.have_quaternion = true;
        state.have_location = _msg_0XD1._msg_1.content.msg.gps_status >= 3;
        state.have_velocity = _msg_0XD1._msg_1.content.msg.gps_status >= 3;

        state.last_location_update_us = AP_HAL::micros();

            // // test purpose
            // state.have_location = true;
            // state.location.lng = 1163398075;
            // state.location.lat = 399786986;
            // state.location.set_alt_cm(5322, Location::AltFrame::ABSOLUTE);


        // if (frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::GPS)) {
            //fake gps
            uint32_t gps_week_ms = _msg_0XD1._msg_1.content.msg.gps_day * 84600 * 1000 + _msg_0XD1._msg_1.content.msg.gps_hh * 3600 * 1000 + _msg_0XD1._msg_1.content.msg.gps_mm * 60 * 1000 + _msg_0XD1._msg_1.content.msg.gps_ss * 1000 + _msg_0XD1._msg_1.content.msg.gps_ms;
            frontend.gps_data.gps_week                    = (_msg_0XD1._msg_1.content.msg.gps_week);
            frontend.gps_data.ms_tow                      = gps_week_ms;
            frontend.gps_data.fix_type                    = (AP_GPS_FixType)(_msg_0XD1._msg_1.content.msg.gps_status);
            frontend.gps_data.satellites_in_view          = (_msg_0XD1._msg_1.content.msg.satellite_num);
            frontend.gps_data.horizontal_pos_accuracy     = (1.0f);
            frontend.gps_data.vertical_pos_accuracy       = (1.0f);
            frontend.gps_data.horizontal_vel_accuracy     = (1.0f);
            frontend.gps_data.hdop                        = ((float)_msg_0XD1._msg_1.content.msg.hdop);
            frontend.gps_data.vdop                        = ((float)_msg_0XD1._msg_1.content.msg.vdop);
            frontend.gps_data.longitude                   = (state.location.lng);
            frontend.gps_data.latitude                    = (state.location.lat);
            frontend.gps_data.msl_altitude                = (state.location.alt);
            frontend.gps_data.ned_vel_north               = (state.velocity.x);
            frontend.gps_data.ned_vel_down                = (state.velocity.z);
            frontend.gps_data.ned_vel_east                = (state.velocity.y);
            frontend.gps_data.gps_yaw                     = (0.1f*(float)_msg_0XD1._msg_1.content.msg.HDT);
            frontend.gps_data.gps_yaw_time_ms             = (AP_HAL::millis());
            frontend.gps_data.gps_yaw_configured          = (true);
            frontend.gps_data.gps_yaw_accuracy            = (0.1f*(float)_msg_0XD1._msg_1.content.msg.HDG_Dev);
            frontend.gps_data.have_gps_yaw                = (0.1f*(float)_msg_0XD1._msg_1.content.msg.HDG_Dev < 5.0f);
            frontend.gps_data.have_gps_yaw_accuracy       = (true);
            frontend.gps_data.ground_speed                = (float)state.velocity.xy().length();
            frontend.gps_data.ground_course               = wrap_360((float)_msg_0XD1._msg_1.content.msg.yaw_gps);

            // // test purpose
            // state.have_location = true;
            // frontend.gps_data.fix_type                    = (AP_GPS_FixType)(4);
            // frontend.gps_data.gps_yaw                     = 233.f;
            // frontend.gps_data.have_gps_yaw                = true;
            // frontend.gps_data.ground_course               = 112.f;

            post_gps();
        // }

        // if (frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::IMU)) {
        //     post_imu();
        // }

    }

    static uint32_t _last_location_ms = 0;

    if (!state.have_origin && state.have_location) {
        if (_last_location_ms == 0) {
            _last_location_ms = AP_HAL::millis();
        }

        if (_last_location_ms != 0 && (AP_HAL::millis() - _last_location_ms > 10000)) {
            state.origin.lng = state.location.lng;
            state.origin.lat = state.location.lat;
            state.origin.alt = state.location.alt;
            state.have_origin = true;
            _last_location_ms = 0;
            gcs().send_text(MAV_SEVERITY_INFO, "Origin: %d, %d", (int)state.origin.lat, (int)state.origin.lng);
        }
    }

    static uint32_t _last_post = AP_HAL::millis();
    if (AP_HAL::millis() - _last_post > 3000) {
        _last_post = AP_HAL::millis();
        if (frontend.debug_print.get() & (1<<6)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Counter %d", int(_msg_0XD1._msg_1.content.msg.counter));
            // gcs().send_text(MAV_SEVERITY_INFO, "AHRS fix %d | ok %d", _msg_0XD1._msg_1.content.msg.gps_fix_state, _msg_0XD1._msg_1.content.msg.gps_ok);
            // gcs().send_text(MAV_SEVERITY_INFO, "AHRS lng %d | lat %d", int(_msg_0XD1._msg_1.content.msg.lng), int(_msg_0XD1._msg_1.content.msg.lat));
            // gcs().send_text(MAV_SEVERITY_INFO, "AHRS alt %d", int(_msg_0XD1._msg_1.content.msg.alt_mm));
            gcs().send_text(MAV_SEVERITY_INFO, "AHRS roll %f | pitch %f | yaw %f", degrees(_msg_0XD1._msg_1.content.msg.roll), degrees(_msg_0XD1._msg_1.content.msg.pitch), degrees(_msg_0XD1._msg_1.content.msg.yaw));
            gcs().send_text(MAV_SEVERITY_INFO, "AHRS vx %f | vy %f | vz %f", _msg_0XD1._msg_1.content.msg.velocity_x, _msg_0XD1._msg_1.content.msg.velocity_y, _msg_0XD1._msg_1.content.msg.velocity_z);
            // gcs().send_text(MAV_SEVERITY_INFO, "AHRS gyror : (%f, %f, %f)", frontend.imu_data.gyro.x, frontend.imu_data.gyro.y, frontend.imu_data.gyro.z);
            gcs().send_text(MAV_SEVERITY_INFO, "AHRS GPSstatus %d, GPS numstats %d", _msg_0XD1._msg_1.content.msg.gps_status, _msg_0XD1._msg_1.content.msg.satellite_num);
        }
    }
}

// Posts data from an gps packet to `state` and `handle_external` methods
void AP_ExternalAHRS_MINS::post_gps()
{
    static uint32_t last_gps_post_ms = AP_HAL::millis();
    if (AP_HAL::millis() - last_gps_post_ms < 100) {
        return;
    }
    last_gps_post_ms = AP_HAL::millis();
    AP::gps().handle_external(frontend.gps_data, 0);
}

// Posts data from an imu packet to `state` and `handle_external` methods
void AP_ExternalAHRS_MINS::post_imu()
{
    AP::ins().handle_external(frontend.imu_data);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "acc (%f, %f, %f)", state.accel.x, state.accel.y, state.accel.z);
}

void AP_ExternalAHRS_MINS::update_log()
{
#if HAL_LOGGING_ENABLED
    static uint32_t last_log_ms = AP_HAL::millis();
    if (AP_HAL::millis() - last_log_ms < 20) {
        return;
    }
    last_log_ms = AP_HAL::millis();

    uint64_t now_us = AP_HAL::micros64();

    // @LoggerMessage: MIN1
    // @Description: MINS data
    // @Field: TimeUS: Time since system startup
    // @Field: GyrX: Gyro X
    // @Field: GyrY: Gyro Y
    // @Field: GyrZ: Gyro z
    // @Field: AccX: Accelerometer X
    // @Field: AccY: Accelerometer Y
    // @Field: AccZ: Accelerometer Z

    AP::logger().WriteStreaming("MIN1", "TimeUS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ",
                                "skkkooo",
                                "F------",
                                "Qffffff",
                                now_us,
                                frontend.imu_data.gyro.x, frontend.imu_data.gyro.y, frontend.imu_data.gyro.z,
                                frontend.imu_data.accel.x, frontend.imu_data.accel.y, frontend.imu_data.accel.z);

    AP::logger().WriteStreaming("MIN2", "TimeUS,Stat,CRATE,FRATE,VIS,HDGDEV,REDU,DT0,DT1",
                                "s--------",
                                "F--------",
                                "QBBBBHBBB",
                                now_us,
                                _msg_0XD1._msg_1.content.msg.state, 
                                _msg_0XD1._msg_1.content.msg.cam_frame_rate,
                                _msg_0XD1._msg_1.content.msg.ins_frame_rate,
                                _msg_0XD1._msg_1.content.msg.visual_connect,
                                _msg_0XD1._msg_1.content.msg.HDG_Dev,
                                _msg_0XD1._msg_1.content.msg.redundancy,
                                _msg_0XD1._msg_1.content.msg.GPS0_DT,
                                _msg_0XD1._msg_1.content.msg.GPS0_DT);
#endif // HAL_LOGGING_ENABLED
}

void AP_ExternalAHRS_MINS::update_print()
{
    static uint32_t _last_post = AP_HAL::millis();
    if (AP_HAL::millis() - _last_post > 3000) {
        _last_post = AP_HAL::millis();
        if (mag_calibrating) {
            return;
        }
        if (frontend.debug_print.get() & (1<<0)) {
            gcs().send_text(MAV_SEVERITY_INFO, "~~~~~~~~~ MINS ~~~~~~~~~");
            gcs().send_text(MAV_SEVERITY_INFO, "State %d, Cam FPS %d, INS FPS %d", _msg_0XD1._msg_1.content.msg.state,  _msg_0XD1._msg_1.content.msg.cam_frame_rate, _msg_0XD1._msg_1.content.msg.ins_frame_rate);
            gcs().send_text(MAV_SEVERITY_INFO, "VISUAL %d, Redundancy %d", _msg_0XD1._msg_1.content.msg.visual_connect, _msg_0XD1._msg_1.content.msg.redundancy);
            gcs().send_text(MAV_SEVERITY_INFO, "GPS0_DT %d, GPS1_DT %d", _msg_0XD1._msg_1.content.msg.GPS0_DT, _msg_0XD1._msg_1.content.msg.GPS1_DT);
            gcs().send_text(MAV_SEVERITY_INFO, "HDG_Dev %0.1f, YAW_GPS %0.1f | %0.1f", (0.1f * (float)_msg_0XD1._msg_1.content.msg.HDG_Dev), (float)_msg_0XD1._msg_1.content.msg.yaw_gps, wrap_360((float)_msg_0XD1._msg_1.content.msg.yaw_gps));
            gcs().send_text(MAV_SEVERITY_INFO, "~~~~~~~~~ END ~~~~~~~~~");
        }
    }
}

int8_t AP_ExternalAHRS_MINS::get_port(void) const
{
    if (uart_ins) {
        return port_num_ins;
    }
    return -1;
};

// Get model/type name
const char* AP_ExternalAHRS_MINS::get_name() const
{
    return "MINS";
}

bool AP_ExternalAHRS_MINS::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return (now - last_ahrs_pkt < 140);
}

bool AP_ExternalAHRS_MINS::initialised(void) const
{
    return last_ahrs_pkt != 0;
}

bool AP_ExternalAHRS_MINS::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "MINS unhealthy");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_MINS::get_filter_status(nav_filter_status &status) const
{
    uint32_t now = AP_HAL::millis();
    memset(&status, 0, sizeof(status));
    if (last_ahrs_pkt != 0) {
        status.flags.initalized = 1;
    }
    if (healthy() && last_ahrs_pkt != 0) {
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

bool AP_ExternalAHRS_MINS::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    return false;
}

void AP_ExternalAHRS_MINS::send_ins_setting()
{
    return;
}

void AP_ExternalAHRS_MINS::update_mag_cal()
{
    if (frontend.mag_cal.get() != 0) {
        send_mag_cal(0x01);
        mag_calibrating = true;
        frontend.mag_cal.set_and_save(0);
        _last_cal_ms = AP_HAL::millis();
        for (uint8_t mag_id = 0; mag_id < 2; mag_id++) {
            hal.util->snprintf(_mag_cal[mag_id].msg, 5, "NONE");
            _mag_cal[mag_id].cal_status = 0;
            _mag_cal[mag_id].mag_calibrating = true;
            _mag_cal[mag_id].last_cal_ms = AP_HAL::millis();
        }
        gcs().send_text(MAV_SEVERITY_INFO, "MINS MAG CAL Start");
    }

    if (mag_calibrating) {
        static uint32_t _last_mag_pct_ms = 0;
        if (AP_HAL::millis() - _last_mag_pct_ms > 500) {
            send_mag_cal(0x04);
            _last_mag_pct_ms = AP_HAL::millis();
        }

        if (AP_HAL::millis() - _last_cal_ms > 10000) {
            gcs().send_text(MAV_SEVERITY_INFO, "No cal progress, cancel");
            send_mag_cal(0x02);
            mag_calibrating = false;
        }


        for (uint8_t mag_id = 0; mag_id < 2; mag_id++) {
            if (AP_HAL::millis() - _mag_cal[mag_id].last_cal_ms > 3000) {
                // gcs().send_text(MAV_SEVERITY_INFO, "No cal progress, cancel");
                hal.util->snprintf(_mag_cal[mag_id].msg, 3, "NO");
                _mag_cal[mag_id].mag_calibrating = false;
            }
        }

        static uint32_t _last_mag_print_ms = 0;
        if (AP_HAL::millis() - _last_mag_print_ms > 1500) {
            _last_mag_print_ms = AP_HAL::millis();
            gcs().send_text(MAV_SEVERITY_INFO, "MAG1 :%s, MAG2 :%s", _mag_cal[0].msg, _mag_cal[1].msg);
            mag_calibrating = _mag_cal[0].mag_calibrating || _mag_cal[1].mag_calibrating;
        }

    }
}

void AP_ExternalAHRS_MINS::send_mag_cal(uint8_t cmd)
{
    _msg_0XCC._msg_1.content.msg.cmd = cmd;
    _msg_0XCC._msg_1.content.msg.ID = 0xcc;
    _msg_0XCC._msg_1.length = 7;
    _msg_0XCC.sum_check();
    if (uart_ins != nullptr) {
        uart_ins->write(_msg_0XCC._msg_1.content.data, _msg_0XCC._msg_1.length);
    }
}

void AP_ExternalAHRS_MINS::handle_mag_cal()
{
    // gcs().send_text(MAV_SEVERITY_INFO, "MINS _msg_0XA1._msg_1.content.msg.cal_status %d", _msg_0XA1._msg_1.content.msg.cal_status);
    // gcs().send_text(MAV_SEVERITY_INFO, "try %d, MINS MAG CAL %d%%", _msg_0XA1._msg_1.content.msg.attempt, _msg_0XA1._msg_1.content.msg.completion_pct);
    _last_cal_ms = AP_HAL::millis();
    // gcs().send_text(MAV_SEVERITY_INFO, "MAG %d", _msg_0XA1._msg_1.content.msg.compass_id);

    for (uint8_t mag_id = 0; mag_id < 2; mag_id++) {
        if (_msg_0XA1._msg_1.length >= (27*(mag_id+1) + 6)) {
            // gcs().send_text(MAV_SEVERITY_INFO, "MAG %d", _msg_0XA1._msg_1.content.msg.mag_cal[mag_id].compass_id);
            _mag_cal[mag_id].last_cal_ms = AP_HAL::millis();
            _mag_cal[mag_id].cal_status = _msg_0XA1._msg_1.content.msg.mag_cal[mag_id].cal_status;
            _mag_cal[mag_id].attempt = _msg_0XA1._msg_1.content.msg.mag_cal[mag_id].attempt;
            _mag_cal[mag_id].completion_pct = _msg_0XA1._msg_1.content.msg.mag_cal[mag_id].completion_pct;

            switch (_mag_cal[mag_id].cal_status) {
                default:
                case 0:
                    hal.util->snprintf(_mag_cal[mag_id].msg, 5, "NONE");
                    break;
                case 1:
                case 2:
                case 3: {
                        hal.util->snprintf(_mag_cal[mag_id].msg, 8, "%d%%", _mag_cal[mag_id].completion_pct);
                    }
                    break;
                case 4:
                    hal.util->snprintf(_mag_cal[mag_id].msg, 8, "SUCCESS");
                    _mag_cal[mag_id].mag_calibrating = false;
                    break;
                case 5:
                    hal.util->snprintf(_mag_cal[mag_id].msg, 5, "FAIL");
                    _mag_cal[mag_id].mag_calibrating = false;
                    break;
            }
        }
    }
}

void AP_ExternalAHRS_MINS::update_airspeed()
{
    static uint32_t _last_airspeed_ms = 0;
    if (AP_HAL::millis() - _last_airspeed_ms > 1000) {
        _msg_0XA2._msg_1.length = 24;
        _msg_0XA2._msg_1.content.msg.ID = 0xA2;
        int32_t temp_airspeed = 0;
        bool have_airspeed = false;

        const auto *airspeed = AP::airspeed();
        if (airspeed == nullptr) {
            temp_airspeed = 0;
        } else {
            if (airspeed->healthy()) {
                temp_airspeed = (int32_t)(airspeed->get_airspeed() * 100.f);
                have_airspeed = true;
            }
        }

        _msg_0XA2._msg_1.content.msg.airspeed = temp_airspeed;
        _msg_0XA2.sum_check();
        if (uart_ins != nullptr && have_airspeed) {
            uart_ins->write(_msg_0XA2._msg_1.content.data, _msg_0XA2._msg_1.length);
        }
        _last_airspeed_ms = AP_HAL::millis();
    }

}

#endif // AP_EXTERNAL_AHRS_MINS_ENABLED
