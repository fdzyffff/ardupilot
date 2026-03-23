#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_HITL_ENABLED

#include "AP_ExternalAHRS_HITL.h"
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
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Compass/AP_Compass.h>
#include <FD_DATA/FD_DATA.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_HITL::AP_ExternalAHRS_HITL(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart_hitl = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!uart_hitl) {
        gcs().send_text(MAV_SEVERITY_INFO, "MINS ExternalAHRS no UART");
        return;
    }

    baudrate_hitl = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num_hitl = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_HITL::update_thread, void), "MINS", 2048, AP_HAL::Scheduler::PRIORITY_UART, 0)) {
        AP_BoardConfig::allocation_error("Failed to allocate ExternalAHRS update thread");
    }

    gcs().send_text(MAV_SEVERITY_INFO, "MINS ExternalAHRS initialised");
}

void AP_ExternalAHRS_HITL::update()
{
    // bool do_print = false;
    // if (port_open_hitl) {
    //     build_packet_hitl();//.读取并解析

    //     if (do_print) {
    //         if (frontend.debug_print.get() & (1<<6)) {
    //             gcs().send_text(MAV_SEVERITY_INFO, "build_packet_hitl");
    //         }
    //     }
    // }

    // update_actuator_controls();

    // update_heartbeat();
}

void AP_ExternalAHRS_HITL::update_thread(void)
{
    hal.scheduler->delay(5000);
    if (uart_hitl) {
        if (!port_open_hitl) {
            port_open_hitl = true;
            uart_hitl->begin(baudrate_hitl, 1024, 512);//.打开串口，配置波特率
            gcs().send_text(MAV_SEVERITY_INFO, "SerialProtocol_AHRS %d", int(baudrate_hitl));
        }
    }

    // uint32_t _last_post = AP_HAL::millis();
    // uint32_t start_up_count = 10000;
    // while (start_up_count > 0) {
    //     AP::ins().handle_external(frontend.imu_data);
    //     AP::compass().handle_external(frontend.mag_data);

    //     frontend.baro_data.instance = 0;
    //     AP::baro().handle_external(frontend.baro_data);
    //     frontend.baro_data.instance = 1;
    //     AP::baro().handle_external(frontend.baro_data);
    //     frontend.baro_data.instance = 2;
    //     AP::baro().handle_external(frontend.baro_data);

    //     hal.scheduler->delay_microseconds(5000);
    //     start_up_count--;
    // }

    bool do_print = false;
    while (true) {
        do_print = false;

        if (port_open_hitl) {
            build_packet_hitl();//.读取并解析

            if (do_print) {
                if (frontend.debug_print.get() & (1<<6)) {
                    gcs().send_text(MAV_SEVERITY_INFO, "build_packet_hitl");
                }
            }
        }

        // update_log();

        // update_print();

        update_actuator_controls();

        update_heartbeat();

        // update_imu_post();

        hal.scheduler->delay_microseconds(1000);
    }
}

// Builds packets by looking at each individual byte, once a full packet has been read in it checks the checksum then handles the packet.
void AP_ExternalAHRS_HITL::build_packet_hitl()
{
    if (uart_hitl == nullptr) {
        return;
    }

    while (uart_hitl->available() > 0) {     //. 检查缓冲区是否有数据
        uint8_t temp = uart_hitl->read();    //. 逐字节读取原始 uint8_t 数据
        //. 传入解析器
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t ret = mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status, temp, &msg, &status);

        if (ret >= MAVLINK_FRAMING_OK) {
            if (msg.msgid == MAVLINK_MSG_ID_HIL_SENSOR) {
                mavlink_msg_hil_sensor_decode(&msg, &hil_sensor_packet);
                handle_sensor(hil_sensor_packet);
            }
            if (msg.msgid == MAVLINK_MSG_ID_HIL_GPS) {
                mavlink_msg_hil_gps_decode(&msg, &hil_gps_packet);
                handle_hil_gps(hil_gps_packet);
            }
        }
    }
}

// Collects data from hil_sensor packet into acc gyro mag and baro
void AP_ExternalAHRS_HITL::handle_sensor(mavlink_hil_sensor_t &in_packet)
{
    last_ins_pkt = AP_HAL::millis();

    {
        frontend.imu_data.accel = Vector3f(in_packet.xacc, in_packet.yacc, in_packet.zacc);
                                        // m/s^2

        frontend.imu_data.gyro = Vector3f(in_packet.xgyro, in_packet.ygyro, in_packet.zgyro);
                                 // rad/s

        frontend.imu_data.temperature = in_packet.temperature;

        // only use for externalahrs case, not use for external sensor
        // state.accel = frontend.imu_data.accel;
        // state.gyro = frontend.imu_data.gyro;

        frontend.mag_data.field = Vector3f(in_packet.xmag, in_packet.ymag, in_packet.zmag);

        frontend.baro_data.instance = 0;
        frontend.baro_data.pressure_pa = in_packet.abs_pressure*100.f;
        frontend.baro_data.temperature = in_packet.temperature;

        {
            WITH_SEMAPHORE(state.sem);
            AP::ins().handle_external(frontend.imu_data);
            AP::compass().handle_external(frontend.mag_data);

            frontend.baro_data.instance = 0;
            AP::baro().handle_external(frontend.baro_data);
            frontend.baro_data.instance = 1;
            AP::baro().handle_external(frontend.baro_data);
            frontend.baro_data.instance = 2;
            AP::baro().handle_external(frontend.baro_data);
        }


    }


    ins_frame_count += 1.0f;
    uint32_t now = AP_HAL::millis();
    if (now - _last_ins_print > 5000) {
        float dt = (float)(now - _last_ins_print) * 0.001f;
        _last_ins_print = AP_HAL::millis();
        if (frontend.debug_print.get()>0) {
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS accel : (%f, %f, %f)", _msg_ins._msg_1.content.msg.acc_x_mss, _msg_ins._msg_1.content.msg.acc_y_mss, _msg_ins._msg_1.content.msg.acc_z_mss);
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS gyro : (%f, %f, %f)", _msg_ins._msg_1.content.msg.rate_n_degrees, _msg_ins._msg_1.content.msg.rate_e_degrees, _msg_ins._msg_1.content.msg.rate_u_degrees);
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS ERROR1: %d ", int(_msg_ins._msg_1.content.msg.error_code>>16));
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS ERROR2: %d ", int(_msg_ins._msg_1.content.msg.error_code&0x0000ffff));
            if (!is_zero(dt)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS Rate [%0.1f Hz]", ins_frame_count/dt);
            }
            ins_frame_count = 0.0f;
        }
    }
}

void AP_ExternalAHRS_HITL::handle_hil_gps(mavlink_hil_gps_t &in_packet)
{
    last_gps_pkt = AP_HAL::millis();

    {
        // only use for externalahrs case, not use for external sensor
        // state.accel = frontend.imu_data.accel;
        // state.gyro = frontend.imu_data.gyro;
        // state.quat.from_euler(_msg_0XD1._msg_1.content.msg.roll, _msg_0XD1._msg_1.content.msg.pitch, _msg_0XD1._msg_1.content.msg.yaw);
        // state.location.lng = _msg_0XD1._msg_1.content.msg.lon;
        // state.location.lat = _msg_0XD1._msg_1.content.msg.lat;
        // state.location.set_alt_cm(_msg_0XD1._msg_1.content.msg.alt, Location::AltFrame::ABSOLUTE);
        // state.velocity = Vector3f(_msg_0XD1._msg_1.content.msg.velocity_x, _msg_0XD1._msg_1.content.msg.velocity_y, _msg_0XD1._msg_1.content.msg.velocity_z);

        // state.have_quaternion = true;
        // state.have_location = _msg_0XD1._msg_1.content.msg.gps_status >= 3;
        // state.have_velocity = _msg_0XD1._msg_1.content.msg.gps_status >= 3;

        // state.last_location_update_us = AP_HAL::micros();


        //fake gps
        uint32_t gps_week_ms = AP_HAL::millis();
        frontend.gps_data.gps_week                    = 2046;
        frontend.gps_data.ms_tow                      = gps_week_ms;
        frontend.gps_data.fix_type                    = (AP_GPS_FixType)(in_packet.fix_type);
        frontend.gps_data.satellites_in_view          = (in_packet.satellites_visible);
        frontend.gps_data.horizontal_pos_accuracy     = (0.5f);
        frontend.gps_data.vertical_pos_accuracy       = (0.5f);
        frontend.gps_data.horizontal_vel_accuracy     = (0.5f);
        frontend.gps_data.hdop                        = (in_packet.eph);
        frontend.gps_data.vdop                        = (in_packet.epv);
        frontend.gps_data.longitude                   = (in_packet.lon);
        frontend.gps_data.latitude                    = (in_packet.lat);
        frontend.gps_data.msl_altitude                = (in_packet.alt/10);
        frontend.gps_data.ned_vel_north               = (float)(in_packet.vn)*0.01f;
        frontend.gps_data.ned_vel_east                = (float)(in_packet.ve)*0.01f;
        frontend.gps_data.ned_vel_down                = (float)(in_packet.vd)*0.01f;
        frontend.gps_data.gps_yaw                     = ((float)(in_packet.yaw)*0.01f);
        frontend.gps_data.gps_yaw_time_ms             = (AP_HAL::millis());
        frontend.gps_data.gps_yaw_configured          = (true);
        frontend.gps_data.gps_yaw_accuracy            = (1.0f);
        frontend.gps_data.have_gps_yaw                = (true);
        frontend.gps_data.have_gps_yaw_accuracy       = (true);
        frontend.gps_data.ground_speed                = (float)(in_packet.vel)*0.01f;
        frontend.gps_data.ground_course               = wrap_360((float)(in_packet.cog)*0.01f);
        post_gps();
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

    if (AP_HAL::millis() - _last_gps_print > 3000) {
        _last_gps_print = AP_HAL::millis();
        if (frontend.debug_print.get() & (1<<6)) {
            // gcs().send_text(MAV_SEVERITY_INFO, "Counter %d", int(_msg_0XD1._msg_1.content.msg.counter));
            // gcs().send_text(MAV_SEVERITY_INFO, "AHRS fix %d | ok %d", _msg_0XD1._msg_1.content.msg.gps_fix_state, _msg_0XD1._msg_1.content.msg.gps_ok);
            // gcs().send_text(MAV_SEVERITY_INFO, "AHRS lng %d | lat %d", int(_msg_0XD1._msg_1.content.msg.lng), int(_msg_0XD1._msg_1.content.msg.lat));
            // gcs().send_text(MAV_SEVERITY_INFO, "AHRS alt %d", int(_msg_0XD1._msg_1.content.msg.alt_mm));
            // gcs().send_text(MAV_SEVERITY_INFO, "AHRS roll %f | pitch %f | yaw %f", degrees(_msg_0XD1._msg_1.content.msg.roll), degrees(_msg_0XD1._msg_1.content.msg.pitch), degrees(_msg_0XD1._msg_1.content.msg.yaw));
            // gcs().send_text(MAV_SEVERITY_INFO, "AHRS vx %f | vy %f | vz %f", _msg_0XD1._msg_1.content.msg.velocity_x, _msg_0XD1._msg_1.content.msg.velocity_y, _msg_0XD1._msg_1.content.msg.velocity_z);
            // gcs().send_text(MAV_SEVERITY_INFO, "AHRS gyror : (%f, %f, %f)", frontend.imu_data.gyro.x, frontend.imu_data.gyro.y, frontend.imu_data.gyro.z);
            // gcs().send_text(MAV_SEVERITY_INFO, "AHRS GPSstatus %d, GPS numstats %d", _msg_0XD1._msg_1.content.msg.gps_status, _msg_0XD1._msg_1.content.msg.satellite_num);
        }
    }
}

// Posts data from an gps packet to `state` and `handle_external` methods
void AP_ExternalAHRS_HITL::post_gps()
{
    if (AP_HAL::millis() - _last_gps_post_ms < 100) {
        return;
    }
    _last_gps_post_ms = AP_HAL::millis();
    {

        WITH_SEMAPHORE(state.sem);
        AP::gps().handle_external(frontend.gps_data, 0);
    }
}

// Posts data from an imu packet to `state` and `handle_external` methods
void AP_ExternalAHRS_HITL::post_imu()
{
    AP::ins().handle_external(frontend.imu_data);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "acc (%f, %f, %f)", state.accel.x, state.accel.y, state.accel.z);
}

void AP_ExternalAHRS_HITL::update_log()
{
#if HAL_LOGGING_ENABLED
    if (AP_HAL::millis() - _last_log_ms < 20) {
        return;
    }
    _last_log_ms = AP_HAL::millis();

    uint64_t now_us = AP_HAL::micros64();

    // @LoggerMessage: ZYH1
    // @Description: ZYH1 data
    // @Field: TimeUS: Time since system startup
    // @Field: GyrX: Gyro X
    // @Field: GyrY: Gyro Y
    // @Field: GyrZ: Gyro z
    // @Field: AccX: Accelerometer X
    // @Field: AccY: Accelerometer Y
    // @Field: AccZ: Accelerometer Z

    AP::logger().WriteStreaming("ZYH1", "TimeUS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ",
                                "skkkooo",
                                "F------",
                                "Qffffff",
                                now_us,
                                frontend.imu_data.gyro.x, frontend.imu_data.gyro.y, frontend.imu_data.gyro.z,
                                frontend.imu_data.accel.x, frontend.imu_data.accel.y, frontend.imu_data.accel.z);

#endif // HAL_LOGGING_ENABLED
}

void AP_ExternalAHRS_HITL::update_print()
{
    if (AP_HAL::millis() - _last_global_print > 3000) {
        _last_global_print = AP_HAL::millis();
        if (frontend.debug_print.get() & (1<<0)) {
            gcs().send_text(MAV_SEVERITY_INFO, "~~~~~~~~~ ZYSIM ~~~~~~~~~");
            // gcs().send_text(MAV_SEVERITY_INFO, "State %d, Cam FPS %d, INS FPS %d", _msg_0XD1._msg_1.content.msg.state,  _msg_0XD1._msg_1.content.msg.cam_frame_rate, _msg_0XD1._msg_1.content.msg.ins_frame_rate);
            // gcs().send_text(MAV_SEVERITY_INFO, "VISUAL %d, Redundancy %d", _msg_0XD1._msg_1.content.msg.visual_connect, _msg_0XD1._msg_1.content.msg.redundancy);
            // gcs().send_text(MAV_SEVERITY_INFO, "GPS0_DT %d, GPS1_DT %d", _msg_0XD1._msg_1.content.msg.GPS0_DT, _msg_0XD1._msg_1.content.msg.GPS1_DT);
            // gcs().send_text(MAV_SEVERITY_INFO, "HDG_Dev %0.1f, YAW_GPS %0.1f | %0.1f", (0.1f * (float)_msg_0XD1._msg_1.content.msg.HDG_Dev), (float)_msg_0XD1._msg_1.content.msg.yaw_gps, wrap_360((float)_msg_0XD1._msg_1.content.msg.yaw_gps));
            gcs().send_text(MAV_SEVERITY_INFO, "~~~~~~~~~ END ~~~~~~~~~");
        }
    }
}

int8_t AP_ExternalAHRS_HITL::get_port(void) const
{
    if (uart_hitl) {
        return port_num_hitl;
    }
    return -1;
};

// Get model/type name
const char* AP_ExternalAHRS_HITL::get_name() const
{
    return "HITL";
}

bool AP_ExternalAHRS_HITL::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return (now - last_ins_pkt < 140);
}

bool AP_ExternalAHRS_HITL::initialised(void) const
{
    return last_ins_pkt != 0;
}

bool AP_ExternalAHRS_HITL::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "MINS unhealthy");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_HITL::get_filter_status(nav_filter_status &status) const
{
    uint32_t now = AP_HAL::millis();
    memset(&status, 0, sizeof(status));
    if (last_ins_pkt != 0) {
        status.flags.initalized = 1;
    }
    if (healthy() && last_ins_pkt != 0) {
        status.flags.attitude = 1;
    }
    if (now - last_gps_pkt < 88) {
        status.flags.horiz_pos_rel = 1;
        status.flags.horiz_pos_abs = 1;
        status.flags.using_gps = 1;
    }
    if (now - last_gps_pkt < 88) {
        status.flags.horiz_vel = 1;
    }
    if (now - last_ins_pkt < 88) {
        status.flags.vert_pos = 1;
        status.flags.vert_vel = 1;
    }
}

bool AP_ExternalAHRS_HITL::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    return false;
}

void AP_ExternalAHRS_HITL::update_imu_post()
{
    if (AP_HAL::millis() - _last_imu_post_ms > 10) {
        _last_imu_post_ms = AP_HAL::millis();
    } else {
        return;
    }

    {
        // WITH_SEMAPHORE(state.sem);
        AP::ins().handle_external(frontend.imu_data);
        AP::compass().handle_external(frontend.mag_data);

        frontend.baro_data.instance = 0;
        AP::baro().handle_external(frontend.baro_data);
        frontend.baro_data.instance = 1;
        AP::baro().handle_external(frontend.baro_data);
        frontend.baro_data.instance = 2;
        AP::baro().handle_external(frontend.baro_data);
    }
}

void AP_ExternalAHRS_HITL::update_actuator_controls()
{
    if (AP_HAL::millis() - _last_srv_post_ms > 2) {
        _last_srv_post_ms = AP_HAL::millis();
    } else {
        return;
    }

    hil_actuator_controls_packet.time_usec = AP_HAL::micros();
    for (uint8_t i_mot = 0; i_mot < 16; i_mot++) {
        hil_actuator_controls_packet.controls[i_mot] = SRV_Channels::get_output_scaled_norm(i_mot);
    }
    hil_actuator_controls_packet.mode |= MAV_MODE_FLAG_HIL_ENABLED;

    mavlink_message_t msg;
    UNUSED_RESULT(mavlink_msg_hil_actuator_controls_encode(gcs().sysid_this_mav(),
                                        0,
                                        &msg, &hil_actuator_controls_packet));
    send_mavlink_message(&msg);
}

void AP_ExternalAHRS_HITL::update_heartbeat()
{
    if (AP_HAL::millis() - _last_hbt_post_ms > 1000) {
        _last_hbt_post_ms = AP_HAL::millis();
    } else {
        return;
    }

    mavlink_message_t msg;
    UNUSED_RESULT(mavlink_msg_heartbeat_encode(gcs().sysid_this_mav(),
                                        0,
                                        &msg, &AP::fd_data().heartbeat_packet));
    send_mavlink_message(&msg);
}


void AP_ExternalAHRS_HITL::send_mavlink_message(mavlink_message_t *msg)
{
    if (uart_hitl == nullptr) {return;}

    uint8_t ck[2];

    ck[0] = (uint8_t)(msg->checksum & 0xFF);
    ck[1] = (uint8_t)(msg->checksum >> 8);
    // XXX use the right sequence here

    uint8_t header_len;
    // uint8_t signature_len;
    
    if (msg->magic == MAVLINK_STX_MAVLINK1) {
        header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1;
        // signature_len = 0;
        // we can't send the structure directly as it has extra mavlink2 elements in it
        uint8_t buf[MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1];
        buf[0] = msg->magic;
        buf[1] = msg->len;
        buf[2] = msg->seq;
        buf[3] = msg->sysid;
        buf[4] = msg->compid;
        buf[5] = msg->msgid & 0xFF;
        uart_hitl->write(buf, header_len);
    } else {
        header_len = MAVLINK_CORE_HEADER_LEN + 1;
        // signature_len = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED)?MAVLINK_SIGNATURE_BLOCK_LEN:0;
        uint8_t buf[MAVLINK_CORE_HEADER_LEN + 1];
        buf[0] = msg->magic;
        buf[1] = msg->len;
        buf[2] = msg->incompat_flags;
        buf[3] = msg->compat_flags;
        buf[4] = msg->seq;
        buf[5] = msg->sysid;
        buf[6] = msg->compid;
        buf[7] = msg->msgid & 0xFF;
        buf[8] = (msg->msgid >> 8) & 0xFF;
        buf[9] = (msg->msgid >> 16) & 0xFF;
        uart_hitl->write(buf, header_len);
    }

    uart_hitl->write((uint8_t *)_MAV_PAYLOAD(msg), msg->len);
    uart_hitl->write((uint8_t *)ck, 2);
}

#endif // AP_EXTERNAL_AHRS_HITL_ENABLED
