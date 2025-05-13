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

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_TZ605::AP_ExternalAHRS_TZ605(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
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

void AP_ExternalAHRS_TZ605::update_thread(void)
{
    if (uart_ins) {
        if (!port_open_ins) {
            port_open_ins = true;
            uart_ins->begin(baudrate_ins);
        }
    }

    if (uart_air) {
        if (!port_open_air) {
            port_open_air = true;
            uart_air->begin(baudrate_air);
        }
    }

    while (true) {
        if (port_open_ins) {build_packet_ins();}
        if (port_open_air) {build_packet_air();}
        hal.scheduler->delay_microseconds(100);
    }
}

// Builds packets by looking at each individual byte, once a full packet has been read in it checks the checksum then handles the packet.
void AP_ExternalAHRS_TZ605::build_packet_ins()
{
    WITH_SEMAPHORE(sem);
    uint32_t nbytes = MIN(uart_ins->available(), 2048u);

    // static uint32_t _last_post = AP_HAL::millis();

    while (nbytes--> 0) {
        const int16_t b = uart_ins->read();

        _msg_ins.parse(b);

        if (_msg_ins._msg_1.updated) {
            if (frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::IMU)) {
                handle_imu();
                post_imu();
            }
            if (frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::GPS)) {
                handle_gps();
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
    WITH_SEMAPHORE(sem);
    uint32_t nbytes = MIN(uart_air->available(), 2048u);

    // static uint32_t _last_post = AP_HAL::millis();

    while (nbytes--> 0) {
        const int16_t b = uart_air->read();

        _msg_air.parse(b);

        if (_msg_air._msg_1.updated) {
            if (frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::BARO)) {
                handle_baro();
                post_baro();
                handle_airspeed();
                post_airspeed();
            }
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
                                        // m/s^2
    imu_data.gyro = Vector3f(radians(_msg_ins._msg_1.content.msg.rate_x_degrees),
                             radians(_msg_ins._msg_1.content.msg.rate_y_degrees),
                             radians(_msg_ins._msg_1.content.msg.rate_z_degrees));
                             // rad/s
    imu_data.temperature = 0.0f;
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
    gps_data.latitude                    = (_msg_ins._msg_1.content.msg.gps_lag);
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
        state.accel = imu_data.accel;
        state.gyro = imu_data.gyro;
        state.quat.from_euler(radians(0.001f*(float)_msg_ins._msg_1.content.msg.roll_micro_deg), radians(0.001f*(float)_msg_ins._msg_1.content.msg.pitch_micro_deg), radians(0.001f*(float)_msg_ins._msg_1.content.msg.yaw_micro_deg));
        state.location.lng = _msg_ins._msg_1.content.msg.lng;
        state.location.lat = _msg_ins._msg_1.content.msg.lng;
        state.location.set_alt_cm(_msg_ins._msg_1.content.msg.alt_mm/10, Location::AltFrame::ABSOLUTE);
        state.velocity = Vector3f(0.1f*(float)_msg_ins._msg_1.content.msg.vel_n_mms, 0.1f*(float)_msg_ins._msg_1.content.msg.vel_e_mms, -0.1f*(float)_msg_ins._msg_1.content.msg.vel_u_mms);

        state.have_quaternion = true;
        state.have_location = _msg_ins._msg_1.content.msg.gps_ok || (_msg_ins._msg_1.content.msg.state == 5);
        state.have_velocity = _msg_ins._msg_1.content.msg.gps_ok || (_msg_ins._msg_1.content.msg.state == 5);

        state.last_location_update_us = AP_HAL::micros();;
    }

    if (!state.have_origin && _msg_ins._msg_1.content.msg.gps_ok) {
        state.origin.lng = _msg_ins._msg_1.content.msg.lng;
        state.origin.lat = _msg_ins._msg_1.content.msg.lng;
        state.origin.set_alt_cm(_msg_ins._msg_1.content.msg.alt_mm/10, Location::AltFrame::ABSOLUTE);
        state.have_origin = true;
    }
}

// Collects data from an imu packet into `baro_data`
void AP_ExternalAHRS_TZ605::handle_baro()
{
    // last_baro_pkt = AP_HAL::millis();

    baro_data.instance = 0;
    baro_data.pressure_pa = ((float)_msg_air._msg_1.content.msg.ps/1024.f *1000.f);
    baro_data.temperature = ((float)_msg_air._msg_1.content.msg.ts/16.f);

    gcs().send_text(MAV_SEVERITY_INFO, "pressure_pa: %f | %f", baro_data.pressure_pa, (float)_msg_air._msg_1.content.msg.ps);
    gcs().send_text(MAV_SEVERITY_INFO, "temperature: %f | %f", baro_data.temperature, (float)_msg_air._msg_1.content.msg.ts);
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
    float ps = ((float)_msg_air._msg_1.content.msg.ps/1024.f*(132.0f-14.0f) + 14.0f)*1000.f;
    float qc = ((float)_msg_air._msg_1.content.msg.qc/1024.f*(132.0f-14.0f) + 14.0f)*1000.f;
    airspeed_data.differential_pressure = ps-qc;
    airspeed_data.temperature = ((float)_msg_air._msg_1.content.msg.ts/16.f*(96.0f+72.0f) - 72.0f);
}

// Posts data from an airspeed packet to `state` and `handle_external` methods
void AP_ExternalAHRS_TZ605::post_airspeed()
{
    {
        AP_ExternalAHRS::airspeed_data_message_t airspeed {
            differential_pressure     : airspeed_data.differential_pressure,
            temperature               : airspeed_data.temperature
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
    return (now - last_ins_pkt < 40);
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
    memset(&status, 0, sizeof(status));
    if (last_ins_pkt != 0) {
        status.flags.initalized = 1;
    }
    if (healthy() && last_ins_pkt != 0) {
        status.flags.attitude = 1;
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
