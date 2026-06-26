#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_SITL_ENABLED

#include "AP_ExternalAHRS_SITL.h"
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

AP_ExternalAHRS_SITL::AP_ExternalAHRS_SITL(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
{
    if (_sitl == nullptr) {
        _sitl = AP::sitl();
        if (_sitl == nullptr) {
            AP_BoardConfig::allocation_error("Failed to allocate ExternalAHRS SITL update thread");
            return;
        }
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_SITL::update_thread, void), "ESITL", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Failed to allocate ExternalAHRS SITL update thread");
    }

    hal.scheduler->delay(5000);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SITL ExternalAHRS initialised");
}

void AP_ExternalAHRS_SITL::update_thread(void)
{
    hal.scheduler->delay(5000);
    if (_sitl == nullptr) {
        _sitl = AP::sitl();
        if (_sitl == nullptr) {
            AP_BoardConfig::allocation_error("Failed to allocate ExternalAHRS SITL update thread");
            return;
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
        build_packet_ins();
        if (do_print) {
            if (frontend.debug_print.get()>0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "build_packet_ins");
            }
        }

        build_packet_air();
        if (do_print) {
            if (frontend.debug_print.get()>0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "build_packet_air");
            }
        }
        hal.scheduler->delay_microseconds(1000);
    }
}

// Builds packets by looking at each individual byte, once a full packet has been read in it checks the checksum then handles the packet.
void AP_ExternalAHRS_SITL::build_packet_ins()
{

    if (_sitl == nullptr) {
        return;
    }

    static uint32_t _last_read = AP_HAL::millis();
    if (AP_HAL::millis() - _last_read > 5) {
        _last_read = AP_HAL::millis();
    } else {
        return;
    }

    handle_imu();
    // handle_gps();
    if (frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::IMU)) {
        post_imu();
    }
    handle_ahrs();
    // if (frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::GPS)) {
    //     post_gps();
    // }


        // if (AP_HAL::millis() - _last_post > 1000) {
        //     _last_post = AP_HAL::millis();
        //     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PORT IN : %x", b);
        // }
}

void AP_ExternalAHRS_SITL::build_packet_air()
{
    if (_sitl == nullptr) {
        return;
    }

    static uint32_t _last_read = AP_HAL::millis();
    if (AP_HAL::millis() - _last_read > 5) {
        _last_read = AP_HAL::millis();
    } else {
        return;
    }

    handle_baro();
    handle_airspeed();
    if (frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::BARO)) {
        post_baro();
    }
    post_airspeed();

}

// Collects data from an imu packet into `imu_data`
void AP_ExternalAHRS_SITL::handle_imu()
{
    last_ins_pkt = AP_HAL::millis();

    const struct SITL::sitl_fdm &fdm = _sitl->state;

    imu_data.accel = Vector3f(fdm.xAccel,fdm.yAccel,fdm.zAccel);
                                    // m/s^2

    imu_data.gyro = Vector3f(radians(fdm.rollRate),
                             radians(fdm.pitchRate),
                             radians(fdm.yawRate));
                             // rad/s
    imu_data.temperature = 0.0f;

    static uint32_t _last_post = AP_HAL::millis();
    static float count = 0.0f;
    count += 1.0f;
    if (AP_HAL::millis() - _last_post > 5000) {
        float dt = (float)(AP_HAL::millis() - _last_post) * 0.001f;
        _last_post = AP_HAL::millis();
        if (frontend.debug_print.get()>0) {
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS accel : (%f, %f, %f)", _msg_ins._msg_1.content.msg.acc_x_mss, _msg_ins._msg_1.content.msg.acc_y_mss, _msg_ins._msg_1.content.msg.acc_z_mss);
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS gyro : (%f, %f, %f)", _msg_ins._msg_1.content.msg.rate_n_degrees, _msg_ins._msg_1.content.msg.rate_e_degrees, _msg_ins._msg_1.content.msg.rate_u_degrees);
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS ERROR1: %d ", int(_msg_ins._msg_1.content.msg.error_code>>16));
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS ERROR2: %d ", int(_msg_ins._msg_1.content.msg.error_code&0x0000ffff));
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS Rate [%0.1f Hz]", count/dt);
            count = 0.0f;
        }
    }

}

// Posts data from an imu packet to `state` and `handle_external` methods
void AP_ExternalAHRS_SITL::post_imu()
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
void AP_ExternalAHRS_SITL::handle_gps()
{
    // last_gps_pkt = AP_HAL::millis();

    // gps_data.gps_week                    = (0XFF);
    // gps_data.ms_tow                      = (AP_HAL::millis());
    // gps_data.fix_type                    = (_msg_ins._msg_1.content.msg.gps_fix_state);
    // gps_data.satellites_in_view          = ((uint8_t)_msg_ins._msg_1.content.msg.gps_numstat);
    // gps_data.horizontal_pos_accuracy     = (1.0f);
    // gps_data.vertical_pos_accuracy       = (1.0f);
    // gps_data.horizontal_vel_accuracy     = (1.0f);
    // gps_data.hdop                        = (0.01f*(float)_msg_ins._msg_1.content.msg.gps_hdop);
    // gps_data.vdop                        = (0.01f*(float)_msg_ins._msg_1.content.msg.gps_vdop);
    // gps_data.longitude                   = (_msg_ins._msg_1.content.msg.gps_lng);
    // gps_data.latitude                    = (_msg_ins._msg_1.content.msg.gps_lat);
    // gps_data.msl_altitude                = (_msg_ins._msg_1.content.msg.gps_alt_mm/10);
    // gps_data.ned_vel_north               = (_msg_ins._msg_1.content.msg.gps_vel_n_ms_o4/100);
    // gps_data.ned_vel_down                = (-_msg_ins._msg_1.content.msg.gps_vel_u_ms_o2);
    // gps_data.ned_vel_east                = (_msg_ins._msg_1.content.msg.gps_vel_e_ms_o4/100);
    // gps_data.gps_yaw                     = (_msg_ins._msg_1.content.msg.yaw_micro_deg/1000);
    // gps_data.gps_yaw_time_ms             = (AP_HAL::millis());
    // gps_data.gps_yaw_configured          = (true);
    // gps_data.gps_yaw_accuracy            = (5.0f);
    // gps_data.have_gps_yaw                = (true);
    // gps_data.have_gps_yaw_accuracy       = (true);

    // static uint32_t _last_post = AP_HAL::millis();
    // if (AP_HAL::millis() - _last_post > 5000) {
    //     _last_post = AP_HAL::millis();
    //     if (frontend.debug_print.get()>0) {
    //         GCS_SEND_TEXT(MAV_SEVERITY_INFO, "gps fix %d", _msg_ins._msg_1.content.msg.gps_fix_state);
    //         GCS_SEND_TEXT(MAV_SEVERITY_INFO, "gps lng %d | lat %d", int(_msg_ins._msg_1.content.msg.gps_lng), int(_msg_ins._msg_1.content.msg.gps_lat));
    //     }
    // }
}

// Posts data from an gps packet to `state` and `handle_external` methods
void AP_ExternalAHRS_SITL::post_gps()
{
    if (AP_HAL::millis() - _last_gps_post_ms < 100) {
        return;
    }
    _last_gps_post_ms = AP_HAL::millis();
    AP::gps().handle_external(frontend.gps_data, 0);
    // printf("GPS post \n");
}

void AP_ExternalAHRS_SITL::handle_ahrs()
{
    {
        WITH_SEMAPHORE(state.sem);
        last_ahrs_pkt = AP_HAL::millis();

        const struct SITL::sitl_fdm &fdm = _sitl->state;

        state.accel = imu_data.accel;
        state.gyro = imu_data.gyro;
        state.quat.from_euler(radians((float)fdm.rollDeg), radians((float)fdm.pitchDeg), radians((float)fdm.yawDeg));
        state.location.lng = (int32_t)(fdm.longitude*1e7);
        state.location.lat = (int32_t)(fdm.latitude*1e7);
        state.location.set_alt_cm(fdm.altitude*100.f, Location::AltFrame::ABSOLUTE);
        state.velocity = Vector3f(fdm.speedN,fdm.speedE,fdm.speedD);

        state.have_quaternion = true;
        state.have_location = (state.location.lng != 0) && (state.location.lat != 0);
        state.have_velocity = (state.location.lng != 0) && (state.location.lat != 0);

        state.last_location_update_us = AP_HAL::micros();

        if (frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::GPS)) {
            //fake gps
            frontend.gps_data.gps_week                    = (0XFF);
            frontend.gps_data.ms_tow                      = (AP_HAL::millis());
            frontend.gps_data.fix_type                    = (AP_GPS_FixType)(state.have_location?3:1);
            frontend.gps_data.satellites_in_view          = (99);
            frontend.gps_data.horizontal_pos_accuracy     = (1.0f);
            frontend.gps_data.vertical_pos_accuracy       = (1.0f);
            frontend.gps_data.horizontal_vel_accuracy     = (1.0f);
            frontend.gps_data.hdop                        = (1.0f);
            frontend.gps_data.vdop                        = (1.0f);
            frontend.gps_data.longitude                   = (state.location.lng);
            frontend.gps_data.latitude                    = (state.location.lat);
            frontend.gps_data.msl_altitude                = (state.location.alt);
            frontend.gps_data.ned_vel_north               = (state.velocity.x);
            frontend.gps_data.ned_vel_down                = (state.velocity.z);
            frontend.gps_data.ned_vel_east                = (state.velocity.y);
            frontend.gps_data.gps_yaw                     = ((float)fdm.yawDeg);
            frontend.gps_data.gps_yaw_time_ms             = (AP_HAL::millis());
            frontend.gps_data.gps_yaw_configured          = (true);
            frontend.gps_data.gps_yaw_accuracy            = (5.0f);
            frontend.gps_data.have_gps_yaw                = (true);
            frontend.gps_data.have_gps_yaw_accuracy       = (true);
            frontend.gps_data.ground_speed                = (float)state.velocity.xy().length();
            frontend.gps_data.ground_course               = wrap_360(degrees(atan2f(state.velocity.y, state.velocity.x)));
            post_gps();
        }

    }



    if (!state.have_origin && state.have_location) {
        state.origin.lng = state.location.lng;
        state.origin.lat = state.location.lat;
        state.origin.set_alt_cm(state.location.alt, Location::AltFrame::ABSOLUTE);
        state.have_origin = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS state origin set");
    }

    static uint32_t _last_post = AP_HAL::millis();
    if (AP_HAL::millis() - _last_post > 3000) {
        _last_post = AP_HAL::millis();
        if (frontend.debug_print.get()>0) {
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS fix %d | ok %d", _msg_ins._msg_1.content.msg.gps_fix_state, _msg_ins._msg_1.content.msg.gps_ok);
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS lng %d | lat %d", int(_msg_ins._msg_1.content.msg.lng), int(_msg_ins._msg_1.content.msg.lat));
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS alt %d", int(state.location.alt));
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS state.have_location %d", state.have_location);
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS roll %d | pitch %d | yaw %d", int(_msg_ins._msg_1.content.msg.roll_micro_deg), int(_msg_ins._msg_1.content.msg.pitch_micro_deg), int(_msg_ins._msg_1.content.msg.yaw_micro_deg));
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS vn %d | ve %d | vu %d", int(_msg_ins._msg_1.content.msg.vel_n), int(_msg_ins._msg_1.content.msg.vel_e), int(_msg_ins._msg_1.content.msg.vel_u));
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS gyror : (%f, %f, %f)", imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z);
        }
    }
}

// Collects data from an imu packet into `baro_data`
void AP_ExternalAHRS_SITL::handle_baro()
{
    // last_baro_pkt = AP_HAL::millis();

    // const struct SITL::sitl_fdm &fdm = _sitl->state;

    baro_data.instance = 0;
    baro_data.pressure_pa = (0.0f);
    baro_data.temperature = (0.0f);

    // bara_alt.instance = 0;
    // bara_alt.altitude = 

    static uint32_t _last_post = AP_HAL::millis();
    static float count = 0.0f;
    count += 1.0f;
    if (AP_HAL::millis() - _last_post > 5000) {
        float dt = (float)(AP_HAL::millis() - _last_post) * 0.001f;
        _last_post = AP_HAL::millis();
        if (frontend.debug_print.get()>0) {
            // gcs().send_text(MAV_SEVERITY_INFO, "baro ps: %f | %f", baro_data.pressure_pa, (float)_msg_air._msg_1.content.msg.ps);
            // gcs().send_text(MAV_SEVERITY_INFO, "baro ts: %f | %f", baro_data.temperature, (float)_msg_air._msg_1.content.msg.ts);
            // gcs().send_text(MAV_SEVERITY_INFO, "baro AOAt1: %f, AOAt2: %f ", ((float)_msg_air._msg_1.content.msg.aoat1/128.f), ((float)_msg_air._msg_1.content.msg.aoat2/128.f));
            // gcs().send_text(MAV_SEVERITY_INFO, "baro AOSt1: %f, AOSt2: %f ", ((float)_msg_air._msg_1.content.msg.aost1/128.f), ((float)_msg_air._msg_1.content.msg.aost2/128.f));
            // gcs().send_text(MAV_SEVERITY_INFO, "baro ERROR: %d ", _msg_air._msg_1.content.msg.faultword);
            gcs().send_text(MAV_SEVERITY_INFO, "baro Rate [%0.1f Hz]", count/dt);
            count = 0.0f;
        }
    }
}

// Posts data from an baro packet to `state` and `handle_external` methods
void AP_ExternalAHRS_SITL::post_baro()
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
void AP_ExternalAHRS_SITL::handle_airspeed()
{
    // last_airspeed_pkt = AP_HAL::millis();
    const struct SITL::sitl_fdm &fdm = _sitl->state;

    airspeed_data.differential_pressure = 0.0f;
    airspeed_data.temperature = 0.0;
    airspeed_data.airspeed = fdm.airspeed;


    static uint32_t _last_post = AP_HAL::millis();
    if (AP_HAL::millis() - _last_post > 5000) {
        _last_post = AP_HAL::millis();
        if (frontend.debug_print.get()>0) {
            // gcs().send_text(MAV_SEVERITY_INFO, "airspeed ps: %f", (float)_msg_air._msg_1.content.msg.ps);
            // gcs().send_text(MAV_SEVERITY_INFO, "airspeed qc: %f", (float)_msg_air._msg_1.content.msg.qc);
            // gcs().send_text(MAV_SEVERITY_INFO, "airspeed vi: %f | %f", rev_airspeed, (float)_msg_air._msg_1.content.msg.vi);
        }
    }
}

// Posts data from an airspeed packet to `state` and `handle_external` methods
void AP_ExternalAHRS_SITL::post_airspeed()
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

// Get model/type name
const char* AP_ExternalAHRS_SITL::get_name() const
{
    return "ESITL";
}

bool AP_ExternalAHRS_SITL::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return (now - last_ahrs_pkt < 140);
}

bool AP_ExternalAHRS_SITL::initialised(void) const
{
    return last_ins_pkt != 0;
}

bool AP_ExternalAHRS_SITL::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "SITL unhealthy");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_SITL::get_filter_status(nav_filter_status &status) const
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
        status.flags.pred_horiz_pos_rel = true;
        status.flags.pred_horiz_pos_abs = true;
        status.flags.using_gps = true;
    }
    if (now - last_ahrs_pkt < 140 && state.have_velocity) {
        status.flags.horiz_vel = 1;
    }
    if (now - last_ahrs_pkt < 140) {
        status.flags.vert_pos = 1;
        status.flags.vert_vel = 1;
    }
}

bool AP_ExternalAHRS_SITL::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    velVar = 0;
    posVar = 0;
    hgtVar = 0;
    magVar.zero();
    tasVar = 0;

    return false;
}

int8_t AP_ExternalAHRS_SITL::get_port(void) const
{
    return 1;
};

#endif // AP_EXTERNAL_AHRS_SITL_ENABLED
