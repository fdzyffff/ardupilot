#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_KY2_ENABLED

#include "AP_ExternalAHRS_KY2.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_ExternalAHRS_KY2::var_info[] = {
    // @Param: CFG
    // @DisplayName: KY2 config send mode
    // @Description: 0=Off, 1=Send config once and reset to 0, 2=Send config and keep
    // @Values: 0:Off,1:SendOnce,2:SendKeep
    // @User: Advanced
    AP_GROUPINFO("CFG", 0, AP_ExternalAHRS_KY2, _cfg_mode, 0),

    // @Param: LOG
    // @DisplayName: KY2 log level
    // @Description: 0=Off, 1=Raw (every frame), 2=Filtered (100ms)
    // @Values: 0:Off,1:Raw,2:Filtered
    // @User: Advanced
    AP_GROUPINFO("LOG", 1, AP_ExternalAHRS_KY2, _log_level, 0),

    AP_GROUPEND
};

AP_ExternalAHRS_KY2::AP_ExternalAHRS_KY2(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
{
    AP_SerialManager &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!uart) {
        gcs().send_text(MAV_SEVERITY_INFO, "KY2 ExternalAHRS no UART");
        return;
    }

    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    _config_sent = false;
    _last_location_ms = 0;
    _has_new_frame = false;

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_KY2::update_thread, void), "KY2", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Failed to allocate ExternalAHRS KY2 update thread");
    }

    gcs().send_text(MAV_SEVERITY_INFO, "KY2 ExternalAHRS initialised");
}

void AP_ExternalAHRS_KY2::update_thread(void)
{
    hal.scheduler->delay(5000);
    if (uart) {
        port_open = true;
        uart->begin(baudrate, 1024, 512);
        gcs().send_text(MAV_SEVERITY_INFO, "KY2 SerialProtocol_AHRS %d", int(baudrate));
    }

    while (true) {
        if (!port_open) {
            hal.scheduler->delay(5000);
            continue;
        }

        // Send config command if needed
        if (!_config_sent) {
            send_config();
            _config_sent = true;
        }

        while (uart->available() > 0) {
            uint8_t b = uart->read();
            _msg_KY2.parse(b);

            if (_msg_KY2._msg_1.updated) {
                print_ahrs_state();
                handle_ahrs();
                _msg_KY2._msg_1.updated = false;
                _has_new_frame = true;
            }
        }

        update_log();

        hal.scheduler->delay_microseconds(200);
    }
}

// Send ASCII config commands to KY2 device
void AP_ExternalAHRS_KY2::send_config()
{
    if (_cfg_mode == 0) {
        return;
    }

    if (uart == nullptr) {
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "KY2: sending config commands");

    // Note: commands below are template placeholders.
    // Uncomment and modify as needed for your specific installation.

    // Stop data output before configuring
    // uart->write("AT+SETNO\r\n", 10);
    // hal.scheduler->delay(100);

    // Set output rate (Hz)
    // uart->write("AT+SET_ODR=350\r\n", 16);
    // hal.scheduler->delay(50);

    // Set lever arm: RTK antenna relative to IMU (cm, FRD frame)
    // X: forward positive, Y: right positive, Z: down positive
    // uart->write("AT+SETGPSX=-25\r\n", 16);
    // hal.scheduler->delay(50);
    // uart->write("AT+SETGPSY=0\r\n", 14);
    // hal.scheduler->delay(50);
    // uart->write("AT+SETGPSZ=0\r\n", 14);
    // hal.scheduler->delay(50);

    // Set dual antenna angle
    // 0 = front-back, -90 = left-right
    // uart->write("AT+RTKANGLE=0\r\n", 15);
    // hal.scheduler->delay(50);

    // Single point init (if no differential signal)
    // uart->write("AT+SINGLE=1\r\n", 13);
    // hal.scheduler->delay(50);

    // Set coordinate system (101 = right-forward-up default)
    // uart->write("AT+SET_ORIENT=101\r\n", 18);
    // hal.scheduler->delay(50);

    // Start navigation data output
    // uart->write("AT+SETNAV\r\n", 10);
    // hal.scheduler->delay(50);

    // Save configuration
    // uart->write("AT+SAVE\r\n", 8);
    // hal.scheduler->delay(500);

    gcs().send_text(MAV_SEVERITY_INFO, "KY2: config commands sent");

    // Mode 1: reset to 0 after sending
    if (_cfg_mode == 1) {
        _cfg_mode.set_and_save(0);
    }
}

void AP_ExternalAHRS_KY2::print_ahrs_state()
{
    // Reserved for future state reporting
}

// Map KY2 fix type to AP_GPS_FixType
AP_GPS_FixType AP_ExternalAHRS_KY2::ky2_fix_to_ap(uint8_t ky2_fix)
{
    switch (ky2_fix) {
    case 0:  return AP_GPS_FixType::NO_GPS;
    case 16: return AP_GPS_FixType::FIX_3D;
    case 17: return AP_GPS_FixType::DGPS;
    case 34: return AP_GPS_FixType::RTK_FLOAT;
    case 50: return AP_GPS_FixType::RTK_FIXED;
    default: return AP_GPS_FixType::NO_GPS;
    }
}

void AP_ExternalAHRS_KY2::handle_ahrs()
{
    const FD1_msg_KY2::MSG_Command_1 &msg = _msg_KY2._msg_1.content.msg;

    // Determine if GPS data is valid
    bool gps_valid = (msg.fix_type == 1 || msg.fix_type == 2 || msg.fix_type == 4 || msg.fix_type == 5);

    // Convert units
    // Accel: g → m/s²
    float accel_x_mss = msg.acc_x * GRAVITY_MSS;
    float accel_y_mss = msg.acc_y * GRAVITY_MSS;
    float accel_z_mss = msg.acc_z * GRAVITY_MSS;

    // Gyro: deg/s → rad/s
    float gyro_x_rad = radians(msg.gyro_x);
    float gyro_y_rad = radians(msg.gyro_y);
    float gyro_z_rad = radians(msg.gyro_z);

    // Attitude: deg → rad
    float roll_rad  = radians(msg.roll);
    float pitch_rad = radians(msg.pitch);
    float yaw_rad   = radians(msg.yaw);

    {
        WITH_SEMAPHORE(state.sem);
        last_pkt_ms = AP_HAL::millis();

        // IMU data (always available)
        frontend.imu_data.accel = Vector3f(accel_x_mss, accel_y_mss, accel_z_mss);
        frontend.imu_data.gyro  = Vector3f(gyro_x_rad, gyro_y_rad, gyro_z_rad);
        frontend.imu_data.temperature = msg.temp;

        state.accel = frontend.imu_data.accel;
        state.gyro  = frontend.imu_data.gyro;

        // Attitude (always available)
        state.quat.from_euler(roll_rad, pitch_rad, yaw_rad);
        state.have_quaternion = true;

        // Position and velocity (only when GPS valid)
        if (gps_valid) {
            state.location.lat = msg.lat;
            state.location.lng = msg.lng;
            state.location.set_alt_cm(msg.alt_mm / 10, Location::AltFrame::ABSOLUTE);
            state.velocity = Vector3f(msg.vel_n, msg.vel_e, msg.vel_d);

            state.have_location = true;
            state.have_velocity = true;
            state.last_location_update_us = AP_HAL::micros();
        } else {
            state.have_location = false;
            state.have_velocity = false;
        }

        // GPS data for AP_GPS backend
        if (frontend.has_sensor(AP_ExternalAHRS::AvailableSensor::GPS)) {
            frontend.gps_data.gps_week                = msg.week;
            frontend.gps_data.ms_tow                  = msg.tow_ms;
            frontend.gps_data.fix_type                = gps_valid ? ky2_fix_to_ap(msg.fix_type) : AP_GPS_FixType::NO_GPS;
            frontend.gps_data.satellites_in_view      = gps_valid ? msg.sats : 0;
            frontend.gps_data.horizontal_pos_accuracy = gps_valid ? ((float)msg.pos_accuracy * 0.01f) : 99.0f;
            frontend.gps_data.vertical_pos_accuracy   = gps_valid ? ((float)msg.pos_accuracy * 0.01f) : 99.0f;
            frontend.gps_data.horizontal_vel_accuracy = gps_valid ? 0.1f : 99.0f;
            frontend.gps_data.hdop                    = 1.0f;
            frontend.gps_data.vdop                    = 1.0f;

            // Dual antenna yaw
            if (gps_valid && msg.dual_ant_status == 50) {
                frontend.gps_data.gps_yaw            = msg.dual_heading;
                frontend.gps_data.gps_yaw_time_ms    = AP_HAL::millis();
                frontend.gps_data.gps_yaw_configured = true;
                frontend.gps_data.gps_yaw_accuracy   = 0.1f;
                frontend.gps_data.have_gps_yaw       = true;
                frontend.gps_data.have_gps_yaw_accuracy = true;
            } else {
                frontend.gps_data.gps_yaw_configured      = false;
                frontend.gps_data.have_gps_yaw            = false;
                frontend.gps_data.have_gps_yaw_accuracy   = false;
            }

            if (gps_valid) {
                frontend.gps_data.longitude    = state.location.lng;
                frontend.gps_data.latitude     = state.location.lat;
                frontend.gps_data.msl_altitude = state.location.alt;
                frontend.gps_data.ned_vel_north = state.velocity.x;
                frontend.gps_data.ned_vel_east  = state.velocity.y;
                frontend.gps_data.ned_vel_down  = state.velocity.z;
                frontend.gps_data.ground_speed  = state.velocity.xy().length();
                frontend.gps_data.ground_course = wrap_360(degrees(atan2f(state.velocity.y, state.velocity.x)));
                post_gps();
            }
        }

        post_imu();
    }

    // Set origin after 10s of stable position (once only)
    if (!state.have_origin && state.have_location) {
        if (_last_location_ms == 0) {
            _last_location_ms = AP_HAL::millis();
        }
        if (_last_location_ms != 0 && (AP_HAL::millis() - _last_location_ms > 10000)) {
            state.origin = Location{state.location.lat, state.location.lng, state.location.alt, Location::AltFrame::ABSOLUTE};
            state.have_origin = true;
            _last_location_ms = 0;
            gcs().send_text(MAV_SEVERITY_INFO, "KY2 Origin: %d, %d", (int)state.origin.lat, (int)state.origin.lng);
        }
    }
}

void AP_ExternalAHRS_KY2::post_imu()
{
    AP::ins().handle_external(frontend.imu_data);
}

void AP_ExternalAHRS_KY2::post_gps()
{
    if (AP_HAL::millis() - last_gps_post_ms < 100) {
        return;
    }
    last_gps_post_ms = AP_HAL::millis();
    AP::gps().handle_external(frontend.gps_data, 0);
}

void AP_ExternalAHRS_KY2::update_log()
{
    uint8_t level = _log_level.get();
    if (level == 0) {
        return;
    }

#if HAL_LOGGING_ENABLED
    if (level == 1) {
        // Raw mode: log every frame
        if (!_has_new_frame) {
            return;
        }
        _has_new_frame = false;
    } else {
        // Filtered mode: log every 100ms
        if (AP_HAL::millis() - last_log_ms < 100) {
            return;
        }
    }
    last_log_ms = AP_HAL::millis();

    uint64_t now_us = AP_HAL::micros64();

    // Extract key fields for logging
    const FD1_msg_KY2::MSG_Command_1 &msg = _msg_KY2._msg_1.content.msg;

    // @LoggerMessage: KY21
    // @Description: KY2 Position & Velocity
    // @Field: TimeUS: Time since system startup
    // @Field: Lat: Latitude (deg*1e7)
    // @Field: Lng: Longitude (deg*1e7)
    // @Field: Alt: Altitude (mm)
    // @Field: VN: North velocity (m/s)
    // @Field: VE: East velocity (m/s)
    // @Field: VD: Down velocity (m/s)
    AP::logger().WriteStreaming("KY21", "TimeUS,Lat,Lng,Alt,VN,VE,VD",
                                "s-DDmfff",
                                "F-GGmnnn",
                                "Qiiifff",
                                now_us,
                                msg.lat, msg.lng, msg.alt_mm,
                                msg.vel_n, msg.vel_e, msg.vel_d);

    // @LoggerMessage: KY22
    // @Description: KY2 Attitude & IMU
    // @Field: TimeUS: Time since system startup
    // @Field: Roll: Roll (deg)
    // @Field: Pitch: Pitch (deg)
    // @Field: Yaw: Yaw (deg)
    // @Field: GyrX: Gyro X (deg/s)
    // @Field: GyrY: Gyro Y (deg/s)
    // @Field: GyrZ: Gyro Z (deg/s)
    // @Field: AccX: Accel X (g)
    // @Field: AccY: Accel Y (g)
    // @Field: AccZ: Accel Z (g)
    AP::logger().WriteStreaming("KY22", "TimeUS,Roll,Pitch,Yaw,GyrX,GyrY,GyrZ,AccX,AccY,AccZ",
                                "sdddffffff",
                                "F000000000",
                                "Qfffffffff",
                                now_us,
                                msg.roll, msg.pitch, msg.yaw,
                                msg.gyro_x, msg.gyro_y, msg.gyro_z,
                                msg.acc_x, msg.acc_y, msg.acc_z);
#endif // HAL_LOGGING_ENABLED
}

int8_t AP_ExternalAHRS_KY2::get_port(void) const
{
    if (uart) {
        return port_num;
    }
    return -1;
}

const char* AP_ExternalAHRS_KY2::get_name() const
{
    return "KY2";
}

bool AP_ExternalAHRS_KY2::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return (now - last_pkt_ms < 140);
}

bool AP_ExternalAHRS_KY2::initialised(void) const
{
    return last_pkt_ms != 0;
}

bool AP_ExternalAHRS_KY2::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "KY2 unhealthy");
        return false;
    }
    if (!initialised()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "KY2 not initialised");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_KY2::get_filter_status(nav_filter_status &status) const
{
    uint32_t now = AP_HAL::millis();
    memset(&status, 0, sizeof(status));
    if (last_pkt_ms != 0) {
        status.flags.initalized = 1;
    }
    if (healthy() && last_pkt_ms != 0) {
        status.flags.attitude = 1;
    }
    if (now - last_pkt_ms < 140 && state.have_location) {
        status.flags.horiz_pos_rel = 1;
        status.flags.horiz_pos_abs = 1;
        status.flags.using_gps = 1;
    }
    if (now - last_pkt_ms < 140 && state.have_velocity) {
        status.flags.horiz_vel = 1;
    }
    if (now - last_pkt_ms < 140) {
        status.flags.vert_pos = 1;
        status.flags.vert_vel = 1;
    }
}

bool AP_ExternalAHRS_KY2::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    velVar = 0;
    posVar = 0;
    hgtVar = 0;
    magVar.zero();
    tasVar = 0;
    return false;
}

#endif // AP_EXTERNAL_AHRS_KY2_ENABLED
