#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_KY1_ENABLED

#include "AP_ExternalAHRS_KY1.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_KY1::AP_ExternalAHRS_KY1(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
{
    AP_SerialManager &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!uart) {
        gcs().send_text(MAV_SEVERITY_INFO, "KY1 ExternalAHRS no UART");
        return;
    }

    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_KY1::update_thread, void), "KY1", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Failed to allocate ExternalAHRS KY1 update thread");
    }

    gcs().send_text(MAV_SEVERITY_INFO, "KY1 ExternalAHRS initialised");
}

void AP_ExternalAHRS_KY1::update_thread(void)
{
    hal.scheduler->delay(5000);
    if (uart) {
        port_open = true;
        uart->begin(baudrate, 128, 128);
        gcs().send_text(MAV_SEVERITY_INFO, "KY1 SerialProtocol_AHRS %d", int(baudrate));
    }

    while (true) {
        if (!port_open) {
            hal.scheduler->delay(5000);
            continue;
        }

        while (uart->available() > 0) {
            uint8_t b = uart->read();
            _msg_KY1.parse(b);

            if (_msg_KY1._msg_1.updated) {
                process_packet();
                _msg_KY1._msg_1.updated = false;
            }
        }

        hal.scheduler->delay_microseconds(1000);
    }
}

void AP_ExternalAHRS_KY1::process_packet()
{
    // Verify checksum: bytes 0-19, paired as little-endian uint16, sum low 16 bits

    // Parse data (big-endian int16)
    // Gyro: deg/s → rad/s
    const float gyro_x  = radians((float)(_msg_KY1._msg_1.content.msg.gyro_x) * _msg_KY1.GYRO_SCALE);
    const float gyro_y  = radians((float)(_msg_KY1._msg_1.content.msg.gyro_y) * _msg_KY1.GYRO_SCALE);
    const float gyro_z  = radians((float)(_msg_KY1._msg_1.content.msg.gyro_z) * _msg_KY1.GYRO_SCALE);

    // Accel: g → m/s²
    const float accel_x = (float)(_msg_KY1._msg_1.content.msg.acc_x) * _msg_KY1.ACCEL_SCALE * GRAVITY_MSS;
    const float accel_y = (float)(_msg_KY1._msg_1.content.msg.acc_y) * _msg_KY1.ACCEL_SCALE * GRAVITY_MSS;
    const float accel_z = (float)(_msg_KY1._msg_1.content.msg.acc_z) * _msg_KY1.ACCEL_SCALE * GRAVITY_MSS;

    // Attitude: deg → rad
    const float roll  = radians((float)(_msg_KY1._msg_1.content.msg.angle_roll) * _msg_KY1.ANGLE_SCALE);
    const float pitch = radians((float)(_msg_KY1._msg_1.content.msg.angle_pitch) * _msg_KY1.ANGLE_SCALE);
    const float yaw   = radians((float)(_msg_KY1._msg_1.content.msg.angle_yaw) * _msg_KY1.ANGLE_SCALE);

    // Update frontend imu_data and state
    {
        WITH_SEMAPHORE(state.sem);
        last_pkt_ms = AP_HAL::millis();

        frontend.imu_data.accel = Vector3f(accel_x, accel_y, accel_z);
        frontend.imu_data.gyro  = Vector3f(gyro_x, gyro_y, gyro_z);
        frontend.imu_data.temperature = 0.0f;

        state.accel = frontend.imu_data.accel;
        state.gyro  = frontend.imu_data.gyro;
        state.quat.from_euler(roll, pitch, yaw);
        state.have_quaternion = true;
    }

    // Post IMU data to InertialSensor (caches in handle_external, pushed by periodic post_data thread)
    post_imu();
}

// Posts IMU data to AP_InertialSensor via handle_external (non-blocking cache write)
void AP_ExternalAHRS_KY1::post_imu()
{
    AP::ins().handle_external(frontend.imu_data);
}

int8_t AP_ExternalAHRS_KY1::get_port(void) const
{
    if (uart) {
        return port_num;
    }
    return -1;
}

// Get model/type name
const char* AP_ExternalAHRS_KY1::get_name() const
{
    return "KY1";
}

bool AP_ExternalAHRS_KY1::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return (now - last_pkt_ms < 50);
}

bool AP_ExternalAHRS_KY1::initialised(void) const
{
    return last_pkt_ms != 0;
}

bool AP_ExternalAHRS_KY1::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "KY1 unhealthy");
        return false;
    }
    if (!initialised()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "KY1 not initialised");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_KY1::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    if (last_pkt_ms != 0) {
        status.flags.initalized = 1;
    }
    if (healthy() && last_pkt_ms != 0) {
        status.flags.attitude = 1;
    }
}

bool AP_ExternalAHRS_KY1::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    velVar = 0;
    posVar = 0;
    hgtVar = 0;
    magVar.zero();
    tasVar = 0;
    return false;
}

#endif // AP_EXTERNAL_AHRS_KY1_ENABLED
