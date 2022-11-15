/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_QMI8658.h"
#include "AP_InertialSensor_QMI8658_registers.h"

/*
  device registers, names follow datasheet conventions, with REGA_
  prefix for accel, and REGG_ prefix for gyro
 */

#define QMI8658_BACKEND_SAMPLE_RATE   1000

extern const AP_HAL::HAL& hal;

AP_InertialSensor_QMI8658::AP_InertialSensor_QMI8658(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::Device> _dev_qmi8658,
                                                   enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , dev_qmi8658(std::move(_dev_qmi8658))
    , rotation(_rotation)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_QMI8658::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::Device> dev_qmi8658,
                                enum Rotation rotation)
{
    if (!dev_qmi8658) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_QMI8658(imu, std::move(dev_qmi8658), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_QMI8658::start()
{
    if (!_imu.register_accel(accel_instance, QMI8658_BACKEND_SAMPLE_RATE, dev_qmi8658->get_bus_id_devtype(DEVTYPE_INS_QMI8658)) ||
        !_imu.register_gyro(gyro_instance, QMI8658_BACKEND_SAMPLE_RATE, dev_qmi8658->get_bus_id_devtype(DEVTYPE_INS_QMI8658))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    // setup callbacks
    dev_qmi8658->register_periodic_callback(1000000UL / QMI8658_BACKEND_SAMPLE_RATE,
                                          FUNCTOR_BIND_MEMBER(&AP_InertialSensor_QMI8658::read_fifo_qmi8658, void));
}

// /*
//   read from accelerometer registers, special SPI handling needed
// */
// bool AP_InertialSensor_QMI8658::read_registers(uint8_t reg, uint8_t *data, uint8_t len)
// {
//     // when on I2C we just read normally
//     if (dev_qmi8658->bus_type() != AP_HAL::Device::BUS_TYPE_SPI) {
//         return dev_qmi8658->read_registers(reg, data, len);
//     }
//     // for SPI we need to discard the first returned byte. See
//     // datasheet for explanation
//     uint8_t b[len+2];
//     b[0] = reg | 0x80;
//     memset(&b[1], 0, len+1);
//     if (!dev_qmi8658->transfer(b, len+2, b, len+2)) {
//         return false;
//     }
//     memcpy(data, &b[2], len);
//     return true;
// }

// /*
//   write to accel registers with retries. The SPI sensor may take
//   several tries to correctly write a register
// */
// bool AP_InertialSensor_QMI8658::write_register(uint8_t reg, uint8_t v)
// {
//     for (uint8_t i=0; i<8; i++) {
//         dev_qmi8658->write_register(reg, v);
//         uint8_t v2 = 0;
//         if (read_registers(reg, &v2, 1) && v2 == v) {
//             return true;
//         }
//     }
//     return false;
// }

bool AP_InertialSensor_QMI8658::init()
{
    dev_qmi8658->set_read_flag(0x80);

    return qmi8658_init();
}

bool AP_InertialSensor_QMI8658::AP_InertialSensor_QMI8658::qmi8658_init()
{
    WITH_SEMAPHORE(dev_qmi8658->get_semaphore());

    // check who am I
    uint8_t v;
    if (!dev_qmi8658->read_registers(QMI8658REGISTER::QMI8658REGISTER_WHOAMI, &v, 1) || v != QMI8658_WHOAMI_VAL) {
        return false;
    }

    //四线SPI，串行接口（SPI 或 I2C）地址自动递增, 大端读取，使能内部2M晶振
    if (!dev_qmi8658->write_register(QMI8658REGISTER::QMI8658REGISTER_CTRL1, 0x60, true)) {
        return false;
    }

    //使能陀螺仪、加速度计
    if (!dev_qmi8658->write_register(QMI8658REGISTER::QMI8658REGISTER_CTRL7, 0x03, true)) {
        return false;
    }

    //加速度计自检，±16g，2000Hz采样
    if (!dev_qmi8658->write_register(QMI8658REGISTER::QMI8658REGISTER_CTRL2, QMI8658_ACCRANGE::QMI8658ACCRANGE_16G | QMI8658_ACCODR::QMI8658ACCODR_2000HZ, true)) {
        return false;
    }

    accel_scale = GRAVITY_MSS * 16.0f /32768.0f;

    //陀螺仪自检，±2048dps，2000Hz采样
    if (!dev_qmi8658->write_register(QMI8658REGISTER::QMI8658REGISTER_CTRL3, QMI8658_GYRRANGE::QMI8658GYRRANGE_2048DPS | QMI8658_GYRODR::QMI8658GYRODR_2000HZ, true)) {
        return false;
    }

    gyro_scale = radians(2048.0f)/32768.0f;

    //无磁力仪
    if (!dev_qmi8658->write_register(QMI8658REGISTER::QMI8658REGISTER_CTRL4, 0x00, true)) {
        return false;
    }

    //使能陀螺仪低通滤波, 14.0%采样频率
    if (!dev_qmi8658->write_register(QMI8658REGISTER::QMI8658REGISTER_CTRL5, (0x01 << 4 ) | (0x03 << 5), true)) {
        return false;
    }

    hal.console->printf("QMI8658 found on bus 0x%x\n", (unsigned)dev_qmi8658->get_bus_id());

    return true;
}

/*
  read qmi8658 fifo
 */
void AP_InertialSensor_QMI8658::read_fifo_qmi8658()
{
    uint8_t status0;
    if (!dev_qmi8658->read_registers(QMI8658REGISTER::QMI8658REGISTER_STATUS0, &status0, 1)) {
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
        return;
    }

    // 检查加速度计、陀螺仪是否有新数据
    if (!(status0 & QMI8658STATUS0_ADA) || !(status0 & QMI8658STATUS0_GDA)) {
        return;
    }

    struct qmi8658_fifo_accgyr accgyr;
    if (!dev_qmi8658->read_registers(QMI8658REGISTER_AX_L, (uint8_t *)&accgyr, sizeof(struct qmi8658_fifo_accgyr))) {
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
        return;
    }

    Vector3f accel((int16_t)(accgyr.accel[0]) * accel_scale,
                   (int16_t)(accgyr.accel[1]) * accel_scale,
                   (int16_t)(accgyr.accel[2]) * accel_scale);

    Vector3f gyro((int16_t)(accgyr.gyro[0]) * gyro_scale,
                  (int16_t)(accgyr.gyro[1]) * gyro_scale,
                  (int16_t)(accgyr.gyro[2]) * gyro_scale);

    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);

    if (temperature_counter++ == 100) {
        temperature_counter = 0;
        int16_t t;
        if (!dev_qmi8658->read_registers(QMI8658REGISTER::QMI8658REGISTER_TEMPEARTURE_L, (uint8_t *)&t, sizeof(t))) {
            _inc_accel_error_count(accel_instance);
            _inc_gyro_error_count(gyro_instance);
        } else {
            float temp_degc = 0.00390625 * t;
            _publish_temperature(accel_instance, temp_degc);
        }
    }

    AP_HAL::Device::checkreg reg;
    if (!dev_qmi8658->check_next_register(reg)) {
        log_register_change(dev_qmi8658->get_bus_id(), reg);
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
    }
}

bool AP_InertialSensor_QMI8658::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}
