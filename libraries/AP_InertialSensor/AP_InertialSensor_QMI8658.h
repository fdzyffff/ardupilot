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
/*
  the BMI088 is unusual as it has separate chip-select for accel and
  gyro, which means it needs two Device pointers
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_QMI8658 : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev_qmi8658,
                                            enum Rotation rotation);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;
    bool update() override;

private:
    AP_InertialSensor_QMI8658(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev_qmi8658,
                             enum Rotation rotation);

    /*
     initialise hardware layer
     */
    bool qmi8658_init();

    /*
      initialise driver
     */
    bool init();

    /*
      read data from the FIFOs
     */
    void read_fifo_qmi8658();
    // /*
    //   read from registers, special SPI handling needed
    //  */
    // bool read_registers(uint8_t reg, uint8_t *data, uint8_t len);

    // /*
    //   write to register with retries
    //  */
    // bool write_register(uint8_t reg, uint8_t v);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev_qmi8658;

    uint8_t accel_instance;
    uint8_t gyro_instance;
    enum Rotation _rotation;
    uint8_t temperature_counter;
    float gyro_scale;
    float accel_scale;

    struct qmi8658_fifo_accgyr {
        uint16_t accel[3];
        uint16_t gyro[3];
    };

    struct qmi8658_fifo_accgyrmag {
        uint16_t accel[3];
        uint16_t gyro[3];
        uint16_t mag[3];
    };

};
