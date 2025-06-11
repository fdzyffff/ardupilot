/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  driver for ST VL53L5CX lidar

  Many thanks to Pololu, https://github.com/pololu/VL53L5CX-arduino and
  the ST example code
 */
#include "AP_RangeFinder_VL53L5CX.h"

#if AP_RANGEFINDER_VL53L5CX_ENABLED

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

static const uint8_t MEASUREMENT_TIME_MS = 50; // Start continuous readings at a rate of one measurement every 50 ms

AP_RangeFinder_VL53L5CX::AP_RangeFinder_VL53L5CX(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev)
    : AP_RangeFinder_Backend(_state, _params)
    , dev(std::move(_dev)) {}

/*
   detect if a VL53L5CX rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_VL53L5CX::detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_VL53L5CX *sensor
        = new AP_RangeFinder_VL53L5CX(_state, _params, std::move(dev));

    if (!sensor) {
        delete sensor;
        return nullptr;
    }

    sensor->dev->get_semaphore()->take_blocking();

    if (!sensor->check_id() || !sensor->init()) {
        sensor->dev->get_semaphore()->give();
        delete sensor;
        return nullptr;
    }

    sensor->dev->get_semaphore()->give();

    return sensor;
}

// check sensor ID registers
bool AP_RangeFinder_VL53L5CX::check_id(void)
{
    uint8_t status = 0;
    uint8_t device_id, revision_id;


    if (!dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NO DEV VL53L5CX53L5CX");
        return true;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BUS ADD 0x%x\n", dev->get_bus_address());


    status |= write_register(0x7fff, 0x00);
    status |= read_register(0, device_id);
    status |= read_register(1, revision_id);
    status |= write_register(0x7fff, 0x02);

    // if(status)
    // {
    //     GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "FIND VL53L5CX53L5CX");
    // } else {
    //     GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "[%x, %x] VL53L5CX53L5CX", device_id, revision_id);
    //     return false;
    // }

    if((device_id == (uint8_t)0xF0) && (revision_id == (uint8_t)0x02))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Detected VL53L5CX on bus 0x%x\n", (uint8_t)dev->get_bus_id());
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[%d], No VL53L5CX53L5CX", status);
        return false;
    }
    printf("Detected VL53L5CX on bus 0x%x\n", dev->get_bus_id());
    return true;
}

bool AP_RangeFinder_VL53L5CX::reset(void) {
    // if (dev->get_bus_id()!=0x29) {
    //     // if sensor is on a different port than the default do not  reset sensor otherwise we will lose the addess.
    //     // we assume it is already confirgured.
    //     return true;
    // }
    // if (!write_register(SOFT_RESET, 0x00)) {
    //     return false;
    // }
    // hal.scheduler->delay_microseconds(100);
    // if (!write_register(SOFT_RESET, 0x01)) {
    //     return false;
    // }
    // hal.scheduler->delay(1000);
    return true;
}

/*
  initialise sensor
 */
bool AP_RangeFinder_VL53L5CX::init()
{
    // we need to do resets and delays in order to configure the sensor, don't do this if we are trying to fast boot
    if (hal.util->was_watchdog_armed()) {
        return false;
    }

    uint8_t status = 0;
    status = vl53l5cx_set_resolution(&(pL5obj->Dev), VL53L5CX_RESOLUTION_4X4);
    status |= vl53l5cx_set_ranging_mode(&(pL5obj->Dev), VL53L5CX_RANGING_MODE_CONTINUOUS);
    status |= vl53l5cx_set_integration_time_ms(&(pL5obj->Dev), TIMING_BUDGET);
    status |= vl53l5cx_set_ranging_frequency_hz(&(pL5obj->Dev), RANGING_FREQUENCY);
    if (status != VL53L5CX_STATUS_OK)
    {
    printf("ERROR : Configuration programming error!\n\n");
    while (1);
    }

    // call timer() every MEASUREMENT_TIME_MS. We expect new data to be available every MEASUREMENT_TIME_MS
    dev->register_periodic_callback(MEASUREMENT_TIME_MS * 1000,
                                    FUNCTOR_BIND_MEMBER(&AP_RangeFinder_VL53L5CX::timer, void));

    return true;
}

uint8_t AP_RangeFinder_VL53L5CX::vl53l5cx_set_resolution(
  uint8_t       resolution)
{
  uint8_t status = VL53L5CX_STATUS_OK;

  switch (resolution) {
    case VL53L5CX_RESOLUTION_4X4:
      status |= vl53l5cx_dci_read_data(
                  p_dev->temp_buffer,
                  VL53L5CX_DCI_DSS_CONFIG, 16);
      p_dev->temp_buffer[0x04] = 64;
      p_dev->temp_buffer[0x06] = 64;
      p_dev->temp_buffer[0x09] = 4;
      status |= vl53l5cx_dci_write_data(
                  p_dev->temp_buffer,
                  VL53L5CX_DCI_DSS_CONFIG, 16);

      status |= vl53l5cx_dci_read_data(
                  p_dev->temp_buffer,
                  VL53L5CX_DCI_ZONE_CONFIG, 8);
      p_dev->temp_buffer[0x00] = 4;
      p_dev->temp_buffer[0x01] = 4;
      p_dev->temp_buffer[0x04] = 8;
      p_dev->temp_buffer[0x05] = 8;
      status |= vl53l5cx_dci_write_data(
                  p_dev->temp_buffer,
                  VL53L5CX_DCI_ZONE_CONFIG, 8);
      break;

    case VL53L5CX_RESOLUTION_8X8:
      status |= vl53l5cx_dci_read_data(
                  p_dev->temp_buffer,
                  VL53L5CX_DCI_DSS_CONFIG, 16);
      p_dev->temp_buffer[0x04] = 16;
      p_dev->temp_buffer[0x06] = 16;
      p_dev->temp_buffer[0x09] = 1;
      status |= vl53l5cx_dci_write_data(
                  p_dev->temp_buffer,
                  VL53L5CX_DCI_DSS_CONFIG, 16);

      status |= vl53l5cx_dci_read_data(
                  p_dev->temp_buffer,
                  VL53L5CX_DCI_ZONE_CONFIG, 8);
      p_dev->temp_buffer[0x00] = 8;
      p_dev->temp_buffer[0x01] = 8;
      p_dev->temp_buffer[0x04] = 4;
      p_dev->temp_buffer[0x05] = 4;
      status |= vl53l5cx_dci_write_data(
                  p_dev->temp_buffer,
                  VL53L5CX_DCI_ZONE_CONFIG, 8);

      break;

    default:
      status = VL53L5CX_STATUS_INVALID_PARAM;
      break;
  }

  status |= _vl53l5cx_send_offset_data(resolution);
  status |= _vl53l5cx_send_xtalk_data(resolution);

  return status;
}


uint8_t AP_RangeFinder_VL53L5CX::vl53l5cx_dci_read_data(
  uint8_t       *data,
  uint32_t      index,
  uint16_t      data_size)
{
  int16_t i;
  uint8_t status = VL53L5CX_STATUS_OK;
  uint32_t rd_size = (uint32_t) data_size + (uint32_t)12;
  uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00,
                   0x00, 0x00, 0x00, 0x0f,
                   0x00, 0x02, 0x00, 0x08
                  };

  /* Check if tmp buffer is large enough */
  if ((data_size + (uint16_t)12) > (uint16_t)VL53L5CX_TEMPORARY_BUFFER_SIZE) {
    status |= VL53L5CX_STATUS_ERROR;
  } else {
    cmd[0] = (uint8_t)(index >> 8);
    cmd[1] = (uint8_t)(index & (uint32_t)0xff);
    cmd[2] = (uint8_t)((data_size & (uint16_t)0xff0) >> 4);
    cmd[3] = (uint8_t)((data_size & (uint16_t)0xf) << 4);

    /* Request data reading from FW */
    status |= WrMulti(&(p_dev->platform),
                      (VL53L5CX_UI_CMD_END - (uint16_t)11), cmd, sizeof(cmd));
    status |= _vl53l5cx_poll_for_answer(4, 1,
                                        VL53L5CX_UI_CMD_STATUS,
                                        0xff, 0x03);

    /* Read new data sent (4 bytes header + data_size + 8 bytes footer) */
    status |= RdMulti(&(p_dev->platform), VL53L5CX_UI_CMD_START,
                      p_dev->temp_buffer, rd_size);
    SwapBuffer(p_dev->temp_buffer, data_size + (uint16_t)12);

    /* Copy data from FW into input structure (-4 bytes to remove header) */
    for (i = 0 ; i < (int16_t)data_size; i++) {
      data[i] = p_dev->temp_buffer[i + 4];
    }
  }

  return status;
}

uint8_t VL53L5CX::vl53l5cx_dci_write_data(
  uint8_t       *data,
  uint32_t      index,
  uint16_t      data_size)
{
  uint8_t status = VL53L5CX_STATUS_OK;
  int16_t i;

  uint8_t headers[] = {0x00, 0x00, 0x00, 0x00};
  uint8_t footer[] = {0x00, 0x00, 0x00, 0x0f, 0x05, 0x01,
                      (uint8_t)((data_size + (uint16_t)8) >> 8),
                      (uint8_t)((data_size + (uint16_t)8) & (uint8_t)0xFF)
                     };

  uint16_t address = (uint16_t)VL53L5CX_UI_CMD_END -
                     (data_size + (uint16_t)12) + (uint16_t)1;

  /* Check if cmd buffer is large enough */
  if ((data_size + (uint16_t)12)
      > (uint16_t)VL53L5CX_TEMPORARY_BUFFER_SIZE) {
    status |= VL53L5CX_STATUS_ERROR;
  } else {
    headers[0] = (uint8_t)(index >> 8);
    headers[1] = (uint8_t)(index & (uint32_t)0xff);
    headers[2] = (uint8_t)(((data_size & (uint16_t)0xff0) >> 4));
    headers[3] = (uint8_t)((data_size & (uint16_t)0xf) << 4);

    /* Copy data from structure to FW format (+4 bytes to add header) */
    SwapBuffer(data, data_size);
    for (i = (int16_t)data_size - (int16_t)1 ; i >= 0; i--) {
      p_dev->temp_buffer[i + 4] = data[i];
    }

    /* Add headers and footer */
    (void)memcpy(&p_dev->temp_buffer[0], headers, sizeof(headers));
    (void)memcpy(&p_dev->temp_buffer[data_size + (uint16_t)4],
                 footer, sizeof(footer));

    /* Send data to FW */
    status |= WrMulti(&(p_dev->platform), address,
                      p_dev->temp_buffer,
                      (uint32_t)((uint32_t)data_size + (uint32_t)12));
    status |= _vl53l5cx_poll_for_answer(4, 1,
                                        VL53L5CX_UI_CMD_STATUS, 0xff, 0x03);

    SwapBuffer(data, data_size);
  }

  return status;
}


// read - return last value measured by sensor
bool AP_RangeFinder_VL53L5CX::get_reading(uint16_t &reading_mm)
{
    reading_mm = 1000.f;

    if (!dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NO DEV VL53L5CX53L5CX");
        return true;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BUS ADD 0x%x\n", (uint8_t)dev->get_bus_id());

    uint8_t status = 0;
    uint8_t tmp[4];

    status |= read_registermulti(0x0, tmp, 4);

    bool isready = (tmp[0] != (uint8_t)255) && (tmp[1] == (uint8_t)0x5) && ((tmp[2] & (uint8_t)0x5) == (uint8_t)0x5) && ((tmp[3] & (uint8_t)0x10) == (uint8_t)0x10);

    if (!isready) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NOT READY VL53L5CX53L5CX");
        return false;
    }

    status = 0;
    status |= read_registermulti(0x0, tmp, 4);

    if((device_id == (uint8_t)0xF0) && (revision_id == (uint8_t)0x02))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "FIND VL53L5CX53L5CX");
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "[%d] No VL53L5CX53L5CX", status);
    }

    // GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "STATUS %d VL53L5CX53L5CX", status);


    hal.scheduler->delay(1);
    return true;
}

bool AP_RangeFinder_VL53L5CX::read_register(uint16_t reg, uint8_t &value)
{
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    return dev->transfer(b, 2, &value, 1);
}

bool AP_RangeFinder_VL53L5CX::read_register16(uint16_t reg, uint16_t &value)
{
    uint16_t v = 0;
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    if (!dev->transfer(b, 2, (uint8_t *)&v, 2)) {
        return false;
    }
    value = be16toh(v);
    return true;
}

bool AP_RangeFinder_VL53L5CX::read_registermulti(uint16_t reg, uint8_t* value, uint32_t len)
{
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    if (!dev->transfer(b, 2, value, len)) {
        return false;
    }
    return true;
}


bool AP_RangeFinder_VL53L5CX::write_register(uint16_t reg, uint8_t value)
{
    uint8_t b[3] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), value };
    return dev->transfer(b, 3, nullptr, 0);
}

bool AP_RangeFinder_VL53L5CX::write_register16(uint16_t reg, uint16_t value)
{
    uint8_t b[4] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), uint8_t(value >> 8), uint8_t(value & 0xFF) };
    return dev->transfer(b, 4, nullptr, 0);
}

bool AP_RangeFinder_VL53L5CX::write_register32(uint16_t reg, uint32_t value)
{
    uint8_t b[6] = { uint8_t(reg >> 8),
                     uint8_t(reg & 0xFF),
                     uint8_t((value >> 24) & 0xFF),
                     uint8_t((value >> 16) & 0xFF),
                     uint8_t((value >>  8) & 0xFF),
                     uint8_t((value)       & 0xFF) };
    return dev->transfer(b, 6, nullptr, 0);
}
/*
  timer called at 20Hz
*/
void AP_RangeFinder_VL53L5CX::timer(void)
{
    uint16_t range_mm;
    if ((get_reading(range_mm)) && (range_mm <= 4000)) {
        WITH_SEMAPHORE(_sem);
        sum_mm += range_mm;
        counter++;
    }
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_VL53L5CX::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (counter > 0) {
        state.distance_m = (sum_mm * 0.001f) / counter;
        state.last_reading_ms = AP_HAL::millis();
        update_status();
        sum_mm = 0;
        counter = 0;
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        // if no updates for 0.2s set no-data
        set_status(RangeFinder::Status::NoData);
    }
}

#endif  // AP_RANGEFINDER_VL53L5CX_ENABLED
