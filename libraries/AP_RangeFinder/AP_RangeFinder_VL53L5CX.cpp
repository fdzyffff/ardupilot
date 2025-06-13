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
        vl53l5cx_start_ranging();
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
    // status = vl53l5cx_set_resolution(&my_dev, VL53L5CX_RESOLUTION_4X4);
    // status |= vl53l5cx_set_ranging_mode(&my_dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
    // status |= vl53l5cx_set_integration_time_ms(&my_dev, TIMING_BUDGET);
    // status |= vl53l5cx_set_ranging_frequency_hz(&my_dev, RANGING_FREQUENCY);
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

// read - return last value measured by sensor
bool AP_RangeFinder_VL53L5CX::get_reading(uint16_t &reading_mm)
{
    reading_mm = 1000.f;

    if (!dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NO DEV VL53L5CX53L5CX");
        return true;
    }

    // GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BUS ADD 0x%x\n", (uint8_t)dev->get_bus_id());

    (void)vl53l5cx_check_data_ready(&my_dev, &NewDataReady);

    if (NewDataReady != 0)
    {
        status = vl53l5cx_get_ranging_data(&my_dev, &data);

        if (status == VL53L5CX_STATUS_OK)
        {
            /*
             Convert the data format to Result format.
             Note that you can print directly from data format
            */
            if (convert_data_format(pL5obj, &data, &Result) < 0)
            {
                printf("convert_data_format failed\n");
            } else {
                ;
            }
        }
    }


    // if((device_id == (uint8_t)0xF0) && (revision_id == (uint8_t)0x02))
    // {
    //     GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "FIND VL53L5CX53L5CX");
    // } else {
    //     GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "[%d] No VL53L5CX53L5CX", status);
    // }

    // GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "STATUS %d VL53L5CX53L5CX", status);


    hal.scheduler->delay(1);
    return true;
}

uint8_t AP_RangeFinder_VL53L5CX::vl53l5cx_start_ranging()
{
    uint8_t resolution, status = VL53L5CX_STATUS_OK;
    uint16_t tmp;
    uint32_t i;
    uint32_t header_config[2] = {0, 0};

    union Block_header *bh_ptr;
    uint8_t cmd[] = {0x00, 0x03, 0x00, 0x00};

    status |= vl53l5cx_get_resolution(p_dev, &resolution);
    p_dev->data_read_size = 0;
    p_dev->streamcount = 255;

    /* Enable mandatory output (meta and common data) */
    uint32_t output_bh_enable[] = {
        0x00000007U,
        0x00000000U,
        0x00000000U,
        0xC0000000U};

    /* Send addresses of possible output */
    uint32_t output[] ={VL53L5CX_START_BH,
        VL53L5CX_METADATA_BH,
        VL53L5CX_COMMONDATA_BH,
        VL53L5CX_AMBIENT_RATE_BH,
        VL53L5CX_SPAD_COUNT_BH,
        VL53L5CX_NB_TARGET_DETECTED_BH,
        VL53L5CX_SIGNAL_RATE_BH,
        VL53L5CX_RANGE_SIGMA_MM_BH,
        VL53L5CX_DISTANCE_BH,
        VL53L5CX_REFLECTANCE_BH,
        VL53L5CX_TARGET_STATUS_BH,
        VL53L5CX_MOTION_DETECT_BH};

    /* Enable selected outputs in the 'platform.h' file */
#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
    output_bh_enable[0] += (uint32_t)8;
#endif
#ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
    output_bh_enable[0] += (uint32_t)16;
#endif
#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
    output_bh_enable[0] += (uint32_t)32;
#endif
#ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
    output_bh_enable[0] += (uint32_t)64;
#endif
#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
    output_bh_enable[0] += (uint32_t)128;
#endif
#ifndef VL53L5CX_DISABLE_DISTANCE_MM
    output_bh_enable[0] += (uint32_t)256;
#endif
#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
    output_bh_enable[0] += (uint32_t)512;
#endif
#ifndef VL53L5CX_DISABLE_TARGET_STATUS
    output_bh_enable[0] += (uint32_t)1024;
#endif
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
    output_bh_enable[0] += (uint32_t)2048;
#endif

    /* Update data size */
    for (i = 0; i < (uint32_t)(sizeof(output)/sizeof(uint32_t)); i++)
    {
        if ((output[i] == (uint8_t)0) 
                    || ((output_bh_enable[i/(uint32_t)32]
                         &((uint32_t)1 << (i%(uint32_t)32))) == (uint32_t)0))
        {
            continue;
        }

        bh_ptr = (union Block_header *)&(output[i]);
        if (((uint8_t)bh_ptr->type >= (uint8_t)0x1) 
                    && ((uint8_t)bh_ptr->type < (uint8_t)0x0d))
        {
            if ((bh_ptr->idx >= (uint16_t)0x54d0) 
                            && (bh_ptr->idx < (uint16_t)(0x54d0 + 960)))
            {
                bh_ptr->size = resolution;
            }
            else
            {
                bh_ptr->size = (uint16_t)((uint16_t)resolution
                                  * (uint16_t)VL53L5CX_NB_TARGET_PER_ZONE);
            }
            p_dev->data_read_size += bh_ptr->type * bh_ptr->size;
        }
        else
        {
            p_dev->data_read_size += bh_ptr->size;
        }
        p_dev->data_read_size += (uint32_t)4;
    }
    p_dev->data_read_size += (uint32_t)24;

    status |= vl53l5cx_dci_write_data(p_dev,
            (uint8_t*)&(output), VL53L5CX_DCI_OUTPUT_LIST,
            (uint16_t)sizeof(output));

    header_config[0] = p_dev->data_read_size;
    header_config[1] = i + (uint32_t)1;

    status |= vl53l5cx_dci_write_data(p_dev,
            (uint8_t*)&(header_config), VL53L5CX_DCI_OUTPUT_CONFIG,
            (uint16_t)sizeof(header_config));

    status |= vl53l5cx_dci_write_data(p_dev,
            (uint8_t*)&(output_bh_enable), VL53L5CX_DCI_OUTPUT_ENABLES,
            (uint16_t)sizeof(output_bh_enable));

    /* Start xshut bypass (interrupt mode) */
    status |= WrByte(0x7fff, 0x00);
    status |= WrByte(0x09, 0x05);
    status |= WrByte(0x7fff, 0x02);

    /* Start ranging session */
    status |= WrMulti(VL53L5CX_UI_CMD_END -
            (uint16_t)(4 - 1), (uint8_t*)cmd, sizeof(cmd));
    status |= _vl53l5cx_poll_for_answer(p_dev, 4, 1,
            VL53L5CX_UI_CMD_STATUS, 0xff, 0x03);

    /* Read ui range data content and compare if data size is the correct one */
    status |= vl53l5cx_dci_read_data(p_dev,
            (uint8_t*)p_dev->temp_buffer, 0x5440, 12);
    (void)memcpy(&tmp, &(p_dev->temp_buffer[0x8]), sizeof(tmp));
    if(tmp != p_dev->data_read_size)
    {
        status |= VL53L5CX_STATUS_ERROR;
    }

    return status;
}

uint8_t AP_RangeFinder_VL53L5CX::vl53l5cx_check_data_ready(
        VL53L5CX_Configuration      *p_dev,
        uint8_t             *p_isReady)
{
    uint8_t status = VL53L5CX_STATUS_OK;

    status |= RdMulti(0x0, p_dev->temp_buffer, 4);

    if((p_dev->temp_buffer[0] != p_dev->streamcount)
            && (p_dev->temp_buffer[0] != (uint8_t)255)
            && (p_dev->temp_buffer[1] == (uint8_t)0x5)
            && ((p_dev->temp_buffer[2] & (uint8_t)0x5) == (uint8_t)0x5)
            && ((p_dev->temp_buffer[3] & (uint8_t)0x10) ==(uint8_t)0x10)
            )
    {
        *p_isReady = (uint8_t)1;
         p_dev->streamcount = p_dev->temp_buffer[0];
    }
    else
    {
        if ((p_dev->temp_buffer[3] & (uint8_t)0x80) != (uint8_t)0)
        {
            status |= p_dev->temp_buffer[2];    /* Return GO2 error status */
        }

        *p_isReady = 0;
    }

    return status;
}

uint8_t AP_RangeFinder_VL53L5CX::vl53l5cx_get_ranging_data(
        VL53L5CX_Configuration      *p_dev,
        VL53L5CX_ResultsData        *p_results)
{
    uint8_t status = VL53L5CX_STATUS_OK;
    union Block_header *bh_ptr;
    uint16_t header_id, footer_id;
    uint32_t i, j, msize;

    status |= RdMulti(0x0, p_dev->temp_buffer, p_dev->data_read_size);
    p_dev->streamcount = p_dev->temp_buffer[0];
    SwapBuffer(p_dev->temp_buffer, (uint16_t)p_dev->data_read_size);

    /* Start conversion at position 16 to avoid headers */
    for (i = 16U; i < (uint32_t)p_dev->data_read_size; i+=4U)
    {
        bh_ptr = (union Block_header *)&(p_dev->temp_buffer[i]);
        if ((bh_ptr->type > 0x1U) 
                    && (bh_ptr->type < 0xdU))
        {
            msize = bh_ptr->type * bh_ptr->size;
        }
        else
        {
            msize = bh_ptr->size;
        }

        switch(bh_ptr->idx){
            case VL53L5CX_METADATA_IDX:
                p_results->silicon_temp_degc =
                        (int8_t)p_dev->temp_buffer[i + (uint32_t)12];
                break;

#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
            case VL53L5CX_AMBIENT_RATE_IDX:
                (void)memcpy(p_results->ambient_per_spad,
                &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
            case VL53L5CX_SPAD_COUNT_IDX:
                (void)memcpy(p_results->nb_spads_enabled,
                &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
            case VL53L5CX_NB_TARGET_DETECTED_IDX:
                (void)memcpy(p_results->nb_target_detected,
                &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
            case VL53L5CX_SIGNAL_RATE_IDX:
                (void)memcpy(p_results->signal_per_spad,
                &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
            case VL53L5CX_RANGE_SIGMA_MM_IDX:
                (void)memcpy(p_results->range_sigma_mm,
                &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_DISTANCE_MM
            case VL53L5CX_DISTANCE_IDX:
                (void)memcpy(p_results->distance_mm,
                &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
            case VL53L5CX_REFLECTANCE_EST_PC_IDX:
                (void)memcpy(p_results->reflectance,
                &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_TARGET_STATUS
            case VL53L5CX_TARGET_STATUS_IDX:
                (void)memcpy(p_results->target_status,
                &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
            case VL53L5CX_MOTION_DETEC_IDX:
                (void)memcpy(&p_results->motion_indicator,
                &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
                break;
#endif
            default:
                break;
        }
        i += msize;
    }

#ifndef VL53L5CX_USE_RAW_FORMAT

    /* Convert data into their real format */
#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
    for(i = 0; i < (uint32_t)VL53L5CX_RESOLUTION_8X8; i++)
    {
        p_results->ambient_per_spad[i] /= (uint32_t)2048;
    }
#endif

    for(i = 0; i < (uint32_t)(VL53L5CX_RESOLUTION_8X8
            *VL53L5CX_NB_TARGET_PER_ZONE); i++)
    {
#ifndef VL53L5CX_DISABLE_DISTANCE_MM
        p_results->distance_mm[i] /= 4;
        if(p_results->distance_mm[i] < 0)
        {
            p_results->distance_mm[i] = 0;
        }
#endif
#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
        p_results->reflectance[i] /= (uint8_t)2;
#endif
#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
        p_results->range_sigma_mm[i] /= (uint16_t)128;
#endif
#ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
        p_results->signal_per_spad[i] /= (uint32_t)2048;
#endif
    }

    /* Set target status to 255 if no target is detected for this zone */
#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
    for(i = 0; i < (uint32_t)VL53L5CX_RESOLUTION_8X8; i++)
    {
        if(p_results->nb_target_detected[i] == (uint8_t)0){
            for(j = 0; j < (uint32_t)
                VL53L5CX_NB_TARGET_PER_ZONE; j++)
            {
#ifndef VL53L5CX_DISABLE_TARGET_STATUS
                p_results->target_status
                [((uint32_t)VL53L5CX_NB_TARGET_PER_ZONE
                    *(uint32_t)i) + j]=(uint8_t)255;
#endif
            }
        }
    }
#endif

#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
    for(i = 0; i < (uint32_t)32; i++)
    {
        p_results->motion_indicator.motion[i] /= (uint32_t)65535;
    }
#endif

#endif

    /* Check if footer id and header id are matching. This allows to detect
     * corrupted frames */
    header_id = ((uint16_t)(p_dev->temp_buffer[0x8])<<8) & 0xFF00U;
    header_id |= ((uint16_t)(p_dev->temp_buffer[0x9])) & 0x00FFU;

    footer_id = ((uint16_t)(p_dev->temp_buffer[p_dev->data_read_size
        - (uint32_t)4]) << 8) & 0xFF00U;
    footer_id |= ((uint16_t)(p_dev->temp_buffer[p_dev->data_read_size
        - (uint32_t)3])) & 0xFFU;

    if(header_id != footer_id)
    {
        status |= VL53L5CX_STATUS_CORRUPTED_FRAME;
    }

    return status;
}

int32_t AP_RangeFinder_VL53L5CX::convert_data_format(VL53L5CX_Object_t *pObj,
    VL53L5CX_ResultsData *data, RANGING_SENSOR_Result_t *pResult)
{
  int32_t ret;
  uint8_t i, j;
  uint8_t resolution;
  uint8_t target_status;

  if ((pObj == NULL) || (pResult == NULL))
  {
    ret = VL53L5CX_INVALID_PARAM;
  }
  else if (vl53l5cx_get_resolution(&pObj->Dev, &resolution) != VL53L5CX_STATUS_OK)
  {
    ret = VL53L5CX_ERROR;
  }
  else
  {
    pResult->NumberOfZones = resolution;

    for (i = 0; i < resolution; i++)
    {
      pResult->ZoneResult[i].NumberOfTargets = data->nb_target_detected[i];

      for (j = 0; j < data->nb_target_detected[i]; j++)
      {
        pResult->ZoneResult[i].Distance[j] = (uint32_t)data->distance_mm[(VL53L5CX_NB_TARGET_PER_ZONE * i) + j];

        /* return Ambient value if ambient rate output is enabled */
        if (pObj->IsAmbientEnabled == 1U)
        {
          /* apply ambient value to all targets in a given zone */
          pResult->ZoneResult[i].Ambient[j] = (float_t)data->ambient_per_spad[i];
        }
        else
        {
          pResult->ZoneResult[i].Ambient[j] = 0.0f;
        }

        /* return Signal value if signal rate output is enabled */
        if (pObj->IsSignalEnabled == 1U)
        {
          pResult->ZoneResult[i].Signal[j] =
            (float_t)data->signal_per_spad[(VL53L5CX_NB_TARGET_PER_ZONE * i) + j];
        }
        else
        {
          pResult->ZoneResult[i].Signal[j] = 0.0f;
        }

        target_status = data->target_status[(VL53L5CX_NB_TARGET_PER_ZONE * i) + j];
        pResult->ZoneResult[i].Status[j] = map_target_status(target_status);
      }
    }

    ret = VL53L5CX_OK;
  }

  return ret;
}

uint8_t vl53l5cx_dci_read_data(
        VL53L5CX_Configuration      *p_dev,
        uint8_t             *data,
        uint32_t            index,
        uint16_t            data_size)
{
    int16_t i;
    uint8_t status = VL53L5CX_STATUS_OK;
        uint32_t rd_size = (uint32_t) data_size + (uint32_t)12;
    uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x0f,
            0x00, 0x02, 0x00, 0x08};

    /* Check if tmp buffer is large enough */
    if((data_size + (uint16_t)12)>(uint16_t)VL53L5CX_TEMPORARY_BUFFER_SIZE)
    {
        status |= VL53L5CX_STATUS_ERROR;
    }
    else
    {
        cmd[0] = (uint8_t)(index >> 8); 
        cmd[1] = (uint8_t)(index & (uint32_t)0xff);         
        cmd[2] = (uint8_t)((data_size & (uint16_t)0xff0) >> 4);
        cmd[3] = (uint8_t)((data_size & (uint16_t)0xf) << 4);

    /* Request data reading from FW */
        status |= WrMulti((VL53L5CX_UI_CMD_END-(uint16_t)11),cmd, sizeof(cmd));
        // status |= _vl53l5cx_poll_for_answer(p_dev, 4, 1, VL53L5CX_UI_CMD_STATUS, 0xff, 0x03);

    /* Read new data sent (4 bytes header + data_size + 8 bytes footer) */
        status |= RdMulti(VL53L5CX_UI_CMD_START, p_dev->temp_buffer, rd_size);
        SwapBuffer(p_dev->temp_buffer, data_size + (uint16_t)12);

    /* Copy data from FW into input structure (-4 bytes to remove header) */
        for(i = 0 ; i < (int16_t)data_size;i++){
            data[i] = p_dev->temp_buffer[i + 4];
        }
    }

    return status;
}

uint8_t vl53l5cx_dci_write_data(
        VL53L5CX_Configuration      *p_dev,
        uint8_t             *data,
        uint32_t            index,
        uint16_t            data_size)
{
    uint8_t status = VL53L5CX_STATUS_OK;
    int16_t i;

    uint8_t headers[] = {0x00, 0x00, 0x00, 0x00};
    uint8_t footer[] = {0x00, 0x00, 0x00, 0x0f, 0x05, 0x01,
            (uint8_t)((data_size + (uint16_t)8) >> 8), 
            (uint8_t)((data_size + (uint16_t)8) & (uint8_t)0xFF)};

    uint16_t address = (uint16_t)VL53L5CX_UI_CMD_END - 
        (data_size + (uint16_t)12) + (uint16_t)1;

    /* Check if cmd buffer is large enough */
    if((data_size + (uint16_t)12) 
           > (uint16_t)VL53L5CX_TEMPORARY_BUFFER_SIZE)
    {
        status |= VL53L5CX_STATUS_ERROR;
    }
    else
    {
        headers[0] = (uint8_t)(index >> 8);
        headers[1] = (uint8_t)(index & (uint32_t)0xff);
        headers[2] = (uint8_t)(((data_size & (uint16_t)0xff0) >> 4));
        headers[3] = (uint8_t)((data_size & (uint16_t)0xf) << 4);

    /* Copy data from structure to FW format (+4 bytes to add header) */
        SwapBuffer(data, data_size);
        for(i = (int16_t)data_size - (int16_t)1 ; i >= 0; i--)
        {
            p_dev->temp_buffer[i + 4] = data[i];
        }

    /* Add headers and footer */
        (void)memcpy(&p_dev->temp_buffer[0], headers, sizeof(headers));
        (void)memcpy(&p_dev->temp_buffer[data_size + (uint16_t)4],
            footer, sizeof(footer));

    /* Send data to FW */
        status |= WrMulti(address, p_dev->temp_buffer, (uint32_t)((uint32_t)data_size + (uint32_t)12));
        // status |= _vl53l5cx_poll_for_answer(p_dev, 4, 1, VL53L5CX_UI_CMD_STATUS, 0xff, 0x03);

        SwapBuffer(data, data_size);
    }

    return status;
}

uint8_t vl53l5cx_dci_replace_data(
        VL53L5CX_Configuration      *p_dev,
        uint8_t             *data,
        uint32_t            index,
        uint16_t            data_size,
        uint8_t             *new_data,
        uint16_t            new_data_size,
        uint16_t            new_data_pos)
{
    uint8_t status = VL53L5CX_STATUS_OK;

    status |= vl53l5cx_dci_read_data(p_dev, data, index, data_size);
    (void)memcpy(&(data[new_data_pos]), new_data, new_data_size);
    status |= vl53l5cx_dci_write_data(p_dev, data, index, data_size);

    return status;
}

void SwapBuffer(
    uint8_t     *buffer,
    uint16_t     size)
{
  uint32_t i, tmp;

  /* Example of possible implementation using <string.h> */
  for(i = 0; i < size; i = i + 4)
  {
    tmp = (
      buffer[i]<<24)
    |(buffer[i+1]<<16)
    |(buffer[i+2]<<8)
    |(buffer[i+3]);

    memcpy(&(buffer[i]), &tmp, 4);
  }
}

bool AP_RangeFinder_VL53L5CX::read_register(uint16_t reg, uint8_t &value)
{
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    return dev->transfer(b, 2, &value, 1);
}

bool AP_RangeFinder_VL53L5CX::RdMulti(uint16_t reg, uint8_t* value, uint32_t len)
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


bool AP_RangeFinder_VL53L5CX::WrByte(uint16_t reg, uint8_t value)
{
    uint8_t b[3] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), value };
    return dev->transfer(b, 3, nullptr, 0);
}

bool AP_RangeFinder_VL53L5CX::WrMulti(uint16_t reg, uint8_t* value, uint32_t len)
{
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    if (!dev->transfer(b, 2, value, len)) {
        return false;
    }
    return true;
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
