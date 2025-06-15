#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_VL53L5CX_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>


#define VL53L5CX_STATUS_OK          ((uint8_t) 0U)
#define VL53L5CX_STATUS_TIMEOUT_ERROR       ((uint8_t) 1U)
#define VL53L5CX_STATUS_CORRUPTED_FRAME     ((uint8_t) 2U)
#define VL53L5CX_STATUS_CRC_CSUM_FAILED     ((uint8_t) 3U)
#define VL53L5CX_MCU_ERROR          ((uint8_t) 66U)
#define VL53L5CX_STATUS_INVALID_PARAM       ((uint8_t) 127U)
#define VL53L5CX_STATUS_ERROR           ((uint8_t) 255U)

#define VL53L5CX_RESOLUTION_4X4     ((uint8_t) 16U)
#define VL53L5CX_RESOLUTION_8X8     ((uint8_t) 64U)

#define VL53L5CX_OK                  (0)
#define VL53L5CX_ERROR               (-1)
#define VL53L5CX_INVALID_PARAM       (-2)
#define VL53L5CX_TIMEOUT             (-3)
#define VL53L5CX_NOT_IMPLEMENTED     (-4)

#ifndef VL53L5CX_NB_TARGET_PER_ZONE
#define VL53L5CX_NB_TARGET_PER_ZONE     (1U)
#endif

#define VL53L5CX_RANGING_MODE_CONTINUOUS    ((uint8_t) 1U)
#define VL53L5CX_RANGING_MODE_AUTONOMOUS    ((uint8_t) 3U)
#define TIMING_BUDGET (30U) /* 5 ms < TimingBudget < 100 ms */
#define RANGING_FREQUENCY (5U) /* Ranging frequency Hz (shall be consistent with TimingBudget value) */


// #define VL53L5CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L5CX_DISABLE_NB_SPADS_ENABLED
#define VL53L5CX_DISABLE_AMBIENT_DMAX
// #define VL53L5CX_DISABLE_NB_TARGET_DETECTED
// #define VL53L5CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L5CX_DISABLE_RANGE_SIGMA_MM
// #define VL53L5CX_DISABLE_DISTANCE_MM
// #define VL53L5CX_DISABLE_TARGET_STATUS

#define VL53L5CX_START_BH               ((uint32_t)0x0000000DU)
#define VL53L5CX_METADATA_BH            ((uint32_t)0x54B400C0U)
#define VL53L5CX_COMMONDATA_BH          ((uint32_t)0x54C00040U)
#define VL53L5CX_AMBIENT_RATE_BH        ((uint32_t)0x54D00104U)
#define VL53L5CX_SPAD_COUNT_BH          ((uint32_t)0x55D00404U)
#define VL53L5CX_NB_TARGET_DETECTED_BH  ((uint32_t)0xDB840401U)
#define VL53L5CX_SIGNAL_RATE_BH         ((uint32_t)0xDBC40404U)
#define VL53L5CX_RANGE_SIGMA_MM_BH      ((uint32_t)0xDEC40402U)
#define VL53L5CX_DISTANCE_BH            ((uint32_t)0xDF440402U)
#define VL53L5CX_REFLECTANCE_BH         ((uint32_t)0xE0440401U)
#define VL53L5CX_TARGET_STATUS_BH       ((uint32_t)0xE0840401U)
#define VL53L5CX_MOTION_DETECT_BH       ((uint32_t)0xD85808C0U)

#define VL53L5CX_METADATA_IDX           ((uint16_t)0x54B4U)
#define VL53L5CX_SPAD_COUNT_IDX         ((uint16_t)0x55D0U)
#define VL53L5CX_AMBIENT_RATE_IDX       ((uint16_t)0x54D0U)
#define VL53L5CX_NB_TARGET_DETECTED_IDX ((uint16_t)0xDB84U)
#define VL53L5CX_SIGNAL_RATE_IDX        ((uint16_t)0xDBC4U)
#define VL53L5CX_RANGE_SIGMA_MM_IDX     ((uint16_t)0xDEC4U)
#define VL53L5CX_DISTANCE_IDX           ((uint16_t)0xDF44U)
#define VL53L5CX_REFLECTANCE_EST_PC_IDX ((uint16_t)0xE044U)
#define VL53L5CX_TARGET_STATUS_IDX      ((uint16_t)0xE084U)
#define VL53L5CX_MOTION_DETEC_IDX       ((uint16_t)0xD858U)

#define VL53L5CX_NVM_DATA_SIZE          ((uint16_t)492U)
#define VL53L5CX_CONFIGURATION_SIZE     ((uint16_t)972U)
#define VL53L5CX_OFFSET_BUFFER_SIZE     ((uint16_t)488U)
#define VL53L5CX_XTALK_BUFFER_SIZE      ((uint16_t)776U)

#define VL53L5CX_DCI_ZONE_CONFIG        ((uint16_t)0x5450U)
#define VL53L5CX_DCI_FREQ_HZ            ((uint16_t)0x5458U)
#define VL53L5CX_DCI_INT_TIME           ((uint16_t)0x545CU)
#define VL53L5CX_DCI_FW_NB_TARGET       ((uint16_t)0x5478)
#define VL53L5CX_DCI_RANGING_MODE       ((uint16_t)0xAD30U)
#define VL53L5CX_DCI_DSS_CONFIG         ((uint16_t)0xAD38U)
#define VL53L5CX_DCI_TARGET_ORDER       ((uint16_t)0xAE64U)
#define VL53L5CX_DCI_SHARPENER          ((uint16_t)0xAED8U)
#define VL53L5CX_DCI_INTERNAL_CP        ((uint16_t)0xB39CU)
#define VL53L5CX_DCI_SYNC_PIN           ((uint16_t)0xB5F0U)
#define VL53L5CX_DCI_MOTION_DETECTOR_CFG ((uint16_t)0xBFACU)
#define VL53L5CX_DCI_SINGLE_RANGE       ((uint16_t)0xD964U)
#define VL53L5CX_DCI_OUTPUT_CONFIG      ((uint16_t)0xD968U)
#define VL53L5CX_DCI_OUTPUT_ENABLES     ((uint16_t)0xD970U)
#define VL53L5CX_DCI_OUTPUT_LIST        ((uint16_t)0xD980U)
#define VL53L5CX_DCI_PIPE_CONTROL       ((uint16_t)0xDB80U)

#define VL53L5CX_UI_CMD_STATUS          ((uint16_t)0x2C00U)
#define VL53L5CX_UI_CMD_START           ((uint16_t)0x2C04U)
#define VL53L5CX_UI_CMD_END             ((uint16_t)0x2FFFU)

#define VL53L5CX_MAX_NB_ZONES        (VL53L5CX_RESOLUTION_8X8)
/**
 * @brief Inner values for API. Max buffer size depends of the selected output.
 */

#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
#define L5CX_AMB_SIZE   260U
#else
#define L5CX_AMB_SIZE   0U
#endif

#ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
#define L5CX_SPAD_SIZE  260U
#else
#define L5CX_SPAD_SIZE  0U
#endif

#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
#define L5CX_NTAR_SIZE  68U
#else
#define L5CX_NTAR_SIZE  0U
#endif

#ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
#define L5CX_SPS_SIZE ((256U * VL53L5CX_NB_TARGET_PER_ZONE) + 4U)
#else
#define L5CX_SPS_SIZE   0U
#endif

#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
#define L5CX_SIGR_SIZE ((128U * VL53L5CX_NB_TARGET_PER_ZONE) + 4U)
#else
#define L5CX_SIGR_SIZE  0U
#endif

#ifndef VL53L5CX_DISABLE_DISTANCE_MM
#define L5CX_DIST_SIZE ((128U * VL53L5CX_NB_TARGET_PER_ZONE) + 4U)
#else
#define L5CX_DIST_SIZE  0U
#endif

#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
#define L5CX_RFLEST_SIZE ((64U *VL53L5CX_NB_TARGET_PER_ZONE) + 4U)
#else
#define L5CX_RFLEST_SIZE    0U
#endif

#ifndef VL53L5CX_DISABLE_TARGET_STATUS
#define L5CX_STA_SIZE ((64U  *VL53L5CX_NB_TARGET_PER_ZONE) + 4U)
#else
#define L5CX_STA_SIZE   0U
#endif

#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
#define L5CX_MOT_SIZE   144U
#else
#define L5CX_MOT_SIZE   0U
#endif

#define VL53L5CX_MAX_RESULTS_SIZE ( 40U \
    + L5CX_AMB_SIZE + L5CX_SPAD_SIZE + L5CX_NTAR_SIZE + L5CX_SPS_SIZE \
    + L5CX_SIGR_SIZE + L5CX_DIST_SIZE + L5CX_RFLEST_SIZE + L5CX_STA_SIZE \
    + L5CX_MOT_SIZE + 20U)

/**
 * @brief Macro VL53L5CX_TEMPORARY_BUFFER_SIZE can be used to know the size of
 * the temporary buffer. The minimum size is 1024, and the maximum depends of
 * the output configuration.
 */

#if VL53L5CX_MAX_RESULTS_SIZE < 1024U
#define VL53L5CX_TEMPORARY_BUFFER_SIZE ((uint32_t) 1024U)
#else
#define VL53L5CX_TEMPORARY_BUFFER_SIZE ((uint32_t) VL53L5CX_MAX_RESULTS_SIZE)
#endif

#define RANGING_SENSOR_NB_TARGET_PER_ZONE   (VL53L5CX_NB_TARGET_PER_ZONE)
#define RANGING_SENSOR_MAX_NB_ZONES         (VL53L5CX_MAX_NB_ZONES)

class AP_RangeFinder_VL53L5CX : public AP_RangeFinder_Backend
{

public:

    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev);

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    // constructor
    AP_RangeFinder_VL53L5CX(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    union PACKED Block_header {
        uint32_t bytes;
        struct {
            uint32_t type : 4;
            uint32_t size : 12;
            uint32_t idx : 16;
        };
    };

    typedef struct
    {
        /* Platform, filled by customer into the 'platform.h' file */
        // VL53L5CX_Platform   platform;
        /* Results streamcount, value auto-incremented at each range */
        uint8_t             streamcount;
        /* Size of data read though I2C */
        uint32_t            data_read_size;
        /* Address of default configuration buffer */
        // uint8_t             *default_configuration;
        /* Address of default Xtalk buffer */
        // uint8_t             *default_xtalk;
        /* Offset buffer */
        uint8_t             offset_data[VL53L5CX_OFFSET_BUFFER_SIZE];
        /* Xtalk buffer */
        uint8_t             xtalk_data[VL53L5CX_XTALK_BUFFER_SIZE];
        /* Temporary buffer used for internal driver processing */
         uint8_t            temp_buffer[VL53L5CX_TEMPORARY_BUFFER_SIZE];
    } VL53L5CX_Configuration;

    typedef struct
    {
        /* Internal sensor silicon temperature */
        int8_t silicon_temp_degc;

        /* Ambient noise in kcps/spads */
    #ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
        uint32_t ambient_per_spad[VL53L5CX_RESOLUTION_8X8];
    #endif

        /* Number of valid target detected for 1 zone */
    #ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
        uint8_t nb_target_detected[VL53L5CX_RESOLUTION_8X8];
    #endif

        /* Number of spads enabled for this ranging */
    #ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
        uint32_t nb_spads_enabled[VL53L5CX_RESOLUTION_8X8];
    #endif

        /* Signal returned to the sensor in kcps/spads */
    #ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
        uint32_t signal_per_spad[(VL53L5CX_RESOLUTION_8X8
                        *VL53L5CX_NB_TARGET_PER_ZONE)];
    #endif

        /* Sigma of the current distance in mm */
    #ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
        uint16_t range_sigma_mm[(VL53L5CX_RESOLUTION_8X8
                        *VL53L5CX_NB_TARGET_PER_ZONE)];
    #endif

        /* Measured distance in mm */
    #ifndef VL53L5CX_DISABLE_DISTANCE_MM
        int16_t distance_mm[(VL53L5CX_RESOLUTION_8X8
                        *VL53L5CX_NB_TARGET_PER_ZONE)];
    #endif

        /* Estimated reflectance in percent */
    #ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
        uint8_t reflectance[(VL53L5CX_RESOLUTION_8X8
                        *VL53L5CX_NB_TARGET_PER_ZONE)];
    #endif

        /* Status indicating the measurement validity (5 & 9 means ranging OK)*/
    #ifndef VL53L5CX_DISABLE_TARGET_STATUS
        uint8_t target_status[(VL53L5CX_RESOLUTION_8X8
                        *VL53L5CX_NB_TARGET_PER_ZONE)];
    #endif

        /* Motion detector results */
    #ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
        struct
        {
            uint32_t global_indicator_1;
            uint32_t global_indicator_2;
            uint8_t  status;
            uint8_t  nb_of_detected_aggregates;
            uint8_t  nb_of_aggregates;
            uint8_t  spare;
            uint32_t motion[32];
        } motion_indicator;
    #endif

    } VL53L5CX_ResultsData;

    typedef struct
    {
      uint32_t NumberOfTargets;
      uint32_t Distance[RANGING_SENSOR_NB_TARGET_PER_ZONE];  /*!< millimeters */
      uint32_t Status[RANGING_SENSOR_NB_TARGET_PER_ZONE];    /*!< OK: 0, NOK: !0 */
      float Ambient[RANGING_SENSOR_NB_TARGET_PER_ZONE];    /*!< kcps / spad */
      float Signal[RANGING_SENSOR_NB_TARGET_PER_ZONE];     /*!< kcps / spad */
    } RANGING_SENSOR_ZoneResult_t;

    typedef struct
    {
      uint32_t NumberOfZones;
      RANGING_SENSOR_ZoneResult_t ZoneResult[RANGING_SENSOR_MAX_NB_ZONES];
    } RANGING_SENSOR_Result_t;

    bool init();
    void timer();

    // check sensor ID
    bool check_id(void);

    // get a reading
    bool get_reading(uint16_t &reading_cm);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    VL53L5CX_Configuration my_dev;

    uint32_t sum_mm;
    uint32_t counter;
    bool calibrated;

    uint8_t distance_mm[64];

    VL53L5CX_ResultsData Data;
    RANGING_SENSOR_Result_t Result;
    Block_header Bh;
    
    bool reset(void);

    uint8_t vl53l5cx_set_resolution(VL53L5CX_Configuration *p_dev, uint8_t resolution);
    uint8_t vl53l5cx_set_ranging_mode(VL53L5CX_Configuration *p_dev, uint8_t ranging_mode);
    uint8_t vl53l5cx_set_integration_time_ms(VL53L5CX_Configuration *p_dev, uint32_t integration_time_ms);
    uint8_t vl53l5cx_set_ranging_frequency_hz(VL53L5CX_Configuration *p_dev, uint8_t frequency_hz);

    uint8_t vl53l5cx_start_ranging(VL53L5CX_Configuration *p_dev);
    uint8_t vl53l5cx_check_data_ready(VL53L5CX_Configuration *p_dev, uint8_t *p_isReady);
    uint8_t vl53l5cx_get_resolution(VL53L5CX_Configuration *p_dev, uint8_t *p_resolution);
    uint8_t vl53l5cx_get_ranging_data(VL53L5CX_Configuration *p_dev, VL53L5CX_ResultsData *p_results);
    int32_t convert_data_format(VL53L5CX_Configuration *p_dev, VL53L5CX_ResultsData *data, RANGING_SENSOR_Result_t *pResult);
    uint8_t vl53l5cx_dci_read_data(VL53L5CX_Configuration *p_dev, uint8_t *data, uint32_t index, uint16_t data_size);
    uint8_t vl53l5cx_dci_write_data(VL53L5CX_Configuration *p_dev, uint8_t *data, uint32_t index, uint16_t data_size);
    uint8_t vl53l5cx_dci_replace_data(VL53L5CX_Configuration *p_dev, uint8_t *data, uint32_t index, uint16_t data_size, uint8_t *new_data, uint16_t new_data_size, uint16_t new_data_pos);
    uint8_t _vl53l5cx_send_offset_data(VL53L5CX_Configuration *p_dev, uint8_t resolution);
    uint8_t _vl53l5cx_send_xtalk_data(VL53L5CX_Configuration *p_dev, uint8_t resolution);\

    void SwapBuffer(uint8_t *buffer, uint16_t size);
    uint8_t map_target_status(uint8_t status);
    bool read_register(uint16_t reg, uint8_t &value);
    bool RdMulti(uint16_t reg, uint8_t* value, uint32_t len);
    bool write_register(uint16_t reg, uint8_t value);
    bool WrByte(uint16_t reg, uint8_t value);
    bool WrMulti(uint16_t reg, uint8_t* value, uint32_t len);
    uint8_t _vl53l5cx_poll_for_answer(VL53L5CX_Configuration  *p_dev, uint8_t size, uint8_t pos, uint16_t address, uint8_t mask, uint8_t expected_value);

};

#endif  // AP_RANGEFINDER_VL53L5CX_ENABLED
