#pragma once

#include "AP_Depth_Backend.h"

#ifndef AP_DEPTH_MS56XX_ENABLED
#define AP_DEPTH_MS56XX_ENABLED 1
#endif

#if AP_DEPTH_MS56XX_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

#ifndef HAL_DEPTH_MS5837_I2C_ADDR
#define HAL_DEPTH_MS5837_I2C_ADDR 0x76
#endif

class AP_Depth_MS56XX : public AP_Depth_Backend
{
public:
    void update() override;

    enum MS56XX_TYPE {
        BARO_MS5837 = 3
    };

    static AP_Depth_Backend *probe_5837(AP_Depth &depth, AP_HAL::OwnPtr<AP_HAL::Device> dev) {
        return probe(depth, std::move(dev), BARO_MS5837);
    }

    static AP_Depth_Backend *probe(AP_Depth &depth, AP_HAL::OwnPtr<AP_HAL::Device> dev, enum MS56XX_TYPE ms56xx_type=BARO_MS5837);

private:

    /*
     * Update @accum and @count with the new sample in @val, taking into
     * account a maximum number of samples given by @max_count; in case
     * maximum number is reached, @accum and @count are updated appropriately
     */
    static void _update_and_wrap_accumulator(uint32_t *accum, uint32_t val,
                                             uint8_t *count, uint8_t max_count);

    AP_Depth_MS56XX(AP_Depth &depth, AP_HAL::OwnPtr<AP_HAL::Device> dev, enum MS56XX_TYPE ms56xx_type);

    bool _init();

    void _calculate_5837();
    bool _read_prom_5637(uint16_t prom[8]);

    uint16_t _read_prom_word(uint8_t word);
    uint32_t _read_adc();

    void _timer();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /* Shared values between thread sampling the HW and main thread */
    struct {
        uint32_t s_D1;
        uint32_t s_D2;
        uint8_t d1_count;
        uint8_t d2_count;
    } _accum;

    uint8_t _state;
    uint8_t _instance;

    /* Last compensated values from accumulated sample */
    float _D1, _D2;

    // Internal calibration registers
    struct {
        uint16_t c1, c2, c3, c4, c5, c6;
    } _cal_reg;

    bool _discard_next;

    enum MS56XX_TYPE _ms56xx_type;
};

#endif  // AP_DEPTH_MS56XX_ENABLED
