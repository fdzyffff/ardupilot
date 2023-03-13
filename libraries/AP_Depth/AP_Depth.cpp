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
 *       APM_Baro.cpp - barometer driver
 *
 */
#include "AP_Depth.h"

#include <utility>
#include <stdio.h>

#include <GCS_MAVLink/GCS.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/I2CDevice.h>

#include "AP_Depth_MS5611.h"


#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#define INTERNAL_TEMPERATURE_CLAMP 35.0f

#ifndef HAL_BARO_FILTER_DEFAULT
 #define HAL_BARO_FILTER_DEFAULT 0 // turned off by default
#endif

#if !defined(HAL_PROBE_EXTERNAL_I2C_BAROS) && !HAL_MINIMIZE_FEATURES
#define HAL_PROBE_EXTERNAL_I2C_BAROS
#endif

#ifndef HAL_BARO_PROBE_EXT_DEFAULT
 #define HAL_BARO_PROBE_EXT_DEFAULT 0
#endif

#ifndef HAL_BARO_EXTERNAL_BUS_DEFAULT
 #define HAL_BARO_EXTERNAL_BUS_DEFAULT -1
#endif

#ifdef HAL_BUILD_AP_PERIPH
#define HAL_BARO_ALLOW_INIT_NO_BARO
#endif

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Depth::var_info[] = {
    // NOTE: Index numbers 0 and 1 were for the old integer
    // ground temperature and pressure

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: 1_GND_PRESS
    // @DisplayName: Ground Pressure
    // @Description: calibrated ground pressure in Pascals
    // @Units: Pa
    // @Increment: 1
    // @ReadOnly: True
    // @Volatile: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("1_GND_PRESS", 2, AP_Depth, sensors[0].ground_pressure, 0, AP_PARAM_FLAG_INTERNAL_USE_ONLY),

    // @Param: _GND_TEMP
    // @DisplayName: ground temperature
    // @Description: User provided ambient ground temperature in degrees Celsius. This is used to improve the calculation of the altitude the vehicle is at. This parameter is not persistent and will be reset to 0 every time the vehicle is rebooted. A value of 0 means use the internal measurement ambient temperature.
    // @Units: degC
    // @Increment: 1
    // @Volatile: True
    // @User: Advanced
    AP_GROUPINFO("_GND_TEMP", 3, AP_Depth, _user_ground_temperature, 0),

    // index 4 reserved for old AP_Int8 version in legacy FRAM
    //AP_GROUPINFO("ALT_OFFSET", 4, AP_Depth, _alt_offset, 0),

    // @Param: _ALT_OFFSET
    // @DisplayName: altitude offset
    // @Description: altitude offset in meters added to barometric altitude. This is used to allow for automatic adjustment of the base barometric altitude by a ground station equipped with a barometer. The value is added to the barometric altitude read by the aircraft. It is automatically reset to 0 when the barometer is calibrated on each reboot or when a preflight calibration is performed.
    // @Units: m
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_ALT_OFFSET", 5, AP_Depth, _alt_offset, 0),

    // @Param: _PRIMARY
    // @DisplayName: Primary barometer
    // @Description: This selects which barometer will be the primary if multiple barometers are found
    // @Values: 0:FirstBaro,1:2ndBaro,2:3rdBaro
    // @User: Advanced
    AP_GROUPINFO("_PRIMARY", 6, AP_Depth, _primary_baro, 0),
#endif // HAL_BUILD_AP_PERIPH

    // @Param: _EXT_BUS
    // @DisplayName: External baro bus
    // @Description: This selects the bus number for looking for an I2C barometer. When set to -1 it will probe all external i2c buses based on the GND_PROBE_EXT parameter.
    // @Values: -1:Disabled,0:Bus0,1:Bus1
    // @User: Advanced
    AP_GROUPINFO("_EXT_BUS", 7, AP_Depth, _ext_bus, HAL_BARO_EXTERNAL_BUS_DEFAULT),

    // @Param{Sub}: _SPEC_GRAV
    // @DisplayName: Specific Gravity (For water depth measurement)
    // @Description: This sets the specific gravity of the fluid when flying an underwater ROV.
    // @Values: 1.0:Freshwater,1.024:Saltwater
    AP_GROUPINFO_FRAME("_SPEC_GRAV", 8, AP_Depth, _specific_gravity, 1.0, AP_PARAM_FRAME_SUB),

#if DEPTH_MAX_INSTANCES > 1
    // @Param: 2_GND_PRESS
    // @DisplayName: Ground Pressure
    // @Description: calibrated ground pressure in Pascals
    // @Units: Pa
    // @Increment: 1
    // @ReadOnly: True
    // @Volatile: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("2_GND_PRESS", 9, AP_Depth, sensors[1].ground_pressure, 0, AP_PARAM_FLAG_INTERNAL_USE_ONLY),

    // Slot 10 used to be TEMP2
#endif

#if DEPTH_MAX_INSTANCES > 2
    // @Param: 3_GND_PRESS
    // @DisplayName: Absolute Pressure
    // @Description: calibrated ground pressure in Pascals
    // @Units: Pa
    // @Increment: 1
    // @ReadOnly: True
    // @Volatile: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("3_GND_PRESS", 11, AP_Depth, sensors[2].ground_pressure, 0, AP_PARAM_FLAG_INTERNAL_USE_ONLY),

    // Slot 12 used to be TEMP3
#endif

    // @Param: _FLTR_RNG
    // @DisplayName: Range in which sample is accepted
    // @Description: This sets the range around the average value that new samples must be within to be accepted. This can help reduce the impact of noise on sensors that are on long I2C cables. The value is a percentage from the average value. A value of zero disables this filter.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    AP_GROUPINFO("_FLTR_RNG", 13, AP_Depth, _filter_range, HAL_BARO_FILTER_DEFAULT),

#if defined(HAL_PROBE_EXTERNAL_I2C_BAROS) || defined(AP_BARO_MSP_ENABLED)
    // @Param: _PROBE_EXT
    // @DisplayName: External barometers to probe
    // @Description: This sets which types of external i2c barometer to look for. It is a bitmask of barometer types. The I2C buses to probe is based on GND_EXT_BUS. If BARO_EXT_BUS is -1 then it will probe all external buses, otherwise it will probe just the bus number given in GND_EXT_BUS.
    // @Bitmask: 0:BMP085,1:BMP280,2:MS5611,3:MS5607,4:MS5637,5:FBM320,6:DPS280,7:LPS25H,8:Keller,9:MS5837,10:BMP388,11:SPL06,12:MSP
    // @User: Advanced
    AP_GROUPINFO("_PROBE_EXT", 14, AP_Depth, _baro_probe_ext, HAL_BARO_PROBE_EXT_DEFAULT),
#endif

    // @Param: 1_DEVID
    // @DisplayName: Baro ID
    // @Description: Barometer sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("1_DEVID", 15, AP_Depth, sensors[0].bus_id, 0, AP_PARAM_FLAG_INTERNAL_USE_ONLY),

#if DEPTH_MAX_INSTANCES > 1
    // @Param: 2_DEVID
    // @DisplayName: Baro ID2
    // @Description: Barometer2 sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("2_DEVID", 16, AP_Depth, sensors[1].bus_id, 0, AP_PARAM_FLAG_INTERNAL_USE_ONLY),
#endif

#if DEPTH_MAX_INSTANCES > 2
    // @Param: 3_DEVID
    // @DisplayName: Baro ID3
    // @Description: Barometer3 sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("3_DEVID", 17, AP_Baro, sensors[2].bus_id, 0, AP_PARAM_FLAG_INTERNAL_USE_ONLY),
#endif

    AP_GROUPEND
};

// singleton instance
AP_Depth *AP_Depth::_singleton;

#if HAL_GCS_ENABLED
#define BARO_SEND_TEXT(severity, format, args...) gcs().send_text(severity, format, ##args)
#else
#define BARO_SEND_TEXT(severity, format, args...)
#endif

/*
  AP_Depth constructor
 */
AP_Depth::AP_Depth()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
    _field_elevation_active = _field_elevation;
}

// calibrate the barometer. This must be called at least once before
// the altitude() or climb_rate() interfaces can be used
void AP_Depth::calibrate(bool save)
{
    // start by assuming all sensors are calibrated (for healthy() test)
    for (uint8_t i=0; i<_num_sensors; i++) {
        sensors[i].calibrated = true;
        sensors[i].alt_ok = true;
    }

    if (hal.util->was_watchdog_reset()) {
        BARO_SEND_TEXT(MAV_SEVERITY_INFO, "Baro: skipping calibration after WDG reset");
        return;
    }

#if AP_SIM_BARO_ENABLED
    if (AP::sitl()->baro_count == 0) {
        return;
    }
#endif

    #ifdef HAL_BARO_ALLOW_INIT_NO_BARO
    if (_num_drivers == 0 || _num_sensors == 0 || drivers[0] == nullptr) {
            BARO_SEND_TEXT(MAV_SEVERITY_INFO, "Baro: no sensors found, skipping calibration");
            return;
    }
    #endif
    
    BARO_SEND_TEXT(MAV_SEVERITY_INFO, "Calibrating barometer");

    // reset the altitude offset when we calibrate. The altitude
    // offset is supposed to be for within a flight
    _alt_offset.set_and_save(0);

    // let the barometer settle for a full second after startup
    // the MS5611 reads quite a long way off for the first second,
    // leading to about 1m of error if we don't wait
    for (uint8_t i = 0; i < 10; i++) {
        uint32_t tstart = AP_HAL::millis();
        do {
            update();
            if (AP_HAL::millis() - tstart > 500) {
                AP_BoardConfig::config_error("Baro: unable to calibrate");
            }
            hal.scheduler->delay(10);
        } while (!healthy());
        hal.scheduler->delay(100);
    }

    // now average over 5 values for the ground pressure settings
    float sum_pressure[DEPTH_MAX_INSTANCES] = {0};
    uint8_t count[DEPTH_MAX_INSTANCES] = {0};
    const uint8_t num_samples = 5;

    for (uint8_t c = 0; c < num_samples; c++) {
        uint32_t tstart = AP_HAL::millis();
        do {
            update();
            if (AP_HAL::millis() - tstart > 500) {
                AP_BoardConfig::config_error("Baro: unable to calibrate");
            }
        } while (!healthy());
        for (uint8_t i=0; i<_num_sensors; i++) {
            if (healthy(i)) {
                sum_pressure[i] += sensors[i].pressure;
                count[i] += 1;
            }
        }
        hal.scheduler->delay(100);
    }
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (count[i] == 0) {
            sensors[i].calibrated = false;
        } else {
            if (save) {
                float p0_sealevel = get_sealevel_pressure(sum_pressure[i] / count[i]);
                sensors[i].ground_pressure.set_and_save(p0_sealevel);
            }
        }
    }

    _guessed_ground_temperature = get_external_temperature();

    // panic if all sensors are not calibrated
    uint8_t num_calibrated = 0;
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (sensors[i].calibrated) {
            BARO_SEND_TEXT(MAV_SEVERITY_INFO, "Barometer %u calibration complete", i+1);
            num_calibrated++;
        }
    }
    if (num_calibrated) {
        return;
    }
    AP_BoardConfig::config_error("Baro: all sensors uncalibrated");
}

/*
   update the barometer calibration
   this updates the baro ground calibration to the current values. It
   can be used before arming to keep the baro well calibrated
*/
void AP_Depth::update_calibration()
{
    const uint32_t now = AP_HAL::millis();
    const bool do_notify = now - _last_notify_ms > 10000;
    if (do_notify) {
        _last_notify_ms = now;
    }
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (healthy(i)) {
            float corrected_pressure = get_sealevel_pressure(get_pressure(i) + sensors[i].p_correction);
            sensors[i].ground_pressure.set(corrected_pressure);
        }

        // don't notify the GCS too rapidly or we flood the link
        if (do_notify) {
            sensors[i].ground_pressure.notify();
        }
    }

    // always update the guessed ground temp
    _guessed_ground_temperature = get_external_temperature();

    // force EAS2TAS to recalculate
    _EAS2TAS = 0;
}

// return altitude difference in meters between current pressure and a
// given base_pressure in Pascal
float AP_Depth::get_altitude_difference(float base_pressure, float pressure) const
{
    float ret;
    float temp    = C_TO_KELVIN(get_ground_temperature());
    float scaling = pressure / base_pressure;

    // This is an exact calculation that is within +-2.5m of the standard
    // atmosphere tables in the troposphere (up to 11,000 m amsl).
    ret = 153.8462f * temp * (1.0f - expf(0.190259f * logf(scaling)))-_field_elevation_active;

    return ret;
}

// return sea level pressure where in which the current measured pressure
// at field elevation returns the same altitude under the
// 1976 standard atmospheric model
float AP_Depth::get_sealevel_pressure(float pressure) const
{
    float temp    = C_TO_KELVIN(get_ground_temperature());
    float p0_sealevel;
    // This is an exact calculation that is within +-2.5m of the standard
    // atmosphere tables in the troposphere (up to 11,000 m amsl).
    p0_sealevel = 8.651154799255761e30f*pressure*powF((769231.0f-(5000.0f*_field_elevation_active)/temp),-5.255993146184937f);

    return p0_sealevel;
}


// return current scale factor that converts from equivalent to true airspeed
// valid for altitudes up to 10km AMSL
// assumes standard atmosphere lapse rate
float AP_Depth::get_EAS2TAS(void)
{
    float altitude = get_altitude();
    if ((fabsf(altitude - _last_altitude_EAS2TAS) < 25.0f) && !is_zero(_EAS2TAS)) {
        // not enough change to require re-calculating
        return _EAS2TAS;
    }

    float pressure = get_pressure();
    if (is_zero(pressure)) {
        return 1.0f;
    }

    // only estimate lapse rate for the difference from the ground location
    // provides a more consistent reading then trying to estimate a complete
    // ISA model atmosphere
    float tempK = C_TO_KELVIN(get_ground_temperature()) - ISA_LAPSE_RATE * altitude;
    const float eas2tas_squared = SSL_AIR_DENSITY / (pressure / (ISA_GAS_CONSTANT * tempK));
    if (!is_positive(eas2tas_squared)) {
        return 1.0f;
    }
    _EAS2TAS = sqrtf(eas2tas_squared);
    _last_altitude_EAS2TAS = altitude;
    return _EAS2TAS;
}

// return air density / sea level density - decreases as altitude climbs
float AP_Depth::get_air_density_ratio(void)
{
    const float eas2tas = get_EAS2TAS();
    if (eas2tas > 0.0f) {
        return 1.0f/(sq(eas2tas));
    } else {
        return 1.0f;
    }
}

// return current climb_rate estimate relative to time that calibrate()
// was called. Returns climb rate in meters/s, positive means up
// note that this relies on read() being called regularly to get new data
float AP_Depth::get_climb_rate(void)
{
    // we use a 7 point derivative filter on the climb rate. This seems
    // to produce somewhat reasonable results on real hardware
    return _climb_rate_filter.slope() * 1.0e3f;
}

// returns the ground temperature in degrees C, selecting either a user
// provided one, or the internal estimate
float AP_Depth::get_ground_temperature(void) const
{
    if (is_zero(_user_ground_temperature)) {
        return _guessed_ground_temperature;
    } else {
        return _user_ground_temperature;
    }
}


/*
  set external temperature to be used for calibration (degrees C)
 */
void AP_Depth::set_external_temperature(float temperature)
{
    _external_temperature = temperature;
    _last_external_temperature_ms = AP_HAL::millis();
}

/*
  get the temperature in degrees C to be used for calibration purposes
 */
float AP_Depth::get_external_temperature(const uint8_t instance) const
{
    // if we have a recent external temperature then use it
    if (_last_external_temperature_ms != 0 && AP_HAL::millis() - _last_external_temperature_ms < 10000) {
        return _external_temperature;
    }
    
#ifndef HAL_BUILD_AP_PERIPH
#if AP_AIRSPEED_ENABLED
    // if we don't have an external temperature then try to use temperature
    // from the airspeed sensor
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed != nullptr) {
        float temperature;
        if (airspeed->healthy() && airspeed->get_temperature(temperature)) {
            return temperature;
        }
    }
#endif
#endif
    
    // if we don't have an external temperature and airspeed temperature
    // then use the minimum of the barometer temperature and 35 degrees C.
    // The reason for not just using the baro temperature is it tends to read high,
    // often 30 degrees above the actual temperature. That means the
    // EAS2TAS tends to be off by quite a large margin, as well as
    // the calculation of altitude difference betweeen two pressures
    // reporting a high temperature will cause the aircraft to
    // estimate itself as flying higher then it actually is.
    return MIN(get_temperature(instance), INTERNAL_TEMPERATURE_CLAMP);
}


bool AP_Depth::_add_backend(AP_Depth_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (_num_drivers >= DEPTH_MAX_DRIVERS) {
        AP_HAL::panic("Too many barometer drivers");
    }
    drivers[_num_drivers++] = backend;
    return true;
}

/*
  wrapper around hal.i2c_mgr->get_device() that prevents duplicate devices being opened
 */
bool AP_Depth::_have_i2c_driver(uint8_t bus, uint8_t address) const
{
    for (int i=0; i<_num_drivers; ++i) {
        if (AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_I2C, bus, address, 0) ==
            AP_HAL::Device::change_bus_id(uint32_t(sensors[i].bus_id.get()), 0)) {
            // device already has been defined.
            return true;
        }
    }
    return false;
}

/*
  macro to add a backend with check for too many sensors
 We don't try to start more than the maximum allowed
 */
#define ADD_BACKEND(backend) \
    do { _add_backend(backend);     \
       if (_num_drivers == DEPTH_MAX_DRIVERS || \
          _num_sensors == DEPTH_MAX_INSTANCES) { \
          return; \
       } \
    } while (0)

// macro for use by HAL_INS_PROBE_LIST
#define GET_I2C_DEVICE(bus, address) _have_i2c_driver(bus, address)?nullptr:hal.i2c_mgr->get_device(bus, address)
/*
  initialise the barometer object, loading backend drivers
 */
void AP_Depth::init(void)
{
    init_done = true;

    // always set field elvation to zero on reboot in the case user
    // fails to update.  TBD automate sanity checking error bounds on
    // on previously saved value at new location etc.
    if (!is_zero(_field_elevation)) {
        _field_elevation.set_and_save(0.0f);
        _field_elevation.notify();
    }

    // zero bus IDs before probing
    for (uint8_t i = 0; i < DEPTH_MAX_INSTANCES; i++) {
        sensors[i].bus_id.set(0);
    }

    ADD_BACKEND(AP_Depth_MS56XX::probe(*this,
                                      std::move(GET_I2C_DEVICE(_ext_bus, HAL_DEPTH_MS5837_I2C_ADDR)), AP_Depth_MS56XX::BARO_MS5837));
}

/*
  call update on all drivers
 */
void AP_Depth::update(void)
{
    WITH_SEMAPHORE(_rsem);

    if (fabsf(_alt_offset - _alt_offset_active) > 0.01f) {
        // If there's more than 1cm difference then slowly slew to it via LPF.
        // The EKF does not like step inputs so this keeps it happy.
        _alt_offset_active = (0.95f*_alt_offset_active) + (0.05f*_alt_offset);
    } else {
        _alt_offset_active = _alt_offset;
    }

    for (uint8_t i=0; i<_num_drivers; i++) {
        drivers[i]->backend_update(i);
    }

    for (uint8_t i=0; i<_num_sensors; i++) {
        if (sensors[i].healthy) {
            // update altitude calculation
            float ground_pressure = sensors[i].ground_pressure;
            if (!is_positive(ground_pressure) || isnan(ground_pressure) || isinf(ground_pressure)) {
                sensors[i].ground_pressure.set(sensors[i].pressure);
            }
            float altitude = sensors[i].altitude;
            float corrected_pressure = sensors[i].pressure + sensors[i].p_correction;
            if (sensors[i].type == BARO_TYPE_WATER) {
                //101325Pa is sea level air pressure, 9800 Pascal/ m depth in water.
                //No temperature or depth compensation for density of water.
                altitude = (sensors[i].ground_pressure - corrected_pressure) / 9800.0f / _specific_gravity;
            }
            // sanity check altitude
            sensors[i].alt_ok = !(isnan(altitude) || isinf(altitude));
            if (sensors[i].alt_ok) {
                sensors[i].altitude = altitude + _alt_offset_active;
            }
        }
    }

    // ensure the climb rate filter is updated
    if (healthy()) {
        _climb_rate_filter.update(get_altitude(), get_last_update());
    }

    // choose primary sensor
    if (_primary_baro >= 0 && _primary_baro < _num_sensors && healthy(_primary_baro)) {
        _primary = _primary_baro;
    } else {
        _primary = 0;
        for (uint8_t i=0; i<_num_sensors; i++) {
            if (healthy(i)) {
                _primary = i;
                break;
            }
        }
    }
}

/*
  call accumulate on all drivers
 */
void AP_Depth::accumulate(void)
{
    for (uint8_t i=0; i<_num_drivers; i++) {
        drivers[i]->accumulate();
    }
}


/* register a new sensor, claiming a sensor slot. If we are out of
   slots it will panic
*/
uint8_t AP_Depth::register_sensor(void)
{
    if (_num_sensors >= DEPTH_MAX_INSTANCES) {
        AP_HAL::panic("Too many depth sensors");
    }
    return _num_sensors++;
}


/*
  check if all barometers are healthy
 */
bool AP_Depth::all_healthy(void) const
{
     for (uint8_t i=0; i<_num_sensors; i++) {
         if (!healthy(i)) {
             return false;
         }
     }
     return _num_sensors > 0;
}

// set a pressure correction from AP_TempCalibration
void AP_Depth::set_pressure_correction(uint8_t instance, float p_correction)
{
    if (instance < _num_sensors) {
        sensors[instance].p_correction = p_correction;
    }
}

namespace AP {

AP_Depth &depth()
{
    return *AP_Depth::get_singleton();
}

};
