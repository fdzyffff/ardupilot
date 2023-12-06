/// @file    FD_WaterSlide.h
/// @brief   Combined Total Energy Speed & Height Control. This is a instance of an

/*
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 *    Selectable speed or height priority modes when calculating pitch angle
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to
 *    height priority
 *  - Underspeed protection that demands maximum throttle switches pitch angle control
 *    to speed priority mode
 *  - Relative ease of tuning through use of intuitive time constant, trim rate and damping parameters and the use
 *    of easy to measure aircraft performance data
 */
#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <Filter/AverageFilter.h>

class FD_WaterSlide {
public:
    FD_WaterSlide(AP_AHRS &ahrs, const AP_FixedWing &parms, const uint32_t log_bitmask)
        : _ahrs(ahrs)
        , aparm(parms)
        , _log_bitmask(log_bitmask)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _airspeed_min = 1.f;
        _airspeed_max = 10.f;
        _flags.enable_yaw_mix = true;
    }

    /* Do not allow copies */
    CLASS_NO_COPY(FD_WaterSlide);

    // Update of the estimated height and height rate internal state
    // Update of the inertial speed rate internal state
    // Should be called at 50Hz or greater
    void update_50hz(void);

    // Update the control loop calculations
    // Do not call slower than 10Hz or faster than 500Hz
    void update_pitch_throttle(int16_t throttle_nudge, float load_factor);

    // demanded throttle in percentage
    // should return -100 to 100, usually positive unless reverse thrust is enabled via _THRminf < 0
    float get_throttle_demand(void) {
        return _throttle_dem * 100.0f;
    }

    // Rate of change of velocity along X body axis in m/s^2
    float get_VXdot(void) {
        return _vel_dot;
    }

    // return current target airspeed
    float get_target_airspeed(void) const {
        return _TAS_dem_adj / _ahrs.get_EAS2TAS();
    }

    // return maximum climb rate
    float get_max_climbrate(void) const {
        return _maxClimbRate;
    }

    // reset throttle integrator
    void reset_throttle_I(void) {
        _integTHR_state = 0.0;
    }

    // reset on next loop
    void reset(void) {
        _need_reset = true;
    }

    bool allow_steering_flag() {
        return _flags.allow_steering;
    }

    bool get_yaw_mix_flag() {
        return _flags.enable_yaw_mix;
    }

    void set_yaw_mix_flag(bool v) {
        _flags.enable_yaw_mix = v;
    }
    // this supports the TECS_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    // Last time update_50Hz was called
    uint64_t _update_50hz_last_usec;

    // Last time update_speed was called
    uint64_t _update_speed_last_usec;

    // Last time update_pitch_throttle was called
    uint64_t _update_pitch_throttle_last_usec;

    // reference to the AHRS object
    AP_AHRS &_ahrs;

    const AP_FixedWing &aparm;

    // Logging  bitmask
    const uint32_t _log_bitmask;

    // TECS tuning parameters
    AP_Float _spdCompFiltOmega;
    AP_Float _maxClimbRate;
    AP_Float _minSinkRate;
    AP_Float _timeConst;
    AP_Float _thrDamp;
    AP_Float _integGain;
    AP_Float _rollComp;
    AP_Float _EAS_target;
    AP_Float _steer_speed;

    float _airspeed_min;
    float _airspeed_max;


    // throttle demand in the range from -1.0 to 1.0, usually positive unless reverse thrust is enabled via _THRminf < 0
    float _throttle_dem;

    // Integrator state 4 - airspeed filter first derivative
    float _integDTAS_state;

    // Integrator state 5 - true airspeed
    float _TAS_state;

    // Integrator state 6 - throttle integrator
    float _integTHR_state;

    // throttle demand rate limiter state
    float _last_throttle_dem;

    // Rate of change of speed along X axis
    float _vel_dot;
    float _vel_dot_lpf;

    // Equivalent airspeed
    float _EAS;

    // True airspeed limits
    float _TASmax;
    float _TASmin;

    // Current true airspeed demand
    float _TAS_dem;

    // Equivalent airspeed demand
    float _EAS_dem;

    // Speed demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_dem_adj;

    // Speed rate demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_rate_dem;
    float _TAS_rate_dem_lpf;

    // Total energy rate filter state
    float _STEdotErrLast;

    struct flags {
        // Underspeed condition
        bool underspeed:1;

        // Bad descent condition caused by unachievable airspeed demand
        bool badDescent:1;

        // true when plane is in auto mode and executing a land mission item
        bool is_doing_auto_land:1;

        // true when we have reached target speed in takeoff
        bool reached_speed_takeoff:1;

        // true if the soaring feature has requested gliding flight
        bool gliding_requested:1;
        // true when a reset of airspeed and height states to current is performed on this frame
        bool reset:1;

        bool allow_steering:1;

        bool enable_yaw_mix:1;
    };
    union {
        struct flags _flags;
        uint8_t _flags_byte;
    };

    // Maximum and minimum specific total energy rate limits
    float _STEdot_max;
    float _STEdot_min;

    // Maximum and minimum floating point throttle limits
    float _THRmaxf;
    float _THRminf;

    // 1 if throttle is clipping at max value, -1 if clipping at min value, 0 otherwise
    enum class clipStatus  : int8_t {
        MIN  = -1,
        NONE =  0,
        MAX  =  1,
    };
    clipStatus _thr_clip_status;

    // Specific energy quantities
    float _SKE_dem;
    float _SKEdot_dem;
    float _SKE_est;
    float _SKEdot;

    // Specific energy error quantities
    float _STE_error;

    // 1 when specific energy balance rate demand is clipping in the up direction
    // -1 when specific energy balance rate demand is clipping in the down direction
    // 0 when not clipping
    clipStatus _SEBdot_dem_clip;

    // Time since last update of main TECS loop (seconds)
    float _DT;

    // need to reset on next loop
    bool _need_reset;

    // using airspeed in throttle calculation this frame
    bool _using_airspeed_for_throttle;


    // aerodynamic load factor
    float _load_factor;

    // Update the airspeed internal state using a second order complementary filter
    void _update_speed(float DT);

    // Update the demanded airspeed
    void _update_speed_demand(void);

    // Detect an underspeed condition
    void _detect_underspeed(void);

    // Update Specific Energy Quantities
    void _update_energies(void);

    // Detect allow_steering condition
    void _detect_allow_steering(void);

    // Update Demanded Throttle
    void _update_throttle_with_airspeed(void);

    // Update Demanded Throttle Non-Airspeed
    void _update_throttle_without_airspeed(int16_t throttle_nudge);

    // get integral gain which is flight_stage dependent
    float _get_i_gain(void);

    // Detect Bad Descent
    void _detect_bad_descent(void);

    // Update Demanded Pitch Angle
    void _update_pitch(void);

    // Initialise states and variables
    void _initialise_states();

    // Calculate specific total energy rate limits
    void _update_STE_rate_lim(void);

    // declares a 5point average filter using floats
    AverageFilterFloat_Size5 _vdot_filter;

    // current time constant
    float timeConstant(void) const;
};
