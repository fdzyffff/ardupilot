#include "FD_WaterSlide.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Landing/AP_Landing.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
# define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
# define Debug(fmt, args ...)
#endif
//Debug("%.2f %.2f %.2f %.2f \n", var1, var2, var3, var4);

// table of user settable parameters
const AP_Param::GroupInfo FD_WaterSlide::var_info[] = {

    // @Param: CLMB_MAX
    // @DisplayName: Maximum Climb Rate (metres/sec)
    // @Description: Maximum demanded climb rate. Do not set higher than the climb speed at THR_MAX at TRIM_ARSPD_CM when the battery is at low voltage. Reduce value if airspeed cannot be maintained on ascent. Increase value if throttle does not increase significantly to ascend.
    // @Increment: 0.1
    // @Range: 0.1 20.0
    // @User: Standard
    AP_GROUPINFO("CLMB_MAX",    0, FD_WaterSlide, _maxClimbRate, 5.0f),

    // @Param: SINK_MIN
    // @DisplayName: Minimum Sink Rate (metres/sec)
    // @Description: Minimum sink rate when at THR_MIN and TRIM_ARSPD_CM.
    // @Increment: 0.1
    // @Range: 0.1 10.0
    // @User: Standard
    AP_GROUPINFO("SINK_MIN",    1, FD_WaterSlide, _minSinkRate, 2.0f),

    // @Param: TIME_CONST
    // @DisplayName: Controller time constant (sec)
    // @Description: Time constant of the TECS control algorithm. Small values make faster altitude corrections but can cause overshoot and aggressive behavior.
    // @Range: 3.0 10.0
    // @Increment: 0.2
    // @User: Advanced
    AP_GROUPINFO("TIME_CONST",  2, FD_WaterSlide, _timeConst, 5.0f),

    // @Param: THR_DAMP
    // @DisplayName: Controller throttle damping
    // @Description: Damping gain for throttle demand loop. Increase to add throttle activity to dampen oscillations in speed and height.
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("THR_DAMP",    3, FD_WaterSlide, _thrDamp, 0.5f),

    // @Param: INTEG_GAIN
    // @DisplayName: Controller integrator
    // @Description: Integrator gain to trim out long-term speed and height errors.
    // @Range: 0.0 0.5
    // @Increment: 0.02
    // @User: Advanced
    AP_GROUPINFO("INTEG_GAIN", 4, FD_WaterSlide, _integGain, 0.3f),

    // @Param: SPD_OMEGA
    // @DisplayName: Speed complementary filter frequency (radians/sec)
    // @Description: This is the cross-over frequency of the complementary filter used to fuse longitudinal acceleration and airspeed to obtain a lower noise and lag estimate of airspeed.
    // @Range: 0.5 2.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("SPD_OMEGA", 5, FD_WaterSlide, _spdCompFiltOmega, 2.0f),

    // @Param: RLL2THR
    // @DisplayName: Bank angle compensation gain
    // @Description: Gain from bank angle to throttle to compensate for loss of airspeed from drag in turns. Set to approximately 10x the sink rate in m/s caused by a 45-degree turn. High efficiency models may need less while less efficient aircraft may need more. Should be tuned in an automatic mission with waypoints and turns greater than 90 degrees. Tune with PTCH2SRV_RLL and KFF_RDDRMIX to achieve constant airspeed, constant altitude turns.
    // @Range: 5.0 30.0
    // @Increment: 1.0
    // @User: Advanced
    AP_GROUPINFO("RLL2THR",  6, FD_WaterSlide, _rollComp, 10.0f),


    AP_GROUPINFO("SPEED",  7, FD_WaterSlide, _EAS_target, 10.0f),

    AP_GROUPINFO("STEER_SPD",  8, FD_WaterSlide, _steer_speed, 5.0f),


    AP_GROUPEND
};

/*
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 *    Selectable speed or height priority modes when calculating pitch angle
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to
 *    height priority
 *  - Underspeed protection that demands maximum throttle and switches pitch angle control
 *    to speed priority mode
 *  - Relative ease of tuning through use of intuitive time constant, integrator and damping gains and the use
 *    of easy to measure aircraft performance data
 *
 */

void FD_WaterSlide::update_50hz(void)
{
    // Implement third order complementary filter for height and height rate
    // estimated height rate = _climb_rate
    // estimated height above field elevation  = _height
    // Reference Paper :
    // Optimizing the Gains of the Baro-Inertial Vertical Channel
    // Widnall W.S, Sinha P.K,
    // AIAA Journal of Guidance and Control, 78-1307R

    // Calculate time in seconds since last update
    uint64_t now = AP_HAL::micros64();
    float DT = (now - _update_50hz_last_usec) * 1.0e-6f;
    _flags.reset = DT > 1.0f;
    if (_flags.reset) {
        DT = 0.02f; // when first starting TECS, use most likely time constant
        _vdot_filter.reset();
    }
    _update_50hz_last_usec = now;

    // Update the speed estimate using a 2nd order complementary filter
    _update_speed(DT);
}

void FD_WaterSlide::_update_speed(float DT)
{
    // Update and average speed rate of change

    // calculate a low pass filtered _vel_dot
    if (_flags.reset) {
        _vdot_filter.reset();
        _vel_dot_lpf = _vel_dot;
    } else {
        // Get DCM
        const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
        // Calculate speed rate of change
        float temp = rotMat.c.x * GRAVITY_MSS + AP::ins().get_accel().x;
        // take 5 point moving average
        _vel_dot = _vdot_filter.apply(temp);
        const float alpha = DT / (DT + timeConstant());
        _vel_dot_lpf = _vel_dot_lpf * (1.0f - alpha) + _vel_dot * alpha;
    }

    bool use_airspeed = _ahrs.airspeed_sensor_enabled();

    // Convert equivalent airspeeds to true airspeeds and harmonise limits

    float EAS2TAS = _ahrs.get_EAS2TAS();
    _TAS_dem = _EAS_dem * EAS2TAS;
    if (_flags.reset || !use_airspeed) {
        _TASmax = aparm.airspeed_max * EAS2TAS;
    } else if (_thr_clip_status == clipStatus::MAX) {
        // wind down airspeed upper limit  to prevent a situation where the aircraft can't climb
        // at the maximum speed
        const float velRateMin = 0.5f * _STEdot_min / MAX(_TAS_state, aparm.airspeed_min * EAS2TAS);
        _TASmax += _DT * velRateMin;
        _TASmax = MAX(_TASmax, 0.01f * (float)aparm.airspeed_cruise_cm * EAS2TAS);
    } else {
        // wind airspeed upper limit back to parameter defined value
        const float velRateMax = 0.5f * _STEdot_max / MAX(_TAS_state, aparm.airspeed_min * EAS2TAS);
        _TASmax += _DT * velRateMax;
    }
    _TASmax   = MIN(_TASmax, aparm.airspeed_max * EAS2TAS);
    _TASmin   = aparm.airspeed_min * EAS2TAS;

    if (aparm.stall_prevention) {
        // when stall prevention is active we raise the minimum
        // airspeed based on aerodynamic load factor
        _TASmin *= _load_factor;
    }

    if (_TASmax < _TASmin) {
        _TASmax = _TASmin;
    }

    // Get measured airspeed or default to trim speed and constrain to range between min and max if
    // airspeed sensor data cannot be used
    if (!use_airspeed || !_ahrs.airspeed_estimate(_EAS)) {
        // If no airspeed available use average of min and max
        _EAS = constrain_float(0.01f * (float)aparm.airspeed_cruise_cm.get(), (float)aparm.airspeed_min.get(), (float)aparm.airspeed_max.get());
    }

    // limit the airspeed to a minimum of 3 m/s
    const float min_airspeed = 3.0;

    // Reset states of time since last update is too large
    if (_flags.reset) {
        _TAS_state = (_EAS * EAS2TAS);
        _TAS_state = MAX(_TAS_state, min_airspeed);
        _integDTAS_state = 0.0f;
        return;
    }

    // Implement a second order complementary filter to obtain a
    // smoothed airspeed estimate
    // airspeed estimate is held in _TAS_state
    float aspdErr = (_EAS * EAS2TAS) - _TAS_state;
    float integDTAS_input = aspdErr * _spdCompFiltOmega * _spdCompFiltOmega;
    // Prevent state from winding up
    if (_TAS_state < 3.1f) {
        integDTAS_input = MAX(integDTAS_input, 0.0f);
    }
    _integDTAS_state = _integDTAS_state + integDTAS_input * DT;
    float TAS_input = _integDTAS_state + _vel_dot + aspdErr * _spdCompFiltOmega * 1.4142f;
    _TAS_state = _TAS_state + TAS_input * DT;
    _TAS_state = MAX(_TAS_state, min_airspeed);

}

void FD_WaterSlide::_update_speed_demand(void)
{
    // Set the airspeed demand to the minimum value if an underspeed condition exists
    // or a bad descent condition exists
    // This will minimise the rate of descent resulting from an engine failure,
    // enable the maximum climb rate to be achieved and prevent continued full power descent
    // into the ground due to an unachievable airspeed value
    if ((_flags.badDescent) || (_flags.underspeed)) {
        _TAS_dem     = _TASmin;
    }

    // Constrain speed demand, taking into account the load factor
    _TAS_dem = constrain_float(_TAS_dem, _TASmin, _TASmax);

    // calculate velocity rate limits based on physical performance limits
    // provision to use a different rate limit if bad descent or underspeed condition exists
    // Use 50% of maximum energy rate to allow margin for total energy contgroller
    const float velRateMax = 0.5f * _STEdot_max / _TAS_state;
    const float velRateMin = 0.5f * _STEdot_min / _TAS_state;
    const float TAS_dem_previous = _TAS_dem_adj;

    // Apply rate limit
    if ((_TAS_dem - TAS_dem_previous) > (velRateMax * _DT)) {
        _TAS_dem_adj = TAS_dem_previous + velRateMax * _DT;
        _TAS_rate_dem = velRateMax;
    } else if ((_TAS_dem - TAS_dem_previous) < (velRateMin * _DT)) {
        _TAS_dem_adj = TAS_dem_previous + velRateMin * _DT;
        _TAS_rate_dem = velRateMin;
    } else {
        _TAS_rate_dem = (_TAS_dem - TAS_dem_previous) / _DT;
        _TAS_dem_adj = _TAS_dem;
    }

    // calculate a low pass filtered _TAS_rate_dem
    if (_flags.reset) {
        _TAS_dem_adj = _TAS_state;
        _TAS_rate_dem_lpf = _TAS_rate_dem;
    } else {
        const float alpha = _DT / (_DT + timeConstant());
        _TAS_rate_dem_lpf = _TAS_rate_dem_lpf * (1.0f - alpha) + _TAS_rate_dem * alpha;
    }

    // Constrain speed demand again to protect against bad values on initialisation.
    _TAS_dem_adj = constrain_float(_TAS_dem_adj, _TASmin, _TASmax);
}


void FD_WaterSlide::_detect_underspeed(void)
{
    _flags.underspeed = false;
}

void FD_WaterSlide::_update_energies(void)
{
    // Calculate specific energy demands
    _SKE_dem = 0.5f * _TAS_dem_adj * _TAS_dem_adj;

    // Calculate specific energy rate demands and high pass filter demanded airspeed
    // rate of change to match the filtering applied to the measurement
    _SKEdot_dem = _TAS_state * (_TAS_rate_dem - _TAS_rate_dem_lpf);

    // Calculate specific energy
    _SKE_est = 0.5f * _TAS_state * _TAS_state;

    // Calculate specific energy rate and high pass filter airspeed rate of change
    // to remove effect of complementary filter induced bias errors
    _SKEdot = _TAS_state * (_vel_dot - _vel_dot_lpf);

}

void FD_WaterSlide::_detect_allow_steering(void)
{
    if (_ahrs.airspeed_sensor_enabled() && _TAS_state > _steer_speed) {
        _flags.allow_steering = false;
    } else {
        _flags.allow_steering = true;
    }
}

/*
  current time constant. It is lower in landing to try to give a precise approach
 */
float FD_WaterSlide::timeConstant(void) const
{
    if (_timeConst < 0.1f) {
        return 0.1f;
    }
    return _timeConst;
}

/*
  calculate throttle demand - airspeed enabled case
 */
void FD_WaterSlide::_update_throttle_with_airspeed(void)
{
    // Calculate limits to be applied to potential energy error to prevent over or underspeed occurring due to large height errors
    // float SPE_err_max = MAX(_SKE_est - 0.5f * _TASmin * _TASmin, 0.0f);
    // float SPE_err_min = MIN(_SKE_est - 0.5f * _TASmax * _TASmax, 0.0f);

    // Calculate total energy error
    _STE_error = _SKE_dem - _SKE_est;
    float STEdot_dem = constrain_float(_SKEdot_dem, _STEdot_min, _STEdot_max);
    float STEdot_error = STEdot_dem - _SKEdot;

    // Apply 0.5 second first order filter to STEdot_error
    // This is required to remove accelerometer noise from the  measurement
    const float filt_coef = 2.0f * _DT;
    STEdot_error = filt_coef * STEdot_error + (1.0f - filt_coef) * _STEdotErrLast;
    _STEdotErrLast = STEdot_error;

    // Calculate throttle demand
    // If underspeed condition is set, then demand full throttle
        // Calculate gain scaler from specific energy error to throttle
        // (_STEdot_max - _STEdot_min) / (_THRmaxf - _THRminf) is the derivative of STEdot wrt throttle measured across the max allowed throttle range.
        const float K_STE2Thr = 1 / (timeConstant() * (_STEdot_max - _STEdot_min) / (_THRmaxf - _THRminf));

        // Calculate feed-forward throttle
        const float nomThr = aparm.throttle_cruise * 0.01f;
        const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
        // Use the demanded rate of change of total energy as the feed-forward demand, but add
        // additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
        // drag increase during turns.
        const float cosPhi = sqrtf((rotMat.a.y*rotMat.a.y) + (rotMat.b.y*rotMat.b.y));
        STEdot_dem = STEdot_dem + _rollComp * (1.0f/constrain_float(cosPhi * cosPhi, 0.1f, 1.0f) - 1.0f);
        const float ff_throttle = nomThr + STEdot_dem / (_STEdot_max - _STEdot_min) * (_THRmaxf - _THRminf);

        // Calculate PD + FF throttle
        float throttle_damp = _thrDamp;

        _throttle_dem = (_STE_error + STEdot_error * throttle_damp) * K_STE2Thr + ff_throttle;

        float THRminf_clipped_to_zero = constrain_float(_THRminf, 0, _THRmaxf);

        // Calculate integrator state upper and lower limits
        // Set to a value that will allow 0.1 (10%) throttle saturation to allow for noise on the demand
        // Additionally constrain the integrator state amplitude so that the integrator comes off limits faster.
        const float maxAmp = 0.5f*(_THRmaxf - THRminf_clipped_to_zero);
        const float integ_max = constrain_float((_THRmaxf - _throttle_dem + 0.1f),-maxAmp,maxAmp);
        const float integ_min = constrain_float((_THRminf - _throttle_dem - 0.1f),-maxAmp,maxAmp);

        // Calculate integrator state, constraining state
        // Set integrator to a max throttle value during climbout
        _integTHR_state = _integTHR_state + (_STE_error * _get_i_gain()) * _DT * K_STE2Thr;
        _integTHR_state = constrain_float(_integTHR_state, integ_min, integ_max);
 
        // Rate limit PD + FF throttle
        // Calculate the throttle increment from the specified slew time
        int8_t throttle_slewrate = aparm.throttle_slewrate;

        if (throttle_slewrate != 0) {
            const float thrRateIncr = _DT * (_THRmaxf - THRminf_clipped_to_zero) * throttle_slewrate * 0.01f;

            _throttle_dem = constrain_float(_throttle_dem,
                                            _last_throttle_dem - thrRateIncr,
                                            _last_throttle_dem + thrRateIncr);
            _last_throttle_dem = _throttle_dem;
        }

        // Sum the components.
        _throttle_dem = _throttle_dem + _integTHR_state;

        // if (AP::logger().should_log(_log_bitmask)){
        //     AP::logger().WriteStreaming("TEC3","TimeUS,KED,PED,KEDD,PEDD,TEE,TEDE,FFT,Imin,Imax,I,Emin,Emax",
        //                                 "Qffffffffffff",
        //                                 AP_HAL::micros64(),
        //                                 (double)_SKEdot,
        //                                 (double)_SPEdot,
        //                                 (double)_SKEdot_dem,
        //                                 (double)_SPEdot_dem,
        //                                 (double)_STE_error,
        //                                 (double)STEdot_error,
        //                                 (double)ff_throttle,
        //                                 (double)integ_min,
        //                                 (double)integ_max,
        //                                 (double)_integTHR_state,
        //                                 (double)SPE_err_min,
        //                                 (double)SPE_err_max);
        // }

    // Constrain throttle demand and record clipping
    if (_throttle_dem > _THRmaxf) {
        _thr_clip_status = clipStatus::MAX;
        _throttle_dem = _THRmaxf;
    } else if (_throttle_dem < _THRminf) {
        _thr_clip_status = clipStatus::MIN;
        _throttle_dem = _THRminf;
    } else {
        _thr_clip_status = clipStatus::NONE;
    }
}

float FD_WaterSlide::_get_i_gain(void)
{
    float i_gain = _integGain;
    return i_gain;
}

/*
  calculate throttle, non-airspeed case
 */
void FD_WaterSlide::_update_throttle_without_airspeed(int16_t throttle_nudge)
{
    // reset clip status after possible use of synthetic airspeed
    _thr_clip_status = clipStatus::NONE;

    // Calculate throttle demand by interpolating between pitch and throttle limits
    float nomThr;
    //If landing and we don't have an airspeed sensor and we have a non-zero
    //TECS_LAND_THR param then use it
    nomThr = (aparm.throttle_cruise + throttle_nudge)* 0.01f;

    _throttle_dem = nomThr;

    // Calculate additional throttle for turn drag compensation including throttle nudging
    const Matrix3f &rotMat = _ahrs.get_rotation_body_to_ned();
    // Use the demanded rate of change of total energy as the feed-forward demand, but add
    // additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
    // drag increase during turns.
    float cosPhi = sqrtf((rotMat.a.y*rotMat.a.y) + (rotMat.b.y*rotMat.b.y));
    float STEdot_dem = _rollComp * (1.0f/constrain_float(cosPhi * cosPhi, 0.1f, 1.0f) - 1.0f);
    _throttle_dem = _throttle_dem + STEdot_dem / (_STEdot_max - _STEdot_min) * (_THRmaxf - _THRminf);
}

void FD_WaterSlide::_detect_bad_descent(void)
{
    _flags.badDescent = false;
}

void FD_WaterSlide::_initialise_states()
{
    _flags.reset = false;

    // Initialise states and variables if DT > 0.2 second or in climbout
    if (_DT > 0.2f || _need_reset) {
        _integTHR_state       = 0.0f;
        _last_throttle_dem    = aparm.throttle_cruise * 0.01f;
        _TAS_dem_adj          = _TAS_dem;
        _flags.reset          = true;
        _DT                   = 0.02f; // when first starting TECS, use the most likely time constant

        _flags.underspeed            = false;
        _flags.badDescent            = false;
        _flags.reached_speed_takeoff = false;
        _flags.allow_steering        = false;
        _need_reset                  = false;
    }

    // reset takeoff speed flag when not in takeoff
    _flags.reached_speed_takeoff = false;
}

void FD_WaterSlide::_update_STE_rate_lim(void)
{
    // Calculate Specific Total Energy Rate Limits
    // This is a trivial calculation at the moment but will get bigger once we start adding altitude effects
    _STEdot_max = _maxClimbRate * GRAVITY_MSS;
    _STEdot_min = - _minSinkRate * GRAVITY_MSS;
}


void FD_WaterSlide::update_pitch_throttle(int16_t throttle_nudge, float load_factor)
{
    uint64_t now = AP_HAL::micros64();
    // check how long since we last did the 50Hz update; do nothing in
    // this loop if that hasn't run for some signficant period of
    // time.  Notably, it may never have run, leaving _TAS_state as
    // zero and subsequently division-by-zero errors.
    const float _DT_for_update_50hz = (now - _update_50hz_last_usec) * 1.0e-6f;
    if (_update_50hz_last_usec == 0 || _DT_for_update_50hz > 1.0) {
        // more than 1 second since it was run, don't do anything yet:
        return;
    }

    // Calculate time in seconds since last update
    _DT = (now - _update_pitch_throttle_last_usec) * 1.0e-6f;
    _DT = MAX(_DT, 0.001f);
    _update_pitch_throttle_last_usec = now;

    // Convert inputs
    _EAS_dem = _EAS_target;
    _load_factor = load_factor;


    _THRmaxf  = aparm.throttle_max * 0.01f;
    _THRminf  = aparm.throttle_min * 0.01f;

    // min of 1% throttle range to prevent a numerical error
    _THRmaxf = MAX(_THRmaxf, _THRminf+0.01);


    // initialise selected states and variables if DT > 1 second or in climbout
    _initialise_states();

    // Calculate Specific Total Energy Rate Limits
    _update_STE_rate_lim();

    // Calculate the speed demand
    _update_speed_demand();

    // Detect underspeed condition
    _detect_underspeed();

    // Calculate specific energy quantitiues
    _update_energies();

    // Detect allow_steering condition
    _detect_allow_steering();

    // Calculate throttle demand - use simple pitch to throttle if no
    // airspeed sensor.
    // Note that caller can demand the use of
    // synthetic airspeed for one loop if needed. This is required
    // during QuadPlane transition when pitch is constrained
    if (_ahrs.airspeed_sensor_enabled()) {
        _update_throttle_with_airspeed();
        _using_airspeed_for_throttle = true;
    } else {
        _update_throttle_without_airspeed(throttle_nudge);
        _using_airspeed_for_throttle = false;
    }

    // Detect bad descent due to demanded airspeed being too high
    _detect_bad_descent();


    // if (AP::logger().should_log(_log_bitmask)){
    //     // log to AP_Logger
    //     // @LoggerMessage: TECS
    //     // @Vehicles: Plane
    //     // @Description: Information about the Total Energy Control System
    //     // @URL: http://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html
    //     // @Field: TimeUS: Time since system startup
    //     // @Field: h: height estimate (UP) currently in use by TECS
    //     // @Field: dh: current climb rate ("delta-height")
    //     // @Field: hin: height demand received by TECS
    //     // @Field: hdem: height demand after rate limiting and filtering that TECS is currently trying to achieve
    //     // @Field: dhdem: climb rate TECS is currently trying to achieve
    //     // @Field: spdem: True AirSpeed TECS is currently trying to achieve
    //     // @Field: sp: current estimated True AirSpeed
    //     // @Field: dsp: x-axis acceleration estimate ("delta-speed")
    //     // @Field: th: throttle output
    //     // @Field: ph: pitch output
    //     // @Field: pmin: pitch lower limit
    //     // @Field: pmax: pitch upper limit
    //     // @Field: dspdem: demanded acceleration output ("delta-speed demand")
    //     // @Field: f: flags
    //     // @FieldBits: f: Underspeed,UnachievableDescent,AutoLanding,ReachedTakeoffSpd
    //     AP::logger().WriteStreaming("TECS", "TimeUS,h,dh,hin,hdem,dhdem,spdem,sp,dsp,th,ph,pmin,pmax,dspdem,f",
    //                                 "smnmmnnnn------",
    //                                 "F00000000------",
    //                                 "QfffffffffffffB",
    //                                 now,
    //                                 (double)_height,
    //                                 (double)_climb_rate,
    //                                 (double)_hgt_dem_in_raw,
    //                                 (double)_hgt_dem,
    //                                 (double)_hgt_rate_dem,
    //                                 (double)_TAS_dem_adj,
    //                                 (double)_TAS_state,
    //                                 (double)_vel_dot,
    //                                 (double)_throttle_dem,
    //                                 (double)_pitch_dem,
    //                                 (double)_PITCHminf,
    //                                 (double)_PITCHmaxf,
    //                                 (double)_TAS_rate_dem,
    //                                 _flags_byte);
    // }
}
