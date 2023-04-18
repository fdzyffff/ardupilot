#pragma once

/// @file	AC_ADRC.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include <Filter/SlewLimiter.h>

#define AC_ADRC_TFILT_HZ_DEFAULT  0.0f   // default input filter frequency
#define AC_ADRC_EFILT_HZ_DEFAULT  0.0f   // default input filter frequency
#define AC_ADRC_DFILT_HZ_DEFAULT  20.0f   // default input filter frequency
#define AC_ADRC_RESET_TC          0.16f   // Time constant for integrator reset decay to zero
#define AC_ADRC_MAX_DELAY         2


#include <AC_PID/AP_PIDInfo.h>

/// @class	AC_ADRC
/// @brief	Copter PID control class
class AC_ADRC {
public:

    // Constructor for PID
    AC_ADRC(float initial_b0, float initial_omegac, float initial_omega0, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz,
           float initial_srmax=0, float initial_srtau=1.0);

    CLASS_NO_COPY(AC_ADRC);

    //  update_all - set target and measured inputs to PID controller and calculate outputs
    //  target and error are filtered
    //  the derivative is then calculated and filtered
    //  the integral is then updated based on the setting of the limit flag
    float update_all(float target, float measurement, float dt, bool limit = false);

    //  update_error - set error input to PID controller and calculate outputs
    //  target is set to zero and error is set and filtered
    //  the derivative then is calculated and filtered
    //  the integral is then updated based on the setting of the limit flag
    //  Target and Measured must be set manually for logging purposes.
    // todo: remove function when it is no longer used.
    float update_error(float error, float dt, bool limit = false);

    // reset_I - reset the integrator
    void reset_I();

    // reset_filter - input filter will be reset to the next value provided to set_input()
    void reset_filter() {
        _flags._reset_filter = true;
    }

    // load gain from eeprom
    void load_gains();

    // save gain to eeprom
    void save_gains();

    /// operator function call for easy initialisation
    void operator()(float p_val, float i_val, float d_val, float ff_val, float imax_val, float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz);

    // get accessors
    float kP() { return _kp; }
    float kD() { return _kd; }
    AP_Float &kIMAX() { return _kimax; }
    AP_Float &filt_T_hz() { return _filt_T_hz; }
    AP_Float &filt_E_hz() { return _filt_E_hz; }
    AP_Float &filt_D_hz() { return _filt_D_hz; }
    AP_Float &slew_limit() { return _slew_rate_max; }

    float imax() const { return _kimax.get(); }
    float get_filt_T_alpha(float dt) const;
    float get_filt_E_alpha(float dt) const;
    float get_filt_D_alpha(float dt) const;

    // set accessors
    void imax(const float v) { _kimax.set(fabsf(v)); }
    void filt_T_hz(const float v);
    void filt_E_hz(const float v);
    void filt_D_hz(const float v);
    void slew_limit(const float v);

    // set the desired and actual rates (for logging purposes)
    void set_target_rate(float target) { _pid_info.target = target; }
    void set_actual_rate(float actual) { _pid_info.actual = actual; }

    // integrator setting functions
    void set_integrator(float target, float measurement, float i);
    void set_integrator(float error, float i);
    void set_integrator(float i);
    void relax_integrator(float integrator, float dt, float time_constant);

    // set slew limiter scale factor
    // void set_slew_limit_scale(int8_t scale) { _slew_limit_scale = scale; }

    // return current slew rate of slew limiter. Will return 0 if SMAX is zero
    float get_slew_rate(void) const { return _slew_limiter.get_slew_rate(); }

    const AP_PIDInfo& get_pid_info(void) const { return _pid_info; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

    // the time constant tau is not currently configurable, but is set
    // as an AP_Float to make it easy to make it configurable for a
    // single user of AC_ADRC by adding the parameter in the param
    // table of the parent class. It is made public for this reason
    AP_Float _slew_rate_tau;
    
protected:

    // parameters
    AP_Float _b0;
    AP_Float _omegac;
    AP_Float _omega0;
    AP_Float _kimax;
    AP_Float _filt_T_hz;         // PID target filter frequency in Hz
    AP_Float _filt_E_hz;         // PID error filter frequency in Hz
    AP_Float _filt_D_hz;         // PID derivative filter frequency in Hz
    AP_Float _slew_rate_max;

    SlewLimiter _slew_limiter{_slew_rate_max, _slew_rate_tau};

    // flags
    struct ac_adrc_flags {
        bool _reset_filter :1; // true when input filter should be reset during next call to set_input
    } _flags;

    // internal variables
    float _z1_k0;
    float _z2_k0;
    float _z3_k0;
    float _z1_k1;
    float _z2_k1;
    float _z3_k1;
    float _u_k0;
    float _u_k[AC_ADRC_MAX_DELAY];
    float _y_k0;
    float _r_k1;
    float _u_k1;
    float _beta1;
    float _beta2;
    float _beta3;
    float _target;            // target value to enable filtering
    float _error;             // error value to enable filtering
    float _kp;
    float _kd;
    float _derivative;        // derivative value to enable filtering

    AP_PIDInfo _pid_info;
};
