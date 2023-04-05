/// @file	AC_ADRC.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_ADRC.h"

const AP_Param::GroupInfo AC_ADRC::var_info[] = {

    AP_GROUPINFO("B", 0, AC_ADRC, _b0, 0),

    AP_GROUPINFO("O_C", 1, AC_ADRC, _omegac, 0),

    AP_GROUPINFO("O_0", 1, AC_ADRC, _omega0, 0),

    // @Param: FLTT
    // @DisplayName: PID Target filter frequency in Hz
    // @Description: Target filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTT", 9, AC_ADRC, _filt_T_hz, AC_ADRC_TFILT_HZ_DEFAULT),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTE", 10, AC_ADRC, _filt_E_hz, AC_ADRC_EFILT_HZ_DEFAULT),

    // @Param: FLTD
    // @DisplayName: PID Derivative term filter frequency in Hz
    // @Description: Derivative filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTD", 11, AC_ADRC, _filt_D_hz, AC_ADRC_DFILT_HZ_DEFAULT),
    AP_GROUPEND
};

// Constructor
AC_ADRC::AC_ADRC(float initial_b0, float initial_omegac, float initial_omega0, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _b0.set_and_default(initial_b0);
    _omegac.set_and_default(initial_omegac);
    _omega0.set_and_default(initial_omega0);
    _filt_T_hz.set_and_default(initial_filt_T_hz);
    _filt_E_hz.set_and_default(initial_filt_E_hz);
    _filt_D_hz.set_and_default(initial_filt_D_hz);

    // reset input filter to first value received
    _flags._reset_filter = true;

    memset(&_pid_info, 0, sizeof(_pid_info));

    // slew limit scaler allows for plane to use degrees/sec slew
    // limit
    _slew_limit_scale = 1;
}

// filt_T_hz - set target filter hz
void AC_ADRC::filt_T_hz(float hz)
{
    _filt_T_hz.set(fabsf(hz));
}

// filt_E_hz - set error filter hz
void AC_ADRC::filt_E_hz(float hz)
{
    _filt_E_hz.set(fabsf(hz));
}

// filt_D_hz - set derivative filter hz
void AC_ADRC::filt_D_hz(float hz)
{
    _filt_D_hz.set(fabsf(hz));
}

// slew_limit - set slew limit
void AC_ADRC::slew_limit(float smax)
{
    _slew_rate_max.set(fabsf(smax));
}

//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
float AC_ADRC::update_all(float target, float measurement, float dt, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _target = target;
        _error = _target - measurement;
        _derivative = 0.0f;
    } else {
        float error_last = _error;
        _target += get_filt_T_alpha(dt) * (target - _target);
        _error += get_filt_E_alpha(dt) * ((_target - measurement) - _error);

        // calculate and filter derivative
        if (is_positive(dt)) {
            float derivative = (_error - error_last) / dt;
            _derivative += get_filt_D_alpha(dt) * (derivative - _derivative);
        }
    }

    // update I term
    update_i(dt, limit);

    float P_out = (_error * _kp);
    float D_out = (_derivative * _kd);

    // calculate slew limit modifier for P+D
    _pid_info.Dmod = _slew_limiter.modifier((_pid_info.P + _pid_info.D) * _slew_limit_scale, dt);
    _pid_info.slew_rate = _slew_limiter.get_slew_rate();

    P_out *= _pid_info.Dmod;
    D_out *= _pid_info.Dmod;

    _pid_info.target = _target;
    _pid_info.actual = measurement;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;

    return P_out + _integrator + D_out;




    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _target = target;
        _error = _target - measurement;
        _derivative = 0.0f;
    } else {
        float error_last = _error;
        _target += get_filt_T_alpha(dt) * (target - _target);
        _error += get_filt_E_alpha(dt) * ((_target - measurement) - _error);

        // calculate and filter derivative
        if (is_positive(dt)) {
            float derivative = (_error - error_last) / dt;
            _derivative += get_filt_D_alpha(dt) * (derivative - _derivative);
        }
    }


    // get info from last step
    _z1_k0 = _z1_k1;
    _z2_k0 = _z2_k1;
    _z3_k0 = _z3_k1;

    _u_k0 = _u_k1;
    _y_k0 = measurement;
    _r_k1 = _target;

    _beta1 = 3.0f*_omega0;
    _beta2 = 3.0f*_omega0*_omega0;
    _beta3 = _omega0*_omega0*_omega0;

    // calculate output
    float a11 = 1.0f/dt + _beta1;
    float a12 = -1.0f;
    float a13 = 0.0f;
    float c1 = _z1_k0/dt + (_beta1*_y_k0);

    float a21 = _beta2;
    float a22 = 1.0f/dt;
    float a23 = 1.0f;
    float c2 = _z2_k0/dt + _b0*_u_k0 + _beta2*_y_k0;

    float a31 = _beta3;
    float a32 = 0.0f;
    float a33 = 1.0/dt;
    float c3 = _z3_k0/dt + _beta3*_y_k0;

    Matrix3 M_A = (a11, a12, a13,
                   a21, a22, a23,
                   a31, a32, a33);
    Vector3F M_C = (c1, c2, c3);

    Matrix3 M_A_inv;
    if (M_A.inverse(M_A_inv)) {
        Vector3F M_X = M_A_inv * M_C;
        _z1_k1 = M_X.x;
        _z2_k1 = M_X.y;
        _z3_k1 = M_X.z;
        _kp = _omegac*_omegac;
        _kd = 2.0f*_omegac;
        _u_k1 = (-_z3_k1 + _kp*(_r_k1-_z1_k1)-_kd*_z2_k1)/b0;
    }

    return _u_k1;

}

//  update_error - set error input to PID controller and calculate outputs
//  target is set to zero and error is set and filtered
//  the derivative then is calculated and filtered
//  the integral is then updated based on the setting of the limit flag
//  Target and Measured must be set manually for logging purposes.
// todo: remove function when it is no longer used.
float AC_ADRC::update_error(float error, float dt, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(error)) {
        return 0.0f;
    }

    _target = 0.0f;

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _error = error;
        _derivative = 0.0f;
    } else {
        float error_last = _error;
        _error += get_filt_E_alpha(dt) * (error - _error);

        // calculate and filter derivative
        if (is_positive(dt)) {
            float derivative = (_error - error_last) / dt;
            _derivative += get_filt_D_alpha(dt) * (derivative - _derivative);
        }
    }

    // update I term
    update_i(dt, limit);

    float P_out = (_error * _kp);
    float D_out = (_derivative * _kd);

    // calculate slew limit modifier for P+D
    _pid_info.Dmod = _slew_limiter.modifier((_pid_info.P + _pid_info.D) * _slew_limit_scale, dt);
    _pid_info.slew_rate = _slew_limiter.get_slew_rate();

    P_out *= _pid_info.Dmod;
    D_out *= _pid_info.Dmod;
    
    _pid_info.target = 0.0f;
    _pid_info.actual = 0.0f;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;

    return P_out + _integrator + D_out;
}

//  update_i - update the integral
//  If the limit flag is set the integral is only allowed to shrink
void AC_ADRC::update_i(float dt, bool limit)
{
    if (!is_zero(_ki) && is_positive(dt)) {
        // Ensure that integrator can only be reduced if the output is saturated
        if (!limit || ((is_positive(_integrator) && is_negative(_error)) || (is_negative(_integrator) && is_positive(_error)))) {
            _integrator += ((float)_error * _ki) * dt;
            _integrator = constrain_float(_integrator, -_kimax, _kimax);
        }
    } else {
        _integrator = 0.0f;
    }
    _pid_info.I = _integrator;
    _pid_info.limit = limit;
}

float AC_ADRC::get_p() const
{
    return _error * _kp;
}

float AC_ADRC::get_i() const
{
    return _integrator;
}

float AC_ADRC::get_d() const
{
    return _kd * _derivative;
}

float AC_ADRC::get_ff()
{
    _pid_info.FF = _target * _kff;
    return _target * _kff;
}

void AC_ADRC::reset_I()
{
    _integrator = 0.0;
    _pid_info.I = 0.0;
}

void AC_ADRC::load_gains()
{
    _kp.load();
    _ki.load();
    _kd.load();
    _kff.load();
    _kimax.load();
    _kimax.set(fabsf(_kimax));
    _filt_T_hz.load();
    _filt_E_hz.load();
    _filt_D_hz.load();
}

// save_gains - save gains to eeprom
void AC_ADRC::save_gains()
{
    _kp.save();
    _ki.save();
    _kd.save();
    _kff.save();
    _kimax.save();
    _filt_T_hz.save();
    _filt_E_hz.save();
    _filt_D_hz.save();
}

/// Overload the function call operator to permit easy initialisation
void AC_ADRC::operator()(float p_val, float i_val, float d_val, float ff_val, float imax_val, float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz)
{
    _kp.set(p_val);
    _ki.set(i_val);
    _kd.set(d_val);
    _kff.set(ff_val);
    _kimax.set(fabsf(imax_val));
    _filt_T_hz.set(input_filt_T_hz);
    _filt_E_hz.set(input_filt_E_hz);
    _filt_D_hz.set(input_filt_D_hz);
}

// get_filt_T_alpha - get the target filter alpha
float AC_ADRC::get_filt_T_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_T_hz);
}

// get_filt_E_alpha - get the error filter alpha
float AC_ADRC::get_filt_E_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_E_hz);
}

// get_filt_D_alpha - get the derivative filter alpha
float AC_ADRC::get_filt_D_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_D_hz);
}

void AC_ADRC::set_integrator(float target, float measurement, float integrator)
{
    set_integrator(target - measurement, integrator);
}

void AC_ADRC::set_integrator(float error, float integrator)
{
    _integrator = constrain_float(integrator - error * _kp, -_kimax, _kimax);
    _pid_info.I = _integrator;
}

void AC_ADRC::set_integrator(float integrator)
{
    _integrator = constrain_float(integrator, -_kimax, _kimax);
    _pid_info.I = _integrator;
}

void AC_ADRC::relax_integrator(float integrator, float dt, float time_constant)
{
    integrator = constrain_float(integrator, -_kimax, _kimax);
    if (is_positive(dt)) {
        _integrator = _integrator + (integrator - _integrator) * (dt / (dt + time_constant));
    }
    _pid_info.I = _integrator;
}
