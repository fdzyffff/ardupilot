/// @file	AC_ADRC.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_ADRC.h"
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AC_ADRC::var_info[] = {

    AP_GROUPINFO("B", 0, AC_ADRC, _b0, 0),

    AP_GROUPINFO("O_C", 1, AC_ADRC, _omegac, 0),

    AP_GROUPINFO("O_0", 2, AC_ADRC, _omega0, 0),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 3, AC_ADRC, _kimax, 0),
    // @Param: FLTT
    // @DisplayName: PID Target filter frequency in Hz
    // @Description: Target filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTT", 4, AC_ADRC, _filt_T_hz, AC_ADRC_TFILT_HZ_DEFAULT),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTE", 5, AC_ADRC, _filt_E_hz, AC_ADRC_EFILT_HZ_DEFAULT),

    // @Param: FLTD
    // @DisplayName: PID Derivative term filter frequency in Hz
    // @Description: Derivative filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTD", 6, AC_ADRC, _filt_D_hz, AC_ADRC_DFILT_HZ_DEFAULT),
    // @Param: SMAX
    // @DisplayName: Slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("SMAX", 7, AC_ADRC, _slew_rate_max, 0),

    AP_GROUPEND
};

// Constructor
AC_ADRC::AC_ADRC(float initial_b0, float initial_omegac, float initial_omega0, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz, float initial_srmax, float initial_srtau)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _b0.set_and_default(initial_b0);
    _omegac.set_and_default(initial_omegac);
    _omega0.set_and_default(initial_omega0);
    _kimax.set_and_default(initial_imax);
    _filt_T_hz.set_and_default(initial_filt_T_hz);
    _filt_E_hz.set_and_default(initial_filt_E_hz);
    _filt_D_hz.set_and_default(initial_filt_D_hz);
    _slew_rate_tau.set_and_default(initial_srtau);

    // reset input filter to first value received
    _flags._reset_filter = true;

    memset(&_pid_info, 0, sizeof(_pid_info));
    memset(&_u_k, 0, sizeof(_u_k));
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
    // gcs().send_text(MAV_SEVERITY_INFO, "CC3 dt %f", dt);
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
    // gcs().send_text(MAV_SEVERITY_INFO, "CC3 P1");

    // get info from last step
    _z1_k0 = _z1_k1;
    _z2_k0 = _z2_k1;
    _z3_k0 = _z3_k1;

    for (uint8_t i_k = 0; i_k < AC_ADRC_MAX_DELAY-1; i_k++) {
        _u_k[i_k] = _u_k[i_k+1];
    }
    _u_k[AC_ADRC_MAX_DELAY-1] = _u_k1;

    _u_k0 = _u_k[0]; // 0 is the oldest
    _y_k0 = measurement;
    _r_k1 = target;

    _beta1 = 3.0f*_omega0;
    _beta2 = 3.0f*_omega0*_omega0;
    _beta3 = _omega0*_omega0*_omega0;

    // calculate output
    float a11 = 1-dt*_beta1;
    float a12 = dt;
    float a13 = 0.0f;

    float a21 = -dt*_beta2;
    float a22 = 1.0f;
    float a23 = dt;

    float a31 = -dt*_beta3;
    float a32 = 0.0f;
    float a33 = 1.0f;

    // gcs().send_text(MAV_SEVERITY_INFO, "CC3 P2");
    Matrix3F M_A = Matrix3F(a11, a12, a13,
                            a21, a22, a23,
                            a31, a32, a33);

    // gcs().send_text(MAV_SEVERITY_INFO, "CC3 P3");
    Vector3F V_Z_k0 = Vector3F(_z1_k0, _z2_k0, _z3_k0);

    // gcs().send_text(MAV_SEVERITY_INFO, "CC3 P4");
    Vector3F V_U = Vector3F(0.0, dt*_b0, 0.0);
    V_U = V_U * _u_k0;

    // gcs().send_text(MAV_SEVERITY_INFO, "CC3 P5");
    Vector3F V_Y = Vector3F(dt*_beta1, dt*_beta2, dt*_beta3);
    V_Y = V_Y * _y_k0;

    // gcs().send_text(MAV_SEVERITY_INFO, "CC3 P6");
    Vector3F V_Z_k1 = M_A * V_Z_k0 + V_U + V_Y;

    // gcs().send_text(MAV_SEVERITY_INFO, "CC3 P7");
    _z1_k1 = V_Z_k1.x;
    _z2_k1 = V_Z_k1.y;
    _z3_k1 = V_Z_k1.z;

    // gcs().send_text(MAV_SEVERITY_INFO, "CC3 P8");
    _kp = _omegac*_omegac;
    _kd = 2*_omegac;
    _u_k1 = (-_z3_k1 + _kp*(_r_k1-_z1_k1)-_kd*_z2_k1)/_b0;
    // gcs().send_text(MAV_SEVERITY_INFO, "CC3 uk1 %f", _u_k1);
    _u_k1 = constrain_float(_u_k1, -1.0f, 1.0f);
    return _u_k1;
}


void AC_ADRC::reset_I()
{
    _z3_k1 = 0.0;
}

void AC_ADRC::load_gains()
{
    _b0.load();
    _omegac.load();
    _omega0.load();
    _kimax.load();
    _kimax.set_and_save(fabsf(_kimax.get()));
    _filt_T_hz.load();
    _filt_E_hz.load();
    _filt_D_hz.load();
    _slew_rate_max.load();
}

// save_gains - save gains to eeprom
void AC_ADRC::save_gains()
{
    _b0.save();
    _omegac.save();
    _omega0.save();
    _kimax.save();
    _filt_T_hz.save();
    _filt_E_hz.save();
    _filt_D_hz.save();
    _slew_rate_max.save();
}

/// Overload the function call operator to permit easy initialisation
void AC_ADRC::operator()(float b0_val, float omegac_val, float omega0_val, float imax_val, float filt_T_hz_val, float filt_E_hz_val, float filt_D_hz_val, float slew_rate_max_val)
{
    _b0.set(b0_val);
    _omegac.set(omegac_val);
    _omega0.set(omega0_val);
    _kimax.set(imax_val);
    _filt_T_hz.set(filt_T_hz_val);
    _filt_E_hz.set(filt_E_hz_val);
    _filt_D_hz.set(filt_D_hz_val);
    _slew_rate_max.set(slew_rate_max_val);
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
    _z3_k1 = -constrain_float(integrator - error * _kp, -_kimax, _kimax);
}

void AC_ADRC::set_integrator(float integrator)
{
    _z3_k1 = -constrain_float(integrator, -_kimax, _kimax);
}

void AC_ADRC::relax_integrator(float integrator, float dt, float time_constant)
{
    float temp_integrator = -_z3_k1;
    integrator = constrain_float(integrator, -_kimax, _kimax);
    if (is_positive(dt)) {
        temp_integrator = temp_integrator + (integrator - temp_integrator) * (dt / (dt + time_constant));
    }
    _z3_k1 = -temp_integrator;
}
