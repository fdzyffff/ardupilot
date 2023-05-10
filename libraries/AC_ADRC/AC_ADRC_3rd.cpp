/// @file	AC_ADRC_3rd.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_ADRC_3rd.h"
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AC_ADRC_3rd::var_info[] = {

    AP_GROUPINFO("B", 0, AC_ADRC_3rd, _b0, 0),

    AP_GROUPINFO("L1", 1, AC_ADRC_3rd, _l1, 0),
    AP_GROUPINFO("L2", 2, AC_ADRC_3rd, _l2, 0),
    AP_GROUPINFO("L3", 3, AC_ADRC_3rd, _l3, 0),
    AP_GROUPINFO("L4", 4, AC_ADRC_3rd, _l4, 0),
    AP_GROUPINFO("KA", 5, AC_ADRC_3rd, _ka, 0),
    AP_GROUPINFO("KB", 6, AC_ADRC_3rd, _kb, 0),
    AP_GROUPINFO("KC", 7, AC_ADRC_3rd, _kc, 0),
    AP_GROUPINFO("KD", 8, AC_ADRC_3rd, _kd, 0),


    AP_GROUPINFO("OMEGAO", 9, AC_ADRC_3rd, _omega_o, 0),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 10, AC_ADRC_3rd, _kimax, 0),
    // @Param: FLTT
    // @DisplayName: PID Target filter frequency in Hz
    // @Description: Target filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTT", 11, AC_ADRC_3rd, _filt_T_hz, AC_ADRC_TFILT_HZ_DEFAULT),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTE", 12, AC_ADRC_3rd, _filt_E_hz, AC_ADRC_EFILT_HZ_DEFAULT),

    // @Param: FLTD
    // @DisplayName: PID Derivative term filter frequency in Hz
    // @Description: Derivative filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTD", 13, AC_ADRC_3rd, _filt_D_hz, AC_ADRC_DFILT_HZ_DEFAULT),
    // @Param: SMAX
    // @DisplayName: Slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("SMAX", 14, AC_ADRC_3rd, _slew_rate_max, 0),

    AP_GROUPINFO("INFO", 15, AC_ADRC_3rd, _print_info, 0),

    AP_GROUPEND
};

// Constructor
AC_ADRC_3rd::AC_ADRC_3rd(float initial_b0,
                            float intial_l1, 
                            float intial_l2, 
                            float intial_l3, 
                            float intial_l4, 
                            float intial_ka, 
                            float intial_kb, 
                            float intial_kc, 
                            float intial_kd, 
                            float initial_omega_o, 
                            float initial_imax, 
                            float initial_filt_T_hz, 
                            float initial_filt_E_hz, 
                            float initial_filt_D_hz, 
                            float initial_srmax, 
                            float initial_srtau)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _b0.set_and_default(initial_b0);
    // _omegac.set_and_default(initial_omegac);
    _l1.set_and_default(intial_l1);
    _l2.set_and_default(intial_l2);
    _l3.set_and_default(intial_l3);
    _l4.set_and_default(intial_l4);
    _ka.set_and_default(intial_ka);
    _kb.set_and_default(intial_kb);
    _kc.set_and_default(intial_kc);
    _kd.set_and_default(intial_kd);
    _omega_o.set_and_default(initial_omega_o);
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
void AC_ADRC_3rd::filt_T_hz(float hz)
{
    _filt_T_hz.set(fabsf(hz));
}

// filt_E_hz - set error filter hz
void AC_ADRC_3rd::filt_E_hz(float hz)
{
    _filt_E_hz.set(fabsf(hz));
}

// filt_D_hz - set derivative filter hz
void AC_ADRC_3rd::filt_D_hz(float hz)
{
    _filt_D_hz.set(fabsf(hz));
}

// slew_limit - set slew limit
void AC_ADRC_3rd::slew_limit(float smax)
{
    _slew_rate_max.set(fabsf(smax));
}

//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
float AC_ADRC_3rd::update_all(float target, float measurement, float dt, bool limit)
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
    _z4_k0 = _z4_k1;

    for (uint8_t i_k = 0; i_k < AC_ADRC_MAX_DELAY-1; i_k++) {
        _u_k[i_k] = _u_k[i_k+1];
    }
    _u_k[AC_ADRC_MAX_DELAY-1] = _u_k1;

    _u_k0 = _u_k[0]; // 0 is the oldest
    _y_k0 = _target - _error;
    _r_k1 = _target;

    _beta1 = _ka*_omega_o;                            // 10
    _beta2 = _kb*_omega_o*_omega_o;                   // 35
    _beta3 = _kc*_omega_o*_omega_o*_omega_o;          // 50
    _beta4 = _kd*_omega_o*_omega_o*_omega_o*_omega_o; // 24

    // calculate output
    // gcs().send_text(MAV_SEVERITY_INFO, "CC3 P7");
    _z1_k1 = (1-dt*_beta1)*_z1_k0 + dt*_z2_k0 + dt*_beta1*_y_k0;
    _z2_k1 = -dt*_beta2*_z1_k0    + _z2_k0    + dt*_z3_k0        + dt*_beta2*_y_k0;
    _z3_k1 = -dt*_beta3*_z1_k0    + _z3_k0    + dt*_z4_k0        + dt*_b0*_u_k0      + dt*_beta3*_y_k0;
    _z4_k1 = -dt*_beta4*_z1_k0    + _z4_k0    + dt*_beta4*_y_k0;

    // gcs().send_text(MAV_SEVERITY_INFO, "CC3 P8");
    _u_k1 = (1/_b0) * (_l3*(_r_k1-_z1_k1) - _l2*_z2_k1 - _l1*_z3_k1 - _z4_k1);
    if (_print_info.get()) {
        gcs().send_text(MAV_SEVERITY_INFO, "[%f, %f, %f, %f]CC3 uk1 %f", _l1.get(), _l2.get(), _l3.get(), _l4.get(), _u_k1);
    }
    // _u_k1 = constrain_float(_u_k1, -1.0f, 1.0f);
    return _u_k1;
}


void AC_ADRC_3rd::reset_I()
{
    _z3_k1 = 0.0;
}

void AC_ADRC_3rd::load_gains()
{
    _b0.load();
    // _omegac.load();
    _l1.load();
    _l2.load();
    _l3.load();
    _l4.load();
    _ka.load();
    _kb.load();
    _kc.load();
    _kd.load();
    _omega_o.load();
    _kimax.load();
    _kimax.set_and_save(fabsf(_kimax.get()));
    _filt_T_hz.load();
    _filt_E_hz.load();
    _filt_D_hz.load();
    _slew_rate_max.load();
}

// save_gains - save gains to eeprom
void AC_ADRC_3rd::save_gains()
{
    _b0.save();
    // _omegac.save();
    _l1.save();
    _l2.save();
    _l3.save();
    _l4.save();
    _ka.save();
    _kb.save();
    _kc.save();
    _kd.save();
    _omega_o.save();
    _kimax.save();
    _filt_T_hz.save();
    _filt_E_hz.save();
    _filt_D_hz.save();
    _slew_rate_max.save();
}

/// Overload the function call operator to permit easy initialisation
void AC_ADRC_3rd::operator()(float b0_val, 
                            float l1_val,
                            float l2_val,
                            float l3_val,
                            float l4_val,
                            float ka_val,
                            float kb_val,
                            float kc_val,
                            float kd_val,
                            float omega_o_val,
                            float imax_val, 
                            float filt_T_hz_val, 
                            float filt_E_hz_val, 
                            float filt_D_hz_val, 
                            float slew_rate_max_val)
{
    _b0.set(b0_val);
    // _omegac.set(omegac_val);
    _l1.set(l1_val);
    _l2.set(l2_val);
    _l3.set(l3_val);
    _l4.set(l4_val);
    _ka.set(ka_val);
    _kb.set(kb_val);
    _kc.set(kc_val);
    _kd.set(kd_val);
    _omega_o.set(omega_o_val);
    _kimax.set(imax_val);
    _filt_T_hz.set(filt_T_hz_val);
    _filt_E_hz.set(filt_E_hz_val);
    _filt_D_hz.set(filt_D_hz_val);
    _slew_rate_max.set(slew_rate_max_val);
}

// get_filt_T_alpha - get the target filter alpha
float AC_ADRC_3rd::get_filt_T_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_T_hz);
}

// get_filt_E_alpha - get the error filter alpha
float AC_ADRC_3rd::get_filt_E_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_E_hz);
}

// get_filt_D_alpha - get the derivative filter alpha
float AC_ADRC_3rd::get_filt_D_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_D_hz);
}

void AC_ADRC_3rd::set_integrator(float target, float measurement, float integrator)
{
    set_integrator(target - measurement, integrator);
}

void AC_ADRC_3rd::set_integrator(float error, float integrator)
{
    _z3_k1 = -constrain_float(integrator - error * _l1, -_kimax, _kimax);
}

void AC_ADRC_3rd::set_integrator(float integrator)
{
    _z3_k1 = -constrain_float(integrator, -_kimax, _kimax);
}

void AC_ADRC_3rd::relax_integrator(float integrator, float dt, float time_constant)
{
    float temp_integrator = -_z3_k1;
    integrator = constrain_float(integrator, -_kimax, _kimax);
    if (is_positive(dt)) {
        temp_integrator = temp_integrator + (integrator - temp_integrator) * (dt / (dt + time_constant));
    }
    _z3_k1 = -temp_integrator;
}
