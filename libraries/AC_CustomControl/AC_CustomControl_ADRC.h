#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_P.h>
#include <AC_ADRC/AC_ADRC.h>

#include "AC_CustomControl_Backend.h"

#ifndef CUSTOMCONTROL_ADRC_ENABLED
    #define CUSTOMCONTROL_ADRC_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_ADRC_ENABLED


#ifndef AC_ATC_MULTI_RATE_RP_B0
  # define AC_ATC_MULTI_RATE_RP_B0          80.0f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_OMEGAC
  # define AC_ATC_MULTI_RATE_RP_OMEGAC      20.0f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_OMEGA0
  # define AC_ATC_MULTI_RATE_RP_OMEGA0      50.0f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_IMAX
 # define AC_ATC_MULTI_RATE_RP_IMAX          0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_FILT_HZ
 # define AC_ATC_MULTI_RATE_RP_FILT_HZ      20.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_B0
 # define AC_ATC_MULTI_RATE_YAW_B0          80.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_OMEGAC
 # define AC_ATC_MULTI_RATE_YAW_OMEGAC      20.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_OMEGA0
 # define AC_ATC_MULTI_RATE_YAW_OMEGA0      50.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_IMAX
 # define AC_ATC_MULTI_RATE_YAW_IMAX        0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_FILT_HZ
 # define AC_ATC_MULTI_RATE_YAW_FILT_HZ     2.5f
#endif


class AC_CustomControl_ADRC : public AC_CustomControl_Backend {
public:
    AC_CustomControl_ADRC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt);

    // run lowest level body-frame rate controller and send outputs to the motors
    Vector3f update() override;
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // put controller related variable here

    // angle P controller  objects
    AC_P                _p_angle_roll2;
    AC_P                _p_angle_pitch2;
    AC_P                _p_angle_yaw2;

	// rate ADRC controller  objects
    AC_ADRC _adrc_atti_rate_roll;
    AC_ADRC _adrc_atti_rate_pitch;
    AC_ADRC _adrc_atti_rate_yaw;
};

#endif
