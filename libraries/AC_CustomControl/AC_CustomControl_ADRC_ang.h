#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <AC_ADRC/AC_ADRC.h>

#include "AC_CustomControl_Backend.h"

#ifndef CUSTOMCONTROL_ADRC_ANG_ENABLED
    #define CUSTOMCONTROL_ADRC_ANG_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_ADRC_ANG_ENABLED


#ifndef AC_ATC_MULTI_RATE_RP_B0
  # define AC_ATC_MULTI_RATE_RP_B0          5000.0f
#endif
// #ifndef AC_ATC_MULTI_RATE_RP_OMEGAC
//   # define AC_ATC_MULTI_RATE_RP_OMEGAC      20.0f
// #endif
#ifndef AC_ATC_MULTI_RATE_RP_KP
  # define AC_ATC_MULTI_RATE_RP_KP          1.0f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_KD
  # define AC_ATC_MULTI_RATE_RP_KD          0.01f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_KA
  # define AC_ATC_MULTI_RATE_RP_KA          1.0f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_KB
  # define AC_ATC_MULTI_RATE_RP_KB          1.0f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_KC
  # define AC_ATC_MULTI_RATE_RP_KC          1.0f
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
 # define AC_ATC_MULTI_RATE_YAW_B0          5000.0f
#endif
// #ifndef AC_ATC_MULTI_RATE_YAW_OMEGAC
//  # define AC_ATC_MULTI_RATE_YAW_OMEGAC      20.0f
// #endif
#ifndef AC_ATC_MULTI_RATE_YAW_KP
  # define AC_ATC_MULTI_RATE_YAW_KP          1.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_KD
  # define AC_ATC_MULTI_RATE_YAW_KD          0.01f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_KA
  # define AC_ATC_MULTI_RATE_YAW_KA          1.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_KB
  # define AC_ATC_MULTI_RATE_YAW_KB          1.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_KC
  # define AC_ATC_MULTI_RATE_YAW_KC          1.0f
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


class AC_CustomControl_ADRC_ang : public AC_CustomControl_Backend {
public:
    AC_CustomControl_ADRC_ang(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt);

    // run lowest level body-frame rate controller and send outputs to the motors
    Vector3f update() override;
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // put controller related variable here

	// angle ADRC controller  objects
    AC_ADRC _adrc_atti_rate_roll;
    AC_ADRC _adrc_atti_rate_pitch;
    AC_ADRC _adrc_atti_rate_yaw;

  // rate PID controller  objects
    AC_PID _pid_atti_rate_roll;
    AC_PID _pid_atti_rate_pitch;
    AC_PID _pid_atti_rate_yaw;
};

#endif
