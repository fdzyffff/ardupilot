#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_HIL", 0, UserParameters, usr_hil_mode, 0),
    // @Param: _XY_P
    // @DisplayName: MyVel P gain
    // @Description: MyVel (horizontal) P gain.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _XY_I
    // @DisplayName: MyVel I gain
    // @Description: MyVel (horizontal) I gain
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _XY_IMAX
    // @DisplayName: MyVel Integrator Max
    // @Description: MyVel (horizontal) integrator maximum
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cdeg
    // @User: Advanced

    // @Param: _XY_FILT_HZ
    // @DisplayName: MyVel filter on input to control
    // @Description: MyVel (horizontal) filter on input to control
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(myvel_pi_xy, "_XY_",  1, UserParameters, AC_PI_2D),

    // @Param: _FILT_HZ
    // @DisplayName: MyVel Filter Frequency
    // @Description: Filter frequency for flow data
    // @Range: 1 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_FILT_HZ", 2, UserParameters, myvel_filter_hz, 1),
    AP_GROUPINFO("_HIL_TEST", 3, UserParameters, usr_hil_test, 0),
    AP_GROUPEND
};
