#include "UserParameters.h"
#include "config.h"

#if USER_PARAMS_ENABLED == ENABLED
// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_FLT_GYRO", 0, UserParameters, filt_gyro_hz, 200.f),
    AP_GROUPINFO("_FLT_ACC", 1, UserParameters, filt_acc_hz, 200.f),
    // @Param: _XY_P
    // @DisplayName: FlowHold P gain
    // @Description: FlowHold (horizontal) P gain.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _XY_I
    // @DisplayName: FlowHold I gain
    // @Description: FlowHold (horizontal) I gain
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _XY_IMAX
    // @DisplayName: FlowHold Integrator Max
    // @Description: FlowHold (horizontal) integrator maximum
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cdeg
    // @User: Advanced

    // @Param: _XY_FILT_HZ
    // @DisplayName: FlowHold filter on input to control
    // @Description: FlowHold (horizontal) filter on input to control
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(assit_pi_xy, "_AST_",  2, UserParameters, AC_PI_2D),

    // the following coordinates are for CMAC, in Canberra
    // @Param: OPOS_LAT
    // @DisplayName: Original Position (Latitude)
    // @Description: Specifies vehicle's startup latitude
    // @User: Advanced
    AP_GROUPINFO("_OPOS_LAT",    3, UserParameters,  opos.lat, 0.0f),
    // @Param: OPOS_LNG
    // @DisplayName: Original Position (Longitude)
    // @Description: Specifies vehicle's startup longitude
    // @User: Advanced
    AP_GROUPINFO("_OPOS_LNG",    4, UserParameters,  opos.lng, 0.0f),
    // @Param: OPOS_ALT
    // @DisplayName: Original Position (Altitude)
    // @Description: Specifies vehicle's startup altitude (AMSL)
    // @User: Advanced
    AP_GROUPINFO("_OPOS_ALT",    5, UserParameters,  opos.alt, 0.0f),
    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
