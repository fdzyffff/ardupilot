#include "UserParameters.h"
#include "config.h"

#if USER_PARAMS_ENABLED == ENABLED
// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_FLT_GYRO", 0, UserParameters, filt_gyro_hz, 200.f),
    AP_GROUPINFO("_FLT_ACC", 1, UserParameters, filt_acc_hz, 200.f),
    AP_GROUPINFO("_ASSIT", 2, UserParameters, assit_gain, 3.0f),

    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
