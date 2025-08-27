#include "UserParameters.h"
#include "config.h"

#if USER_PARAMS_ENABLED
// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_PRINT",    0, UserParameters, print, 0),
    AP_GROUPINFO("_LOG_RAW",  1, UserParameters, log_raw, 0),
    AP_GROUPINFO("_LOG_TEST", 2, UserParameters, log_test, 0),
    AP_GROUPINFO("_X_SPD",    3, UserParameters, x_speed, 1.0f),
    AP_GROUPINFO("_Y_SPD",    4, UserParameters, y_speed, 1.0f),
    AP_GROUPINFO("_Z_SPD",    5, UserParameters, z_speed, 1.0f),
    AP_GROUPINFO("_HEADING",  6, UserParameters, heading_correction, 0),

    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
