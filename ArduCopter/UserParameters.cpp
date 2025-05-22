#include "UserParameters.h"
#include "config.h"

#if USER_PARAMS_ENABLED == ENABLED
// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_ROLE", 0, UserParameters, role, 0),
    AP_GROUPINFO("_DR", 1, UserParameters, detection_R, 15.0f),
    AP_GROUPINFO("_CR", 2, UserParameters, connection_R, 15.0f),

    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
