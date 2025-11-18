#include "UserParameters.h"
#include "config.h"

#if USER_PARAMS_ENABLED
// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_SET_ORIGIN", 0, UserParameters, set_origin, 0),
    AP_GROUPINFO("_POS_OFF_X", 1, UserParameters, origin_pos_off_x, 0.0f),
    AP_GROUPINFO("_POS_OFF_Y", 2, UserParameters, origin_pos_off_y, 0.0f),
    AP_GROUPINFO("_POS_OFF_Z", 3, UserParameters, origin_pos_off_z, 0.0f),

    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
