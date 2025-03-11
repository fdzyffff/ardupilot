#include "UserParameters.h"

// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_PRINT",      0, UserParameters, print,                   0),
    AP_GROUPINFO("_MODE",       1, UserParameters, angle_mode,              0),
    AP_GROUPINFO("_ATT_HZ_SAM", 2, UserParameters, freq_sample,           400),
    AP_GROUPINFO("_ATT_HZ_CUT", 3, UserParameters, freq_cutoff,            50),

    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
