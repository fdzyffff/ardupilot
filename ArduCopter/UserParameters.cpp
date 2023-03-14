#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    // Note that there should be LESS THAN 63 parameters in one group
    AP_GROUPINFO("_NG_ANG_MAX", 0, UserParameters, netgun_max, 9000),
    AP_GROUPINFO("_NG_ANG_NIN", 1, UserParameters, netgun_min, 0),
    
    AP_GROUPEND
};
