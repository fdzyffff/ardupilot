#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    // Note that there should be LESS THAN 63 parameters in one group
    AP_GROUPINFO("_STAT_PRINT", 0, UserParameters, stat_print, 0),
    AP_GROUPINFO("_NC_ANG_MAX", 1, UserParameters, netgun_max, 9000),
    AP_GROUPINFO("_NC_ANG_NIN", 2, UserParameters, netgun_min, 0),
    AP_GROUPINFO("_NC_SLEW", 3, UserParameters, netgun_slew, 5000),
    
    AP_GROUPEND
};
