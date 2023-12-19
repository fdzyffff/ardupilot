#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_PITCH_RANGE", 0, UserParameters, user_pitch_range, 2500.0f),
    AP_GROUPINFO("_PITCH_RATE", 1, UserParameters, user_pitch_rate, 0.5f),
    
    AP_GROUPEND
};
