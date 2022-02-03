#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars

    AP_GROUPINFO("_EF3_RNG_ALT", 0, UserParameters, EF3_target_alt, 150.0f),
    AP_GROUPINFO("_EF3_R2Y_F", 1, UserParameters, EF3_yaw_factor, 0.5f),
    AP_GROUPINFO("_EF2_ALT_MAX", 2, UserParameters, EF2_alt_max, 300.0f),
    AP_GROUPINFO("_EF2_ALT_MIN", 3, UserParameters, EF2_alt_min, 100.0f),
    AP_GROUPINFO("_UWB_PORT_USE", 4, UserParameters, EF_use_uwb_port, 0),
    AP_GROUPINFO("_FOOTBALL_DIR", 5, UserParameters, EF_football_dir, 0.0f),

    
    AP_GROUPEND
};
