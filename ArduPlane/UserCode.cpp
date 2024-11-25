#include "Plane.h"

void Plane::userhook_init()
{
    //;
}

void Plane::userhook_100Hz()
{
    //;
}

void Plane::userhook_1Hz()
{
    //;
}

void Plane::send_user_1()
{
    global_position_int_packet.time_boot_ms = millis();
    global_position_int_packet.lat          = global_position_current_loc.lat;                 // in 1E7 degrees
    global_position_int_packet.lon          = global_position_current_loc.lng;                 // in 1E7 degrees
    global_position_int_packet.alt          = global_position_current_loc.alt * 10UL;  
}
