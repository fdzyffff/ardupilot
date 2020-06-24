#include "Plane.h"

void Plane::HB1_update_follow(void)
{
    guided_WP_loc = HB1_follow_loc;

    prev_WP_loc = current_loc;

    // always look 100m ahead
    guided_WP_loc.offset_bearing(prev_WP_loc.get_bearing_to(HB1_follow_loc)*0.01f, 100);
    
    next_WP_loc = guided_WP_loc;
}
