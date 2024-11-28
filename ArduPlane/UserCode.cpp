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
    // gcs().send_message(MSG_USER_1);
    send_user_1();
}

void Plane::send_user_1()
{
    nav_filter_status filt_status;
    ahrs.get_filter_status(filt_status);
    if (!filt_status.flags.attitude || !filt_status.flags.horiz_pos_abs || !filt_status.flags.vert_pos) {
        return;
    }
    int32_t lat = g2.target_loc_lat.get()*1e7;
    int32_t lng = g2.target_loc_lng.get()*1e7;
    int32_t alt = g2.target_loc_alt.get()*100;
    Location tmp_loc = Location(lat, lng, alt, Location::AltFrame::ABSOLUTE);
    float bearing = wrap_PI(current_loc.get_bearing(tmp_loc) - ahrs.get_yaw());

    // gcs().send_text(MAV_SEVERITY_INFO, "Send USER_1");
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 127) {
                mavlink_msg_command_long_send(
                                        channel, 
                                        0, 
                                        0, 
                                        MAV_CMD_USER_1, 
                                        0, 
                                        bearing, 
                                        0.0f, 
                                        0.0f, 
                                        0.0f, 
                                        0.0f, 
                                        0.0f, 
                                        0.0f);
            }
        }
    }
}
