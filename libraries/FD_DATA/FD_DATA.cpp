#include <AP_Math/AP_Math.h>
#include "FD_DATA.h"

extern const AP_HAL::HAL& hal;

/*
 * init - perform required initialisation
 */

FD_DATA *FD_DATA::_singleton;

// constructor
FD_DATA::FD_DATA()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("FD_DATA must be singleton");
    }
    _singleton = this;
}

void FD_DATA::send_mav_tof_matrix(mavlink_wxbs_tof_distance_t *packet)
{
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    // mavlink_command_int_t command_int;
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
        if (mask & (1U<<i)) {
            if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 99) {
                mavlink_msg_wxbs_tof_distance_send_struct(
                    channel,
                    packet);
            }
        }
    }
}

namespace AP {

FD_DATA &fd_data()
{
    return *FD_DATA::get_singleton();
}

};
