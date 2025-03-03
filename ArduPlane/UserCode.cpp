#include "Plane.h"

void Plane::userhook_50Hz()
{
    ufollow.update();
}

void Plane::userhook_1Hz()
{
    // ufollow.print();
    // gcs().send_message(MSG_HEARTBEAT);
    // gcs().send_message(MSG_EKF_STATUS_REPORT);
    // gcs().send_message(MSG_SYS_STATUS);
    // gcs().send_text(MAV_SEVERITY_INFO, "%d|%d|%f",plane.airspeed.enabled(), plane.airspeed.healthy(), plane.airspeed.get_airspeed());
    // plane.airspeed.print_status();
}
