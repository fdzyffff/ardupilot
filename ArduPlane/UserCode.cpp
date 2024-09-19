#include "Plane.h"

void Plane::userhook_50Hz()
{
    ufollow.update();
}

void Plane::userhook_1Hz()
{
    // ufollow.print();
    gcs().send_message(MSG_HEARTBEAT);
    gcs().send_message(MSG_EKF_STATUS_REPORT);
    gcs().send_message(MSG_SYS_STATUS);
}
