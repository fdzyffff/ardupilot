#include "Plane.h"

void Plane::userhook_init()
{
    uattack.init();
    ufollow.init();
    umission.init();
}

void Plane::userhook_100Hz()
{
    uattack.update();
    ufollow.update();
    umission.update();
}

void Plane::userhook_1Hz()
{
    if ((g2.user_print.get() & (1<<0)) && uattack.display_info_new) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f,%0.0f,%0.0f,%0.0f", uattack.display_info_count_log, uattack.display_info_p1, uattack.display_info_p2, uattack.display_info_p3, uattack.display_info_p4);
        uattack.display_info_new = false;
    }
    if (g2.user_print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "Corr (%0.0f,%0.0f) on:%d", uattack.get_ef_rate_info().x,uattack.get_ef_rate_info().y, uattack.is_active());
    }
    if (g2.user_print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_WARNING, "rpy (%0.1f,%0.1f,%0.1f)", uattack.get_target_roll_angle()*0.01f, uattack.get_target_pitch_rate()*0.01f, uattack.get_target_yaw_rate()*0.01f);
    }
    if (g2.user_print.get() & (1<<5)) { // 32
        ufollow.print();
    }

}
