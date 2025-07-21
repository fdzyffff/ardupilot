#include "Plane.h"

void Plane::userhook_init()
{
    uart.init();
    uattack.init();
    udelay.init();
}

void Plane::userhook_100Hz()
{
    uattack.update();
    udelay.push();
}

void Plane::userhook_1Hz()
{
    AP::fd_data().set_is_flying(is_flying());
    AP::fd_data().update();
    // ufollow.update();

    // put your 1Hz code here
    if ((uattack.print.get() & (1<<0)) && uattack.display_info.new_data) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f , %0.0f , %0.0f , %0.0f", uattack.display_info.count_log, uattack.display_info.p1, uattack.display_info.p2, uattack.display_info.p3, uattack.display_info.p4);
        uattack.display_info.new_data = false;
    }
    if (uattack.print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "ef_angle (%0.2f , %0.2f) on:%d", uattack.get_ef_info().x,uattack.get_ef_info().y, uattack.is_active());
    }
    if (uattack.print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_WARNING, "ef_rate (%0.2f , %0.2f) on:%d", uattack.get_ef_rate_info().x,uattack.get_ef_rate_info().y, uattack.is_active());
    }
    if (uattack.print.get() & (1<<3)) { // 8
        gcs().send_text(MAV_SEVERITY_WARNING, "ar (%0.1f , %0.1f , %0.2f , %0.2f)", uattack._attack_angle_target, uattack._attack_angle_measure, uattack._attack_angle_rate_target, uattack._attack_angle_rate_measure);
    }
    if (uattack.print.get() & (1<<4)) { // 16
        gcs().send_text(MAV_SEVERITY_WARNING, "rpyt (%0.1f , %0.1f , %0.1f , %0.2f)", uattack.get_target_roll_angle(), uattack.get_target_pitch_rate(), uattack.get_target_yaw_rate(), uattack.attack_throttle.get());
    }
    // if (uattack.print.get() & (1<<5)) { // 32
    //     gcs().send_text(MAV_SEVERITY_WARNING, "apid (%0.1f , %0.1f , %0.1f , %0.2f)", uattack._attack_throttle_pid, uattack._attack_throttle_p, uattack._attack_throttle_i, uattack._attack_throttle_d);
    // }
    if (uattack.print.get() & (1<<6)) { // 364
        gcs().send_text(MAV_SEVERITY_WARNING, "%0.0f , %0.0f , %0.0f , %0.0f", uattack.display_info.p11, uattack.display_info.p12, uattack.display_info.p13, uattack.display_info.p14);
    }
}
