#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    uattack.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    uattack.update();
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    if ((uattack.print.get() & (1<<0)) && uattack.display_info.new_data) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f , %0.0f , %0.0f , %0.0f", uattack.display_info.count_log, uattack.display_info.p1, uattack.display_info.p2, uattack.display_info.p3, uattack.display_info.p4);
        uattack.display_info.new_data = false;
    }
    if (uattack.print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "gun_angle (%0.2f , %0.2f) on:%d", uattack.get_ef_gun_info().x,uattack.get_ef_gun_info().y, uattack.is_active());
    }
    if (uattack.print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_WARNING, "cam_angle (%0.2f , %0.2f) on:%d", uattack.get_ef_cam_info().x,uattack.get_ef_cam_info().y, uattack.is_active());
    }
    if (uattack.print.get() & (1<<3)) { // 8
        gcs().send_text(MAV_SEVERITY_WARNING, "y|xyz (%0.1f , %0.1f , %0.1f , %0.1f)", uattack.get_target_angle_yaw(), uattack.get_target_vel_x(), uattack.get_target_vel_y(), uattack.get_target_vel_z());
    }
    // if (uattack.print.get() & (1<<4)) { // 16
    //     gcs().send_text(MAV_SEVERITY_WARNING, "vzpid (%0.1f , %0.1f , %0.1f , %0.2f)", uattack.get_target_roll_angle(), uattack.get_target_pitch_rate(), uattack.get_target_yaw_rate(), uattack._attack_throttle);
    // }
    // if (uattack.print.get() & (1<<5)) { // 32
    //     gcs().send_text(MAV_SEVERITY_WARNING, "apid (%0.1f , %0.1f , %0.1f , %0.2f)", uattack._attack_throttle_pid, uattack._attack_throttle_p, uattack._attack_throttle_i, uattack._attack_throttle_d);
    // }
    // if (uattack.print.get() & (1<<6)) { // 64
    //     gcs().send_text(MAV_SEVERITY_WARNING, "%0.0f , %0.0f , %0.0f , %0.0f", uattack.display_info.p11, uattack.display_info.p12, uattack.display_info.p13, uattack.display_info.p14);
    // }
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
    switch (ch_flag) {
    case RC_Channel::AuxSwitchPos::LOW:
        break;
    case RC_Channel::AuxSwitchPos::MIDDLE:
        break;
    case RC_Channel::AuxSwitchPos::HIGH:
        mode_mission.do_final_track();
        break;
    }
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
