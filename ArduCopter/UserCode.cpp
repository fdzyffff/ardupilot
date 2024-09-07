#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    useruartfwd.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    useruartfwd.update();
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
void Copter::userhook_25Hz()
{
    // put your 25Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    ugimbal.update();
#if LOGGING_ENABLED == ENABLED
    if (logger.logging_started()) {
        Log_Write_Uatk();
    }
#endif
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
    if (ugimbal.display_info.count > 0) {
        if ((g2.user_parameters.cam_print.get() & (1<<0)) ) { // 1
            gcs().send_text(MAV_SEVERITY_WARNING, "1[%d] %0.0f,%0.0f,%0.0f,%0.0f", ugimbal.display_info.count, ugimbal.display_info.p1, ugimbal.display_info.p2, ugimbal.display_info.p3, ugimbal.display_info.p4);
        }
        if ((g2.user_parameters.cam_print.get() & (1<<1)) ) { // 2
            gcs().send_text(MAV_SEVERITY_WARNING, "2[%d] %0.0f,%0.0f,%0.0f,%0.0f", ugimbal.display_info.count, ugimbal.display_info.p11, ugimbal.display_info.p12, ugimbal.display_info.p13, ugimbal.display_info.p14);
        }
        ugimbal.display_info.count = 0;
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "hal.util->available_memory:%ld",hal.util->available_memory());



    // Vector3f dist_vec;  // vector to lead vehicle
    // Vector3f dist_vec_offs;  // vector to lead vehicle + offset
    // Vector3f vel_of_target;  // velocity of lead vehicle
    // if (g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target)) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "dist_vec.x | y : %f | %f", dist_vec.x, dist_vec.y);
    //     gcs().send_text(MAV_SEVERITY_INFO, "dist_vec_offs.x | y : %f | %f", dist_vec_offs.x, dist_vec_offs.y);
    //     gcs().send_text(MAV_SEVERITY_INFO, "vel_of_target.x | y : %f | %f", vel_of_target.x, vel_of_target.y);
    // }
    // else {
    //     gcs().send_text(MAV_SEVERITY_INFO, "False");
    // }
    // gcs().send_text(MAV_SEVERITY_INFO, "%f", pos_control->get_max_speed_xy_cms());

}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
    // ufollow.set_target_loc();
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

void Copter::print_target_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_POD_MEAS) {
        // decode packet
        mavlink_pod_meas_t pod_meas;
        mavlink_msg_pod_meas_decode(&msg, &pod_meas);
        gcs().send_text(MAV_SEVERITY_INFO, "pod_meast %f | %f", (float)pod_meas.tgt_lat, (float)pod_meas.tgt_lon);
        gcs().send_text(MAV_SEVERITY_INFO, "pod_measm %f | %f", (float)pod_meas.mother_lat, (float)pod_meas.mother_lon);
    }
}