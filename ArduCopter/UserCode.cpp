#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    sim_init();
    uattack.init();
    udelay.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    sim_update();
    uattack.update();
    udelay.push();
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
#if OSD_ENABLED == ENABLED
    osd.set_atk_angle(copter.g2.user_parameters.attack_angle.get());
#endif

}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
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
    if ((g2.user_parameters.print.get() & (1<<0)) && uattack.display_info_new) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f , %0.0f , %0.0f , %0.0f", uattack.display_info_count_log, uattack.display_info_p1, uattack.display_info_p2, uattack.display_info_p3, uattack.display_info_p4);
        uattack.display_info_new = false;
    }
    if (g2.user_parameters.print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "ef_angle (%0.2f , %0.2f) on:%d", uattack.get_ef_info().x,uattack.get_ef_info().y, uattack.is_active());
    }
    if (g2.user_parameters.print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_WARNING, "ef_rate (%0.2f , %0.2f) on:%d", uattack.get_ef_rate_info().x,uattack.get_ef_rate_info().y, uattack.is_active());
    }
    if (g2.user_parameters.print.get() & (1<<3)) { // 8
        gcs().send_text(MAV_SEVERITY_WARNING, "ar (%0.1f , %0.1f , %0.2f , %0.2f)", uattack._attack_angle_target, uattack._attack_angle_measure, uattack._attack_angle_rate_target, uattack._attack_angle_rate_measure);
    }
    if (g2.user_parameters.print.get() & (1<<4)) { // 16
        gcs().send_text(MAV_SEVERITY_WARNING, "rpyt (%0.1f , %0.1f , %0.1f , %0.2f)", uattack.get_target_roll_angle(), uattack.get_target_pitch_rate(), uattack.get_target_yaw_rate(), uattack._attack_throttle);
    }
    if (g2.user_parameters.print.get() & (1<<5)) { // 32
        gcs().send_text(MAV_SEVERITY_WARNING, "apid (%0.1f , %0.1f , %0.1f , %0.2f)", uattack._attack_throttle_pid, uattack._attack_throttle_p, uattack._attack_throttle_i, uattack._attack_throttle_d);
    }
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
    int16_t lock_size = copter.g2.user_parameters.lock_size.get();
    switch(ch_flag) {
    case RC_Channel::AuxSwitchPos::HIGH: {
        lock_size += 1;
        break;
    }
    case RC_Channel::AuxSwitchPos::LOW: {
        lock_size -= 1;
        break;
    }
    default:
        break;
    }
    lock_size = constrain_int16(lock_size, 1, 4);
    copter.g2.user_parameters.lock_size.set_and_save(lock_size);
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
    int16_t lock_y_down = copter.g2.user_parameters.lock_y_down.get();
    switch(ch_flag) {
    case RC_Channel::AuxSwitchPos::HIGH: {
        lock_y_down += 1;
        break;
    }
    case RC_Channel::AuxSwitchPos::LOW: {
        lock_y_down -= 1;
        break;
    }
    default:
        break;
    }
    lock_y_down = constrain_int16(lock_y_down, 1, 4);
    copter.g2.user_parameters.lock_y_down.set_and_save(lock_y_down);
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
    float attack_angle = copter.g2.user_parameters.attack_angle.get();
    switch(ch_flag) {
    case RC_Channel::AuxSwitchPos::HIGH: {
        attack_angle += 1;
        break;
    }
    case RC_Channel::AuxSwitchPos::LOW: {
        attack_angle -= 1;
        break;
    }
    default:
        break;
    }
    attack_angle = constrain_int16(attack_angle, 1.0f, 60.0f);
    copter.g2.user_parameters.attack_angle.set_and_save(attack_angle);
}

void Copter::userhook_auxSwitch4(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #4 handler here (CHx_OPT = 199)
    switch(ch_flag) {
    case RC_Channel::AuxSwitchPos::HIGH: {
        uattack.do_cmd_on(1);
        break;
    }
    default:
        uattack.do_cmd_on(0);
    }
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