#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    
    umav.init();
    upayload.init();
    uattack.init();
    user_gps_spd_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), 5.0f);
    g2.user_parameters.assit_pi_xy.set_dt(1.0/copter.scheduler.get_loop_rate_hz());
}
#endif

void Copter::userhook_SuperLoop()
{
#ifdef USERHOOK_FASTLOOP
    umav.send_raw_imu();
#endif
}

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    umav.update();
    upayload.update();
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
    gcs().send_message(MSG_ESTIMATOR_STATUS);
    // umav.send_status();
    umav.send_all();
    // userhook_i2c_test();

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
        gcs().send_text(MAV_SEVERITY_WARNING, "rpyt (%0.1f , %0.1f , %0.1f , %0.2f)", uattack.get_target_roll_angle(), uattack.get_target_pitch_rate(), uattack.get_target_yaw_rate(), uattack._attack_throttle);
    }
    if (uattack.print.get() & (1<<5)) { // 32
        gcs().send_text(MAV_SEVERITY_WARNING, "apid (%0.1f , %0.1f , %0.1f , %0.2f)", uattack._attack_throttle_pid, uattack._attack_throttle_p, uattack._attack_throttle_i, uattack._attack_throttle_d);
    }
    if (uattack.print.get() & (1<<6)) { // 64
        gcs().send_text(MAV_SEVERITY_WARNING, "%0.0f , %0.0f , %0.0f , %0.0f", uattack.display_info.p11, uattack.display_info.p12, uattack.display_info.p13, uattack.display_info.p14);
    }
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
        upayload.set_state(UPayload::payload_arm2);
        break;
    }

}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
    switch (ch_flag) {
    case RC_Channel::AuxSwitchPos::LOW:
        break;
    case RC_Channel::AuxSwitchPos::MIDDLE:
        break;
    case RC_Channel::AuxSwitchPos::HIGH:
        upayload.set_state(UPayload::payload_armfinal);
        break;
    }
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
    switch (ch_flag) {
    case RC_Channel::AuxSwitchPos::LOW:
        break;
    case RC_Channel::AuxSwitchPos::MIDDLE:
        break;
    case RC_Channel::AuxSwitchPos::HIGH:
        upayload.set_state(UPayload::payload_fire);
        break;
    }
}
#endif

void Copter::userhook_auxSwitch10(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
    switch (ch_flag) {
    case RC_Channel::AuxSwitchPos::LOW:
        break;
    case RC_Channel::AuxSwitchPos::MIDDLE:
        upayload.handle_destory(false, 0.f);
        break;
    case RC_Channel::AuxSwitchPos::HIGH:
        upayload.handle_destory(true, 5.f);
        break;
    }
}

// void Copter::userhook_i2c_test()
// {
//     // FOREACH_I2C_EXTERNAL(i) {
//     //     dev = std::move(hal.i2c_mgr->get_device(i, 0x29));

//     //     dev->get_semaphore()->take_blocking();

//     //     uint8_t status = 0;
//     //     uint8_t device_id = 0;
//     //     uint8_t revision_id = 0;


//     //     if (!dev) {
//     //         gcs().send_text(MAV_SEVERITY_INFO, "NO DEV VL53L5CX53L5CX");
//     //     }

//     //     gcs().send_text(MAV_SEVERITY_INFO, "BUS ADD 0x%x, 0x%x\n", dev->get_bus_address(), (uint8_t)dev->get_bus_id());


//     //     status |= write_register(0x7fff, 0x00);
//     //     status |= read_register(0, device_id);
//     //     status |= read_register(1, revision_id);
//     //     status |= write_register(0x7fff, 0x02);

//     //     if(status)
//     //     {
//     //         gcs().send_text(MAV_SEVERITY_INFO, "FIND VL53L5CX53L5CX [%x, %x]", device_id, revision_id);
//     //     } else {
//     //         gcs().send_text(MAV_SEVERITY_INFO, "[%x, %x] VL53L5CX53L5CX", device_id, revision_id);
//     //     }

//     //     dev->get_semaphore()->give();

//     //     // delete &dev;
//     // }
// }


// bool Copter::write_register(uint16_t reg, uint8_t value)
// {
//     uint8_t b[3] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), value };
//     return dev->transfer(b, 3, nullptr, 0);
// }

// bool Copter::read_register(uint16_t reg, uint8_t &value)
// {
//     uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
//     return dev->transfer(b, 2, &value, 1);
// }

// void Copter::user_count_msg(const mavlink_message_t &msg) {
//     // static uint32_t _last_print = millis();
//     // static uint32_t count = 0;
//     // if (msg.msgid == MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE) {
//     //     count++;
//     // }
//     // if (millis() - _last_print > 1000) {
//     //     float dt = MAX(0.01f, (float)(millis() - _last_print) * 0.001f);
//     //     _last_print = millis();
//     //     gcs().send_text(MAV_SEVERITY_INFO, "VISION_POSITION_ESTIMATE %0.1f/s", (float)count/dt);
//     //     count = 0;
//     // }
// }

void Copter::user_update_assit(float &target_roll, float &target_pitch)
{
    // if (is_zero(g2.user_parameters.assit_gain.get())) {return;}
    // if (!position_ok() || !motors->armed()) {
    //     return;
    // }

    // float kp = g2.user_parameters.assit_gain.get();

    // if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
    //     return;
    // }

    // Vector3f vec = gps.velocity();

    // Vector2f bf_vel = ahrs.earth_to_body2D(vec.xy());
    // float assit_max = 20.f*100.f;
    // float assit_roll = constrain_float(-bf_vel.y*100.f*kp, -assit_max, assit_max);
    // float assit_pitch = constrain_float(bf_vel.x*100.f*kp, -assit_max, assit_max);

    // if (target_roll >= 0.0f && assit_roll > 0.0f) {
    //     target_roll = MAX(target_roll, assit_roll);
    // }

    // if (target_roll <= 0.0f && assit_roll < 0.0f) {
    //     target_roll = MIN(target_roll, assit_roll);
    // }

    // if (target_pitch >= 0.0f && assit_pitch > 0.0f) {
    //     target_pitch = MAX(target_pitch, assit_pitch);
    // }

    // if (target_pitch <= 0.0f && assit_pitch < 0.0f) {
    //     target_pitch = MIN(target_pitch, -assit_max);
    // }

    Vector2f bf_angles;
    bf_angles.x = target_roll;
    bf_angles.y = target_pitch;

    if (!is_zero(target_roll) || !is_zero(target_pitch)) {
        g2.user_parameters.assit_pi_xy.reset_I();
        return;
    }

    static uint32_t last_ms = 0;
    // static uint32_t print_ms = AP_HAL::millis();
    static bool limited = false;
    uint32_t now = AP_HAL::millis();

    if (!motors->armed()) {
        return;
    }

    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        return;
    }

    if (now - last_ms > 1000) {
        g2.user_parameters.assit_pi_xy.reset_I();
        user_gps_spd_filter.reset();
        gcs().send_text(MAV_SEVERITY_INFO, "AST reset");
    }
    last_ms = now;

    Vector3f raw_vec = gps.velocity();

    // x for pitch and y for roll, same direction, rotate later
    user_gps_spd_filter.apply(Vector2f(-raw_vec.y, raw_vec.x));

    // rotate controller input to earth frame
    Vector2f input_ef = user_gps_spd_filter.get();

    // run PI controller
    g2.user_parameters.assit_pi_xy.set_input(input_ef);

    // get earth frame controller attitude in centi-degrees
    Vector2f ef_output;

    // get P term
    ef_output = g2.user_parameters.assit_pi_xy.get_p();

    Vector2f xy_I;

    // get I term
    if (limited) {
        // only allow I term to shrink in length
        xy_I = g2.user_parameters.assit_pi_xy.get_i_shrink();
    } else {
        // normal I term operation
        xy_I = g2.user_parameters.assit_pi_xy.get_pi();
    }

    ef_output += xy_I;
    ef_output *= copter.aparm.angle_max;

    // convert to body frame
    bf_angles += copter.ahrs.earth_to_body2D(ef_output);

    // set limited flag to prevent integrator windup
    limited = fabsf(bf_angles.x) > copter.aparm.angle_max || fabsf(bf_angles.y) > copter.aparm.angle_max;

    // constrain to angle limit
    bf_angles.x = constrain_float(bf_angles.x, -copter.aparm.angle_max, copter.aparm.angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -copter.aparm.angle_max, copter.aparm.angle_max);

    target_roll = bf_angles.x;
    target_pitch = bf_angles.y;
}
