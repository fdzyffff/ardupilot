#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    
    umav.init();
    upayload.init();
    uattack.init();
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
