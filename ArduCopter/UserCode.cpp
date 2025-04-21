#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    
    umav.init();
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
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
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

// void Copter::userhook_i2c_test()
// {
//     FOREACH_I2C_EXTERNAL(i) {
//         dev = std::move(hal.i2c_mgr->get_device(i, 0x52));

//         dev->get_semaphore()->take_blocking();

//         uint8_t status = 0;
//         uint8_t device_id = 0;
//         uint8_t revision_id = 0;


//         if (!dev) {
//             gcs().send_text(MAV_SEVERITY_INFO, "NO DEV VL53L5CX53L5CX");
//         }

//         gcs().send_text(MAV_SEVERITY_INFO, "BUS ADD 0x%x, 0x%x\n", dev->get_bus_address(), (uint8_t)dev->get_bus_id());


//         status |= write_register(0x7fff, 0x00);
//         // status |= read_register(0, device_id);
//         // status |= read_register(1, revision_id);
//         // status |= write_register(0x7fff, 0x02);

//         if(status)
//         {
//             gcs().send_text(MAV_SEVERITY_INFO, "FIND VL53L5CX53L5CX");
//         } else {
//             gcs().send_text(MAV_SEVERITY_INFO, "[%x, %x] VL53L5CX53L5CX", device_id, revision_id);
//         }

//         dev->get_semaphore()->give();

//         // delete &dev;
//     }
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