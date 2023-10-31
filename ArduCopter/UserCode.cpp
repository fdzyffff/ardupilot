#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    // hal.serial(2)->write(0xEE);
    // hal.serial(3)->write(0xEE);
    // hal.serial(4)->write(0xEE);
    // hal.serial(5)->write(0xEE);
    // hal.serial(6)->write(0xEE);
    // hal.serial(7)->write(0xEE);
    // hal.serial(8)->write(0xEE);
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
    if (hal.serial(4)->available()) {
        gcs().send_text(MAV_SEVERITY_INFO, "ssss %x", hal.serial(4)->read());
    }
    // hal.serial(4)->write(0xEE);
    // uint8_t temp = 0;
    // temp = 0xEB;
    // hal.serial(4)->write(temp);
    // temp = 0x90;
    // hal.serial(4)->write(temp);
    // temp = 0x01;
    // hal.serial(4)->write(temp);
    // temp = 0x01;
    // hal.serial(4)->write(temp);
    // temp = 0x01;
    // hal.serial(4)->write(temp);
    // temp = 0x01;
    // hal.serial(4)->write(temp);
    // temp = 0x01;
    // hal.serial(4)->write(temp);
    // temp = 0xB1;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0xD6;
    // hal.serial(4)->write(temp);
    // temp = 0xC5;
    // hal.serial(4)->write(temp);
    // temp = 0xE8;
    // hal.serial(4)->write(temp);
    // temp = 0x42;
    // hal.serial(4)->write(temp);
    // temp = 0xA1;
    // hal.serial(4)->write(temp);
    // temp = 0x1F;
    // hal.serial(4)->write(temp);
    // temp = 0x1F;
    // hal.serial(4)->write(temp);
    // temp = 0x42;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x48;
    // hal.serial(4)->write(temp);
    // temp = 0x42;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x40;
    // hal.serial(4)->write(temp);
    // temp = 0x9C;
    // hal.serial(4)->write(temp);
    // temp = 0x45;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);
    // temp = 0x00;
    // hal.serial(4)->write(temp);

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
