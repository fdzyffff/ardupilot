#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    udrop.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    FD1_uart_msg_ue4.init();
    FD1_uart_msg_ue4.get_msg_ue4_ahrs().set_enable();
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    udrop.update();
    // put your 50Hz code here
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SIM* _sitl = AP::sitl();
    if (_sitl == nullptr) {
        return;
    }
    if (_sitl) {
        const struct SITL::sitl_fdm &fdm = _sitl->state;

        FD1_msg_ue4_ahrs &tmp_msg = FD1_uart_msg_ue4.get_msg_ue4_ahrs();
        tmp_msg._msg_1.content.msg.header.head_1 = FD1_msg_ue4_ahrs::PREAMBLE1;
        tmp_msg._msg_1.content.msg.header.head_2 = FD1_msg_ue4_ahrs::PREAMBLE2;
        tmp_msg._msg_1.content.msg.vehicle_id = 0;
        tmp_msg._msg_1.content.msg.lat = fdm.latitude * 1e7;
        tmp_msg._msg_1.content.msg.lng = fdm.longitude * 1e7;
        tmp_msg._msg_1.content.msg.alt = fdm.altitude*100;
        tmp_msg._msg_1.content.msg.roll = fdm.rollDeg;
        tmp_msg._msg_1.content.msg.pitch = fdm.pitchDeg;
        tmp_msg._msg_1.content.msg.yaw = fdm.yawDeg;
        tmp_msg._msg_1.content.msg.sum_check = 0;

        for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
            tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
        }
        tmp_msg._msg_1.need_send = true;
    }

    FD1_uart_msg_ue4.write();
// #elif
//     if (position_ok()){
//         int32_t alt_absolute;
//         if (!current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_absolute)) {
//             alt_absolute = 0;
//         }

//         FD1_msg_ue4_ahrs &tmp_msg = FD1_uart_msg_ue4.get_msg_ue4_ahrs();
//         tmp_msg._msg_1.content.msg.header.head_1 = FD1_msg_ue4_ahrs::PREAMBLE1;
//         tmp_msg._msg_1.content.msg.header.head_2 = FD1_msg_ue4_ahrs::PREAMBLE2;
//         tmp_msg._msg_1.content.msg.vehicle_id = 0;
//         tmp_msg._msg_1.content.msg.lat = current_loc.lat;
//         tmp_msg._msg_1.content.msg.lng = current_loc.lng;
//         tmp_msg._msg_1.content.msg.alt = alt_absolute;
//         tmp_msg._msg_1.content.msg.roll = degrees(ahrs_view->roll);
//         tmp_msg._msg_1.content.msg.pitch = degrees(ahrs_view->pitch);
//         tmp_msg._msg_1.content.msg.yaw = degrees(ahrs_view->yaw);
//         tmp_msg._msg_1.content.msg.sum_check = 0;

//         for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
//             tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
//         }
//         tmp_msg._msg_1.need_send = true;
//     }
//     FD1_uart_msg_ue4.write();
#endif
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
