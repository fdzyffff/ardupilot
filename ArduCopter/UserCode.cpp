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
    const RangeFinder *rnf = RangeFinder::get_singleton();

    bool rngfnd_good_1 = false;
    bool rngfnd_good_2 = false;
    float rngfnd_dist = -1.0f;
    if (rnf != nullptr) {
        rngfnd_good_1 = (rnf->status_orient(ROTATION_NONE) == RangeFinder::Status::Good);
        rngfnd_good_2 = (rnf->range_valid_count_orient(ROTATION_NONE) >= 3);
        float tilt_correction = sinf(fabsf(AP::ahrs().get_pitch()));
        rngfnd_dist = tilt_correction * rnf->distance_cm_orient(ROTATION_NONE);
    }

    // put your 1Hz code here
    if ((uattack.print.get() & (1<<0)) && uattack.display_info.new_data) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "[%d] %0.0f , %0.0f , %0.0f , %0.0f", uattack.display_info.count_log, uattack.display_info.p1, uattack.display_info.p2, uattack.display_info.p3, uattack.display_info.p4);
        uattack.display_info.new_data = false;
    }
    if (uattack.print.get() & (1<<1)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "bf_angle (%0.2f , %0.2f) on:%d", uattack.get_bf_info().x,uattack.get_bf_info().y, uattack.is_active());
    }
    if (uattack.print.get() & (1<<2)) { // 4
        gcs().send_text(MAV_SEVERITY_INFO, "DIST: G1 %d, G2 %d D %f", rngfnd_good_1, rngfnd_good_2, rngfnd_dist);
    }
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
