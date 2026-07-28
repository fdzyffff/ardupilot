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

#define USER_FORCE_SAFE_FENCE_LAT 401570981
#define USER_FORCE_SAFE_FENCE_LNG 1164080429
#define USER_FORCE_SAFE_RADIUS_M 100000.0f
#define USER_FORCE_SAFE_REARM_M 101000.0f
#define USER_FORCE_SAFE_DISARM_M 95000.0f
#define USER_FORCE_SAFE_OUTSIDE_DISARM_M 5000.0f

struct UserForceSafeArea {
    int32_t lat;
    int32_t lng;
    float radius_m;
};

const UserForceSafeArea user_force_safe_areas[] = {
    {  322500000,  830000000, 490000.0f }, // 1  Tibet Ali
    {  455000000, 1300000000, 480000.0f }, // 2  Eastern Heilongjiang
    {  222500000, 1112500000, 530000.0f }, // 3  Western Guangdong and Guangxi coast
    {  385000000, 1045000000, 510000.0f }, // 4  Ningxia and central Gansu
    {  452500000,  850000000, 480000.0f }, // 5  Northern Xinjiang
    {  282500000,  995000000, 500000.0f }, // 6  Western Sichuan and northwestern Yunnan
    {  337500000, 1177500000, 510000.0f }, // 7  Jiangsu, Anhui and Huang-Huai
    {  445000000, 1142500000, 480000.0f }, // 8  Central Inner Mongolia
    {  357500000,  950000000, 480000.0f }, // 9  Southern Qinghai
    {  310000000, 1092500000, 520000.0f }, // 10 Chongqing, Hubei and Three Gorges
    {  387500000,  785000000, 480000.0f }, // 11 Western southern Xinjiang
    {  280000000, 1172500000, 520000.0f }, // 12 Jiangxi, Fujian and southern Zhejiang
    {  307500000,  900000000, 490000.0f }, // 13 Central Tibet
    {  245000000, 1037500000, 530000.0f }, // 14 Central and southern Yunnan
    {  412500000, 1217500000, 480000.0f }, // 15 Liaoning
    {  327500000, 1000000000, 490000.0f }, // 16 Southeastern Qinghai and northern Sichuan
    {  392500000, 1120000000, 510000.0f }, // 17 Shanxi and northern Shaanxi
    {  425000000,  955000000, 500000.0f }, // 18 Hexi Corridor
    {  495000000, 1220000000, 470000.0f }, // 19 Northeastern Inner Mongolia and western Heilongjiang
    {  385000000,  870000000, 470000.0f }, // 20 Central Xinjiang
    { -353627273, 1491651642, 100000.0f }, // 21 Australia SITL
};

void Copter::check_forced_land_or_rtl()
{
    if (!arming.is_armed()) {
        return;
    }

    Location cur = current_loc;
    if (cur.lat == 0 && cur.lng == 0) {
        if (AP::gps().status() < AP_GPS::GPS_OK_FIX_2D) {
            return;
        }
        cur = AP::gps().location();
    }

    const Location fence_loc(USER_FORCE_SAFE_FENCE_LAT,
                             USER_FORCE_SAFE_FENCE_LNG,
                             0,
                             Location::AltFrame::ABOVE_ORIGIN);
    const float fence_dist_m = cur.get_distance(fence_loc);

    if (fence_dist_m > USER_FORCE_SAFE_REARM_M) {
        force_safe_triggered = false;
    } else if (fence_dist_m > USER_FORCE_SAFE_RADIUS_M) {
        if (!force_safe_triggered) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Force RTL for Safety Reason");
            set_mode(Mode::Number::RTL, ModeReason::USER_FORCE_SAFE);
            force_safe_triggered = true;
        }
        return;
    } else if (fence_dist_m > USER_FORCE_SAFE_DISARM_M) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Force Land for Safety Reason");
        set_mode(Mode::Number::LAND, ModeReason::USER_FORCE_SAFE);
        return;
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Force Disarm for Safety Reason");
        arming.disarm(AP_Arming::Method::TERMINATION);
        return;
    }

    float minimum_outside_dist_m = 0.0f;
    bool minimum_outside_dist_valid = false;

    for (uint8_t i = 0; i < ARRAY_SIZE(user_force_safe_areas); i++) {
        const UserForceSafeArea &area = user_force_safe_areas[i];
        Location area_center = cur;
        area_center.lat = area.lat;
        area_center.lng = area.lng;
        const float outside_dist_m = cur.get_distance(area_center) - area.radius_m;

        if (outside_dist_m <= 0.0f) {
            return;
        }
        if (!minimum_outside_dist_valid || outside_dist_m < minimum_outside_dist_m) {
            minimum_outside_dist_m = outside_dist_m;
            minimum_outside_dist_valid = true;
        }
    }

    if (minimum_outside_dist_m <= USER_FORCE_SAFE_OUTSIDE_DISARM_M) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Force RTL Outside Allowed Area");
        set_mode(Mode::Number::RTL, ModeReason::USER_FORCE_SAFE);
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Force Disarm Outside Allowed Area");
        arming.disarm(AP_Arming::Method::TERMINATION);
    }
}
