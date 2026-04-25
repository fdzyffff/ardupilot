#include "Copter.h"
#include <AP_Redundancy/AP_Redundancy.h>

#if ENABLE_REDUNDANCY_CONTROL

class CopterRedundancy : public AP_Redundancy {
protected:
    uint8_t vehicle_mode_number() const override {
        return (uint8_t)copter.flightmode->mode_number();
    }
    void vehicle_set_failover_mode(uint8_t prev_ctrl_mode) override {
        (void)prev_ctrl_mode;  // Copter always fails over to LOITER
        copter.set_mode(Mode::Number::LOITER, ModeReason::SCRIPTING);
    }
    void vehicle_set_target_location(const Location &loc) override {
        copter.set_target_location(loc);
    }
    bool vehicle_arm(AP_Arming::Method method) override {
        return copter.arming.arm(method);
    }
    void vehicle_disarm(AP_Arming::Method method) override {
        copter.arming.disarm(method);
    }
    bool vehicle_is_armed() const override {
        return copter.arming.is_armed();
    }
    bool vehicle_is_flying() const override {
        return copter.arming.is_armed() && !copter.ap.land_complete;
    }
    Location vehicle_current_location() const override {
        Location loc;
        AP::ahrs().get_location(loc);
        return loc;
    }
};

static CopterRedundancy copter_redundancy;

void Copter::init_redundancy_control()
{
    redundancy = &copter_redundancy;
    g2.redundancy_ptr = &copter_redundancy;
    redundancy->init(serial_manager);
}

void Copter::update_redundancy_control()
{
    if (redundancy) {
        redundancy->update();
    }
}

#endif // ENABLE_REDUNDANCY_CONTROL
