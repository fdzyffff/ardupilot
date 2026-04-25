#include "Plane.h"
#include <AP_Redundancy/AP_Redundancy.h>

#if ENABLE_REDUNDANCY_CONTROL

class PlaneRedundancy : public AP_Redundancy {
protected:
    uint8_t vehicle_mode_number() const override {
        return plane.control_mode->mode_number();
    }
    void vehicle_set_failover_mode(uint8_t prev_ctrl_mode) override {
        // 如果之前的控制余度处于MANUAL模式，则新控制余度也保持MANUAL
        if (prev_ctrl_mode == (uint8_t)Mode::Number::MANUAL) {
            plane.set_mode(Mode::Number::MANUAL, ModeReason::SCRIPTING);
            return;
        }
        // 其他模式切换为LOITER
        plane.set_mode(Mode::Number::LOITER, ModeReason::SCRIPTING);
    }
    void vehicle_set_target_location(const Location &loc) override {
        plane.set_target_location(loc);
    }
    bool vehicle_arm(AP_Arming::Method method) override {
        return plane.arming.arm(method);
    }
    void vehicle_disarm(AP_Arming::Method method) override {
        plane.arming.disarm(method);
    }
    bool vehicle_is_armed() const override {
        return plane.arming.is_armed();
    }
    bool vehicle_is_flying() const override {
        return plane.is_flying();
    }
    Location vehicle_current_location() const override {
        Location loc;
        AP::ahrs().get_location(loc);
        return loc;
    }
};

static PlaneRedundancy plane_redundancy;

void Plane::init_redundancy_control()
{
    redundancy = &plane_redundancy;
    g2.redundancy_ptr = &plane_redundancy;
    redundancy->init(serial_manager);
}

void Plane::update_redundancy_control()
{
    if (redundancy) {
        redundancy->update();
    }
}

#endif // ENABLE_REDUNDANCY_CONTROL
