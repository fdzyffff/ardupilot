#include "FD_Engine.h"
#include "FD_Engine_EP4.h"
#include "FD_Engine_LUTAN.h"

const AP_Param::GroupInfo FD_Engine::var_info[] = {

    AP_GROUPINFO("TYPE", 1, FD_Engine, engine_type, 1),

    AP_GROUPINFO("THRMIN", 2, FD_Engine, throttle_min, 25.f),

    AP_GROUPEND
};

FD_Engine::FD_Engine()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void FD_Engine::init() {
    if (engine_type == 0) {
        return;
    }
    if (engine_type == 1) {
        if (driver == nullptr) {
            driver = new FD_Engine_EP4(*this);
            gcs().send_text(MAV_SEVERITY_INFO, "Init EP4 Engine");
            driver->init();
            type = 1;
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Fail init EP4 Engine");
        }
    }
    if (engine_type == 2) {
        if (driver == nullptr) {
            driver = new FD_Engine_LUTAN(*this);
            gcs().send_text(MAV_SEVERITY_INFO, "Init LUTAN Engine");
            driver->init();
            type = 2;
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Fail init LUTAN Engine");
        }
    }
}

void FD_Engine::update() {
    if (driver != nullptr && type > 0) {
        driver->update();
    }
}

void FD_Engine::start() {
    if (driver != nullptr && type > 0) {
        driver->start();
    }
}

void FD_Engine::stop() {
    if (driver != nullptr && type > 0) {
        driver->stop();
    }
}

void FD_Engine::test_uart(uint8_t msg_id, uint8_t option)
{
    if (driver != nullptr && type > 0) {
        driver->test_uart(msg_id, option);
    }
}

bool FD_Engine::valid() {
    if (driver != nullptr && type > 0) {
        return driver->valid();
    }
    return false;
}