#include "Plane.h"

void Plane::HB1_Power_update() {
    HB1_Power_pwm_update();
    HB1_Power_status_update();
}

void Plane::HB1_Power_pwm_update() {
    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    float HB1_throttle = 0.0f;
    float thr_min = 0.0f;
    float thr_max = 100.0f;
    if (!hal.util->get_soft_armed()) {
        HB1_throttle = thr_min;
        HB1_status_set_HB_Power_Action(HB1_PoserAction_None);
    } else {
        HB1_throttle = throttle;

        if (!plane.throttle_suppressed) {
            if ( (HB1_Power.state == HB1_PoserAction_None) && (control_mode == &mode_takeoff) ) {
                HB1_status_set_HB_Power_Action(HB1_PoserAction_RocketON);
            }
        } else {
            HB1_throttle = thr_min;
        }
        
        if ( plane.parachute.released() && HB1_Power.state != HB1_PoserAction_ParachuteON) {
            HB1_status_set_HB_Power_Action(HB1_PoserAction_EngineOFF);
        }

        if (HB1_Power.state == HB1_PoserAction_RocketON) {
            float timer = millis() - HB1_Power.timer;
            const float tau = 2000.f;
            HB1_throttle = constrain_float(20.f + 80.f*timer/tau, thr_min, thr_max);
        }
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle_HB1, HB1_throttle);

}

void Plane::HB1_Power_status_update() {
    uint32_t timer = millis() - HB1_Power.timer;
    switch (HB1_Power.state) {
        case HB1_PoserAction_None:
            break;
        case HB1_PoserAction_RocketON:
            if (timer > 2000) {
                HB1_status_set_HB_Power_Action(HB1_PoserAction_EngineSTART);
            }
            break;
        case HB1_PoserAction_EngineSTART:
            if (timer > 5000) {
                HB1_status_set_HB_Power_Action(HB1_PoserAction_EngineON);
            }
            break;
        case HB1_PoserAction_EngineON:
            break;
        case HB1_PoserAction_EngineOFF:
            if (timer > 3000) {
                HB1_status_set_HB_Power_Action(HB1_PoserAction_ParachuteON);
            }
            break;
        case HB1_PoserAction_ParachuteON:
            break;
        default:
            break;
    }
}

void Plane::HB1_status_set_HB_Power_Action(HB1_Power_Action_t action) {
    if (HB1_Power.state == action) {
        return;
    } else {
        HB1_Power.state = action;
    }
    uint32_t tnow = millis();
    HB1_Power.timer = tnow;
    switch (HB1_Power.state) {
        case HB1_PoserAction_None:
            gcs().send_text(MAV_SEVERITY_INFO, "Power None");
            relay.off(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            break;
        case HB1_PoserAction_RocketON:
            gcs().send_text(MAV_SEVERITY_INFO, "Rocket ON");
            relay.on(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            break;
        case HB1_PoserAction_EngineSTART:
            gcs().send_text(MAV_SEVERITY_INFO, "Engine Starting");
            relay.off(0);
            relay.off(1);
            relay.off(2);
            relay.on(3);
            break;
        case HB1_PoserAction_EngineON:
            gcs().send_text(MAV_SEVERITY_INFO, "Engine ON");
            relay.off(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            break;
        case HB1_PoserAction_EngineOFF:
            gcs().send_text(MAV_SEVERITY_INFO, "Engine OFF");
            relay.off(0);
            relay.off(1);
            relay.on(2);
            relay.off(3);
            break;
        case HB1_PoserAction_ParachuteON:
            gcs().send_text(MAV_SEVERITY_INFO, "Parachute ON");
            relay.off(0);
            relay.on(1);
            relay.off(2);
            relay.off(3);
            break;
        default:
            break;
    }
}
