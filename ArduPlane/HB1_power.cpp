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
        //HB1_status_set_HB_Power_Action(HB1_PowerAction_None);
    } else {
        HB1_throttle = throttle;

        if (plane.throttle_suppressed) {
            HB1_throttle = thr_min;
        }

        if (HB1_Power.state == HB1_PowerAction_RocketON) {
            HB1_throttle = thr_min;
        }
        
        if (HB1_Power.state == HB1_PowerAction_EngineSTART) {
            float timer = millis() - HB1_Power.timer;
            if (timer < 1200.f) {
                HB1_throttle = constrain_float(35.f*timer/1200.f, thr_min, 35.f);
            } else if (timer < 2000.f) {
                HB1_throttle = constrain_float(35.f + 65.f*(timer-1200.f)/800.f, 35.f, thr_max);
            } else {
                HB1_throttle = constrain_float(100.f, thr_min, thr_max);
            }
        }
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle_HB1, HB1_throttle);

}

void Plane::HB1_Power_status_update() {
    uint32_t timer = millis() - HB1_Power.timer;
    switch (HB1_Power.state) {
        case HB1_PowerAction_None:
        if (hal.util->get_soft_armed() && !plane.throttle_suppressed && (control_mode == &mode_takeoff)) {
            HB1_status_set_HB_Power_Action(HB1_PowerAction_RocketON);
        }
        break;
        case HB1_PowerAction_RocketON:
            if (timer > 2000) {
                float airspeed_measured = 0;
                if (!ahrs.airspeed_estimate(&airspeed_measured)) {airspeed_measured = 0.0f;}
                float gspd = ahrs.groundspeed_vector().length();
                bool speed_reached = (airspeed_measured > 10.0f || gspd > 5.0f);
                if (speed_reached || (g2.hb1_test_mode != 0)) {
                    HB1_status_set_HB_Power_Action(HB1_PowerAction_EngineSTART);
                } else {
                    set_mode(plane.mode_fbwa, MODE_REASON_UNAVAILABLE);
                    HB1_status_set_HB_Power_Action(HB1_PowerAction_None);
                }
            }
            break;
        case HB1_PowerAction_EngineSTART:
            if (timer > 5000) {
                HB1_status_set_HB_Power_Action(HB1_PowerAction_EngineON);
            }
            break;
        case HB1_PowerAction_EngineON:
            break;
        case HB1_PowerAction_EngineOFF:
            if (timer > 3000) {
                if (hal.util->get_soft_armed()) {
                    HB1_status_set_HB_Power_Action(HB1_PowerAction_ParachuteON);
                } else {
                    HB1_status_set_HB_Power_Action(HB1_PowerAction_None);
                }
            }
            break;
        case HB1_PowerAction_ParachuteON:
            break;
        default:
            break;
    }
        
    if ( plane.parachute.released() && HB1_Power.state != HB1_PowerAction_ParachuteON) {
        HB1_status_set_HB_Power_Action(HB1_PowerAction_EngineOFF);
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
        case HB1_PowerAction_None:
            gcs().send_text(MAV_SEVERITY_INFO, "Power None");
            relay.off(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            break;
        case HB1_PowerAction_RocketON:
            gcs().send_text(MAV_SEVERITY_INFO, "Rocket ON");
            relay.on(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            break;
        case HB1_PowerAction_EngineSTART:
            gcs().send_text(MAV_SEVERITY_INFO, "Engine Starting");
            relay.off(0);
            relay.off(1);
            relay.off(2);
            relay.on(3);
            break;
        case HB1_PowerAction_EngineON:
            gcs().send_text(MAV_SEVERITY_INFO, "Engine ON");
            relay.off(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            break;
        case HB1_PowerAction_EngineOFF:
            gcs().send_text(MAV_SEVERITY_INFO, "Engine OFF");
            relay.off(0);
            relay.off(1);
            relay.on(2);
            relay.off(3);
            break;
        case HB1_PowerAction_ParachuteON:
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
