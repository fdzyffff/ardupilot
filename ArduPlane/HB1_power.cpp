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
    float timer = (millis() - HB1_Power.timer);
    if (!arming.is_armed()) {
        HB1_throttle = thr_min;
        switch (HB1_Power.state) {
            case HB1_PowerAction_GROUND_EngineFULL:
                if (timer < 2000.f) {
                    HB1_throttle = constrain_float(100.f*timer/2000.f, thr_min, thr_max);
                } else if (timer < 20000.f) {
                    HB1_throttle = constrain_float(100.f, thr_min, thr_max);
                } else {
                    HB1_throttle = thr_min;
                }
                break;
            case HB1_PowerAction_GROUND_EngineMID:
                HB1_throttle = constrain_float(30.f, thr_min, thr_max);
                break;
            default:
                break;
        }
    } else {
        HB1_throttle = throttle;

        if (plane.throttle_suppressed) {
            HB1_throttle = thr_min;
        }

        if ( (HB1_Power.state == HB1_PowerAction_RocketON) ) {
            HB1_throttle = thr_min;
        }

        if ( (HB1_Status.state == HB1_Mission_Takeoff) && (HB1_Power.state == HB1_PowerAction_None) ) {
            HB1_throttle = thr_min;
        }

        if ( (HB1_Power.state == HB1_PowerAction_EnginePrepare) ) {
            HB1_throttle = constrain_float(10.f, thr_min, thr_max);
        }

        if (HB1_Power.state == HB1_PowerAction_EngineSTART) {
            float timer_delay = MAX(timer - 0.0f, 0.0f);
            if (timer_delay < 800.f) {
                HB1_throttle = constrain_float(35.f*timer_delay/800.f, thr_min, 35.f);
            } else if (timer_delay < 1500.f) {
                HB1_throttle = constrain_float(35.f + 65.f*(timer_delay-800.f)/700.f, 35.f, thr_max);
            }
        }
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle_HB1, HB1_throttle);
}

void Plane::HB1_Power_status_update() {
    uint32_t timer = millis() - HB1_Power.timer;
    switch (HB1_Power.state) {
        case HB1_PowerAction_None:
            if (arming.is_armed() && !plane.throttle_suppressed && (control_mode == &mode_takeoff)) {
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
//                    if (timer > 4000 || (ins.get_accel().x < 10.0f)) {
                        gcs().send_text(MAV_SEVERITY_INFO, "timer: %d ax %0.2f", (int32_t)timer, ins.get_accel().x);
                        if (plane.HB1_Power.HB1_engine_rpm < 1500.f && (g2.hb1_rpm_used == 1)) {
                            HB1_status_set_HB_Power_Action(HB1_PowerAction_EnginePrepare);
                        } else {
                            HB1_status_set_HB_Power_Action(HB1_PowerAction_EngineSTART);
                        }
//                    }
                } else {
                    set_mode(plane.mode_fbwa, MODE_REASON_UNAVAILABLE);
                    gcs().send_text(MAV_SEVERITY_INFO, "WARNING, low spd %0.2f, %0.2f", airspeed_measured, gspd);
                    HB1_status_set_HB_Power_Action(HB1_PowerAction_None);
                }
            }
            break;
        case HB1_PowerAction_EnginePrepare:
            if (timer > 2000) {
                HB1_status_set_HB_Power_Action(HB1_PowerAction_EngineSTART);
            }
            break;
        case HB1_PowerAction_EngineSTART:
            if (timer > 2000) {
                HB1_status_set_HB_Power_Action(HB1_PowerAction_EngineON);
            }
            break;
        case HB1_PowerAction_EngineON:
            break;
        case HB1_PowerAction_EngineOFF:
            if (timer > 3000) {
                if (arming.is_armed()) {
                    HB1_status_set_HB_Power_Action(HB1_PowerAction_ParachuteON);
                } else {
                    HB1_status_set_HB_Power_Action(HB1_PowerAction_None);
                }
            }
            break;
        case HB1_PowerAction_GROUND_EngineFULL:
            break;
        case HB1_PowerAction_ParachuteON:
            break;
        case HB1_PowerAction_GROUND_EngineSTART: // triggered by RC OR cmd
            if (timer > 1000) {
                HB1_status_set_HB_Power_Action(HB1_PowerAction_None);
            }
            break;
        case HB1_PowerAction_GROUND_RocketON: // triggered by RC
        case HB1_PowerAction_GROUND_EngineOFF: // triggered by RC OR cmd
            break;
        default:
            break;
    }
        
    if ( plane.parachute.released() && HB1_Power.state != HB1_PowerAction_ParachuteON) {
        HB1_status_set_HB_Power_Action(HB1_PowerAction_EngineOFF);
    }
}

void Plane::HB1_status_set_HB_Power_Action(HB1_Power_Action_t action, bool Force_set) {
    if ((HB1_Power.state == action) && !Force_set) {
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
            SRV_Channels::set_output_pwm(SRV_Channel::k_launcher_HB1, 1100);
            break;
        case HB1_PowerAction_RocketON:
            gcs().send_text(MAV_SEVERITY_INFO, "Rocket ON");
            relay.on(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            HB1_Status.already_takeoff = true;
            SRV_Channels::set_output_pwm(SRV_Channel::k_launcher_HB1, 1900);
            break;
        case HB1_PowerAction_GROUND_RocketON:
            gcs().send_text(MAV_SEVERITY_INFO, "G Rocket ON");
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
            relay.off(3);
            break;
        case HB1_PowerAction_GROUND_EngineSTART:
            gcs().send_text(MAV_SEVERITY_INFO, "G Engine Starting");
            relay.off(0);
            relay.off(1);
            relay.off(2);
            relay.on(3);
        case HB1_PowerAction_EnginePrepare:
            gcs().send_text(MAV_SEVERITY_INFO, "R Engine Starting");
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
        case HB1_PowerAction_GROUND_EngineOFF:
            gcs().send_text(MAV_SEVERITY_INFO, "G Engine OFF");
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
    HB1_Power.send_counter = 3;
    HB1_msg_apm2power_send();
}

void Plane::HB1_Power_throttle_update() {
    float throttle_out = 10.f*SRV_Channels::get_output_scaled(SRV_Channel::k_throttle_HB1);
    HB1_apm2power &tmp_msg = HB1_uart_power.get_msg_apm2power();
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.content.msg.ctrl_cmd = 0;
    tmp_msg._msg_1.content.msg.thr_value = constrain_int16((int16_t)throttle_out, 0, 999);

    tmp_msg._msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    tmp_msg._msg_1.content.msg.sum_check = 0;
    for (int8_t i = 0; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
    tmp_msg._msg_1.print = false;
    return;
}

void Plane::HB1_Power_on_send() {
    HB1_apm2power &tmp_msg = HB1_uart_power.get_msg_apm2power();
    tmp_msg._msg_1.need_send = true;
    tmp_msg._msg_1.content.msg.ctrl_cmd = 11;
    tmp_msg._msg_1.content.msg.thr_value = 0;

    tmp_msg._msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    tmp_msg._msg_1.content.msg.sum_check = 0;
    for (int8_t i = 0; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
    tmp_msg._msg_1.print = true;
    return;
}
