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
        if (HB1_Power_engine_type() == 0) {
            thr_min = g2.hb1_engine60_min.get();
        }
        switch (HB1_Power.state) {
            case HB1_PowerAction_None:
            case HB1_PowerAction_RocketON:
            case HB1_PowerAction_EnginePullUP:
            case HB1_PowerAction_EngineON:
            case HB1_PowerAction_EngineOFF:
            case HB1_PowerAction_ParachuteON:
            case HB1_PowerAction_GROUND_RocketON:
                HB1_throttle = 0.0f;
                break;
            case HB1_PowerAction_GROUND_EngineSTART:
            case HB1_PowerAction_GROUND_EngineSTART_PRE:
                HB1_throttle = thr_min;
                break;
            case HB1_PowerAction_GROUND_EngineON:
                HB1_throttle = thr_min;
                break;
            case HB1_PowerAction_GROUND_EngineOFF:
                HB1_throttle = 0.0f;
                break;
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

        if (HB1_Power_engine_type() == 0) {
            thr_min = g2.hb1_engine60_min.get();
        }

        if (plane.throttle_suppressed) {
            HB1_throttle = thr_min;
        }

        switch (HB1_Power.state) {
            case HB1_PowerAction_None:
                if (HB1_Status.state == HB1_Mission_Takeoff) {
                    HB1_throttle = thr_min;
                } else {
                    HB1_throttle = throttle;
                }
                break;
            case HB1_PowerAction_RocketON:
                HB1_throttle = thr_min;
                break;
            case HB1_PowerAction_EnginePullUP:
                {   
                    float timer_delay = MAX(timer - 0.0f, 0.0f);
                    if (timer_delay < 800.f) {
                        HB1_throttle = constrain_float(35.f*timer_delay/800.f, thr_min, 35.f);
                    } else if (timer_delay < 1500.f) {
                        HB1_throttle = constrain_float(35.f + 65.f*(timer_delay-800.f)/700.f, 35.f, thr_max);
                    }
                    break;
                }
            case HB1_PowerAction_EngineON:
                HB1_throttle = constrain_float(throttle, 30.f, thr_max);
                break;
            case HB1_PowerAction_GROUND_EngineOFF:
            case HB1_PowerAction_EngineOFF:
            case HB1_PowerAction_ParachuteON:
                HB1_throttle = 0.0f;
                break;
            case HB1_PowerAction_GROUND_RocketON:
            case HB1_PowerAction_GROUND_EngineSTART:
            case HB1_PowerAction_GROUND_EngineSTART_PRE:
            case HB1_PowerAction_GROUND_EngineON:
            case HB1_PowerAction_GROUND_EngineFULL:
            case HB1_PowerAction_GROUND_EngineMID:
                HB1_throttle = thr_min;
                break;
            default:
                break;
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
            if (timer > (uint32_t)g2.hb1_rocket_timer_ms.get()) {
                float airspeed_measured = 0;
                if (!ahrs.airspeed_estimate(&airspeed_measured)) {airspeed_measured = 0.0f;}
                float gspd = ahrs.groundspeed_vector().length();
                bool speed_reached = (airspeed_measured > 10.0f || gspd > 5.0f) || (g2.hb1_test_mode != 0);
                if (speed_reached) {
//                    if (timer > 4000 || (ins.get_accel().x < 10.0f)) {
                        gcs().send_text(MAV_SEVERITY_INFO, "timer: %d ax %0.2f", (int32_t)timer, ins.get_accel().x);
                        HB1_status_set_HB_Power_Action(HB1_PowerAction_EnginePullUP);
//                    }
                } else {
                    set_mode(plane.mode_fbwa, MODE_REASON_UNAVAILABLE);
                    gcs().send_text(MAV_SEVERITY_INFO, "WARNING, low spd %0.2f, %0.2f", airspeed_measured, gspd);
                    HB1_status_set_HB_Power_Action(HB1_PowerAction_None);
                }
            }
            break;
        case HB1_PowerAction_EnginePullUP:
            if (timer > 2000) {
                HB1_status_set_HB_Power_Action(HB1_PowerAction_EngineON);
            }
            break;
        case HB1_PowerAction_EngineON:
            if (landing.is_flaring() || landing.is_on_approach()) {
                if (rangefinder_state.in_range) {
                    if (rangefinder_state.height_estimate < 1.5f) {
                        gcs().send_text(MAV_SEVERITY_INFO, "Engine protection engaged at %.2fm", (double)rangefinder_state.height_estimate);
                        HB1_status_set_HB_Power_Action(HB1_PowerAction_EngineOFF);
                    }
                }
            }
            break;
        case HB1_PowerAction_EngineOFF:
            if (timer > 3000) {
                if (arming.is_armed()) {
                    HB1_status_set_HB_Power_Action(HB1_PowerAction_None);
                } else {
                    HB1_status_set_HB_Power_Action(HB1_PowerAction_None);
                }
            }
            break;
        case HB1_PowerAction_GROUND_EngineFULL:
        case HB1_PowerAction_GROUND_EngineMID:
            // if (timer > 2000) {
            //     HB1_status_set_HB_Power_Action(HB1_PowerAction_GROUND_EngineON);
            // }
            // break;
        case HB1_PowerAction_GROUND_EngineON:
            if (plane.HB1_Power.HB1_engine_rpm.get() < 1000.f && (g2.hb1_rpm_used == 1)) {
                if (plane.HB1_Power.HB1_engine_startcount < 6) {
                    HB1_status_set_HB_Power_Action(HB1_PowerAction_GROUND_EngineSTART);
                    gcs().send_text(MAV_SEVERITY_INFO, "Restart[%d]: %0.2f", plane.HB1_Power.HB1_engine_startcount, plane.HB1_Power.HB1_engine_rpm.get());
                    plane.HB1_Power.HB1_engine_startcount += 1;
                } else {
                    gcs().send_text(MAV_SEVERITY_INFO, "Restart[%d] fail", plane.HB1_Power.HB1_engine_startcount); 
                    HB1_status_set_HB_Power_Action(HB1_PowerAction_None);   
                }
            }
            break;
        case HB1_PowerAction_ParachuteON:
            if (timer > 10000) {
                if (!arming.is_armed()) {
                    HB1_status_set_HB_Power_Action(HB1_PowerAction_None);
                }
            }
            break;
        case HB1_PowerAction_GROUND_EngineSTART_PRE:
            if (timer > 1000) {
                HB1_status_set_HB_Power_Action(HB1_PowerAction_GROUND_EngineSTART);
            }
            break;
        case HB1_PowerAction_GROUND_EngineSTART: // triggered by RC OR cmd
            if (timer > 6000) {
                HB1_status_set_HB_Power_Action(HB1_PowerAction_GROUND_EngineON);
            }
            break;
        case HB1_PowerAction_GROUND_RocketON: // triggered by RC
            if (timer > 10000) {
                HB1_status_set_HB_Power_Action(HB1_PowerAction_None);
            }
        case HB1_PowerAction_GROUND_EngineOFF: // triggered by RC OR cmd
            break;
        default:
            break;
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
    SRV_Channels::set_output_scaled(SRV_Channel::k_parachute_HB1, 0);
    switch (HB1_Power.state) {
        case HB1_PowerAction_None:
            plane.HB1_Power.HB1_engine_startcount = 0;
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
            HB1_Status.already_takeoff = true;
            SRV_Channels::set_output_scaled(SRV_Channel::k_launcher_HB1, 100);
            break;
        case HB1_PowerAction_GROUND_RocketON:
            gcs().send_text(MAV_SEVERITY_INFO, "G Rocket ON");
            relay.on(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            break;
        case HB1_PowerAction_EnginePullUP:
            gcs().send_text(MAV_SEVERITY_INFO, "Engine Pull up");
            relay.off(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            break;
        case HB1_PowerAction_GROUND_EngineSTART_PRE:
            plane.HB1_Power.HB1_engine_startcount = 0;
            gcs().send_text(MAV_SEVERITY_INFO, "Engine Prepare");
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
            break;
        case HB1_PowerAction_GROUND_EngineON:
            gcs().send_text(MAV_SEVERITY_INFO, "G Engine ON");
            relay.off(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
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
            SRV_Channels::set_output_scaled(SRV_Channel::k_parachute_HB1, 100);
            break;
        default:
            break;
    }
    HB1_msg_apm2power_set();
}

void Plane::HB1_msg_apm2power_set() {
    HB1_apm2power &tmp_msg = HB1_uart_power.get_msg_apm2power();
    tmp_msg._msg_1.need_send = false;
    switch (HB1_Power.state) {
        case HB1_PowerAction_RocketON:
        case HB1_PowerAction_GROUND_RocketON:
            tmp_msg.set_rocket_on();
            tmp_msg._msg_1.need_send = true;
            break;
        case HB1_PowerAction_GROUND_EngineSTART:
            tmp_msg.set_engine_start();
            tmp_msg._msg_1.need_send = true;
            break;
        case HB1_PowerAction_GROUND_EngineSTART_PRE:
        case HB1_PowerAction_EnginePullUP:
            tmp_msg._msg_1.need_send = false;
            break;
        case HB1_PowerAction_EngineOFF:
        case HB1_PowerAction_GROUND_EngineOFF:
        case HB1_PowerAction_ParachuteON:
            tmp_msg.set_engine_stop();
            tmp_msg._msg_1.need_send = true;
            break;
        default:
            tmp_msg._msg_1.need_send = false;
            break;
    }
    if (!tmp_msg._msg_1.need_send) {
        HB1_Power.send_counter = 0;
        return;
    }
    HB1_Power.send_counter = 1;

    // tmp_msg._msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    // tmp_msg._msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    // tmp_msg._msg_1.content.msg.sum_check = 0;
    // for (int8_t i = 0; i < tmp_msg._msg_1.length - 1; i++) {
    //     tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    // }
    tmp_msg.make_sum();
    tmp_msg._msg_1.print = true;
    return;
}

void Plane::HB1_Power_request() {
    HB1_apm2power &tmp_msg = HB1_uart_power.get_msg_apm2power();
    tmp_msg._msg_1.need_send = true;
    tmp_msg.set_request();
    tmp_msg.make_sum();
    tmp_msg._msg_1.print = false;
    return;
}

void Plane::HB1_Power_throttle_update() {
    float rpm_out = 10.f*SRV_Channels::get_output_scaled(SRV_Channel::k_throttle_HB1);
    HB1_apm2power &tmp_msg = HB1_uart_power.get_msg_apm2power();
    tmp_msg._msg_1.need_send = true;
    tmp_msg.set_throttle(rpm_out);
    tmp_msg.make_sum();
    tmp_msg._msg_1.print = false;
    return;
}

void Plane::HB1_Power_on_send() {
    // HB1_apm2power &tmp_msg = HB1_uart_power.get_msg_apm2power();
    // tmp_msg._msg_1.need_send = true;
    // tmp_msg._msg_1.content.msg.ctrl_cmd = 11;
    // tmp_msg._msg_1.content.msg.thr_value = 0;

    // tmp_msg._msg_1.content.msg.header.head_1 = HB1_apm2power::PREAMBLE1;
    // tmp_msg._msg_1.content.msg.header.head_2 = HB1_apm2power::PREAMBLE2;
    // tmp_msg._msg_1.content.msg.sum_check = 0;
    // for (int8_t i = 0; i < tmp_msg._msg_1.length - 1; i++) {
    //     tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    // }
    // tmp_msg._msg_1.print = true;
    return;
}

bool Plane::HB1_Power_running() {
    if (g2.hb1_rpm_used == 1 && plane.HB1_Power.HB1_engine_rpm.get() > 1000.f) {
        return true;
    }
    return false;
}