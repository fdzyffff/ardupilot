#include "Plane.h"

UPower::UPower()
{
    ;
}

void UPower::init()
{
    uart_power.init();
    uart_power.get_msg_power2apm().set_enable();
    engine_rpm.reset(0.0f);
    engine_rpm.set_cutoff_frequency(50.f, 5.f);
    engine_temp = 0.0f;
    engine_fuel = 0.0f;
    engine_status = 0;
    engine_startcount = 0;
    send_counter = 0;
}

// need 50Hz
void UPower::update()
{
    pwm_update();
    state_update();
    uart_update();
}

void UPower::pwm_update()
{
    static float HB1_throttle_out = 0.0f;
    static uint32_t last_pwm_ms = millis();
    uint32_t tnow = millis();
    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    float HB1_throttle = 0.0f;
    float thr_min = g2.hb1_engine_min.get();
    float thr_max = 100.0f;
    float thr_takeoff = g2.hb1_engine_takeoff.get();
    float timer = (millis() - state_timer);
    if (!arming.is_armed()) {
        switch (state) {
            case Power_Action_t::None:
            case Power_Action_t::RocketON:
            case Power_Action_t::EnginePullUP:
            case Power_Action_t::EngineON:
            case Power_Action_t::EngineOFF:
            case Power_Action_t::ParachuteON:
            case Power_Action_t::GROUND_RocketON:
                HB1_throttle = 0.0f;
                break;
            case Power_Action_t::GROUND_EngineSTART:
            case Power_Action_t::GROUND_EngineSTART_PRE:
                HB1_throttle = thr_min;
                break;
            case Power_Action_t::GROUND_EngineON:
                HB1_throttle = thr_min;
                break;
            case Power_Action_t::GROUND_EngineOFF:
                HB1_throttle = 0.0f;
                break;
            case Power_Action_t::GROUND_EngineFULL:
                if (timer < 10000.f) {
                    HB1_throttle = constrain_float(100.f*timer/10000.f, thr_min, thr_max);
                } else if (timer < 20000.f) {
                    HB1_throttle = constrain_float(100.f, thr_min, thr_max);
                } else {
                    HB1_throttle = thr_min;
                }
                break;
            case Power_Action_t::GROUND_EngineMID:
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

        switch (state) {
            case Power_Action_t::None:
                if (HB1_Status.state == HB1_Mission_Takeoff) {
                    HB1_throttle = thr_takeoff;
                } else {
                    HB1_throttle = throttle;
                }
                break;
            case Power_Action_t::RocketON:
                HB1_throttle = thr_takeoff;
                break;
            case Power_Action_t::EnginePullUP:
            case Power_Action_t::EngineON:
                HB1_throttle = constrain_float(throttle, thr_min, thr_max);
                break;
            case Power_Action_t::GROUND_EngineOFF:
            case Power_Action_t::EngineOFF:
            case Power_Action_t::ParachuteON:
                HB1_throttle = 0.0f;
                break;
            case Power_Action_t::GROUND_RocketON:
            case Power_Action_t::GROUND_EngineSTART:
            case Power_Action_t::GROUND_EngineSTART_PRE:
            case Power_Action_t::GROUND_EngineON:
            case Power_Action_t::GROUND_EngineFULL:
            case Power_Action_t::GROUND_EngineMID:
                HB1_throttle = thr_min;
                break;
            default:
                break;
        }
    }

    float delta_t = (float)(tnow - last_pwm_ms) * 0.001f;
    HB1_throttle_out = HB1_throttle_out + constrain_float(HB1_throttle - HB1_throttle_out, -delta_t*10.0f, delta_t*10.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle_HB1, HB1_throttle_out);

    last_pwm_ms = tnow;
}

void UPower::state_update()
{
    uint32_t timer = millis() - state_timer;
    switch (state) {
        case PowerAction_t::None:
            if (arming.is_armed() && !plane.throttle_suppressed && (control_mode == &mode_takeoff)) {
                set_Action(PowerAction_t::RocketON);
            }
            break;
        case PowerAction_t::RocketON:
            if (timer > (uint32_t)g2.hb1_rocket_timer_ms.get()) {
                float airspeed_measured = 0;
                if (!ahrs.airspeed_estimate(&airspeed_measured)) {airspeed_measured = 0.0f;}
                float gspd = ahrs.groundspeed_vector().length();
                bool speed_reached = (airspeed_measured > 10.0f || gspd > 5.0f) || (g2.hb1_test_mode != 0);
                if (speed_reached) {
//                    if (timer > 4000 || (ins.get_accel().x < 10.0f)) {
                        gcs().send_text(MAV_SEVERITY_INFO, "timer: %d ax %0.2f", (int32_t)timer, ins.get_accel().x);
                        set_Action(PowerAction_t::EngineON);
//                    }
                } else {
                    plane.set_mode(plane.mode_fbwa, MODE_REASON_UNAVAILABLE);
                    gcs().send_text(MAV_SEVERITY_INFO, "WARNING, low spd %0.2f, %0.2f", airspeed_measured, gspd);
                    set_Action(PowerAction_t::None);
                }
            }
            break;
        case PowerAction_t::EnginePullUP: // not use
            if (timer > 2000) {
                set_Action(PowerAction_t::EngineON);
            }
            break;
        case PowerAction_t::EngineON:
            if (plane.landing.is_flaring() || plane.landing.is_on_approach()) {
                if (plane.rangefinder_state.in_range) {
                    if (plane.rangefinder_state.height_estimate < 1.5f) {
                        gcs().send_text(MAV_SEVERITY_INFO, "Engine protection engaged at %.2fm", (double)rangefinder_state.height_estimate);
                        set_Action(PowerAction_t::EngineOFF);
                    }
                }
            }
            break;
        case PowerAction_t::EngineOFF:
            if (timer > 3000) {
                if (arming.is_armed()) {
                    set_Action(PowerAction_t::None);
                } else {
                    set_Action(PowerAction_t::None);
                }
            }
            break;
        case PowerAction_t::GROUND_EngineFULL:
        case PowerAction_t::GROUND_EngineMID:
            // if (timer > 2000) {
            //     set_Action(PowerAction_t::GROUND_EngineON);
            // }
            // break;
        case PowerAction_t::GROUND_EngineON:
            if (plane.HB1_engine_rpm.get() < 1000.f && (plane.g2.hb1_rpm_used == 1)) {
                if (engine_startcount < 6) {
                    set_Action(PowerAction_t::GROUND_EngineSTART);
                    gcs().send_text(MAV_SEVERITY_INFO, "Restart[%d]: %0.2f", engine_startcount, plane.HB1_engine_rpm.get());
                    engine_startcount += 1;
                } else {
                    gcs().send_text(MAV_SEVERITY_INFO, "Restart[%d] fail", engine_startcount); 
                    set_Action(PowerAction_t::None);   
                }
            }
            break;
        case PowerAction_t::ParachuteON:
            if (timer > 10000) {
                if (!arming.is_armed()) {
                    set_Action(PowerAction_t::None);
                }
            }
            break;
        case PowerAction_t::GROUND_EngineSTART_PRE:
            if (timer > 1000) {
                set_Action(PowerAction_t::GROUND_EngineSTART);
            }
            break;
        case PowerAction_t::GROUND_EngineSTART: // triggered by RC OR cmd
            if (timer > 1000) {
                set_Action(PowerAction_t::GROUND_EngineON);
            }
            break;
        case PowerAction_t::GROUND_RocketON: // triggered by RC
            if (timer > 5000) {
                set_Action(PowerAction_t::None);
            }
        case PowerAction_t::GROUND_EngineOFF: // triggered by RC OR cmd
            break;
        default:
            break;
    }
}

void UPower::uart_update()
{
    // receive
    if (uart_power.initialized()) {
        uart_power.read();
        msg_power2apm_handle();
    }

    // send
    uart_power_send();
}

void Plane::msg_power2apm_handle() {
    if (uart_mission.get_msg_power2apm()._msg_1.updated) {
        uart_mission.get_msg_power2apm()._msg_1.updated = false;
        HB1_power2apm &tmp_msg = uart_power.get_msg_power2apm();
        uint16_t rpm = ((uint16_t)tmp_msg._msg_1.content.msg.rpm_h << 8 | tmp_msg._msg_1.content.msg.rpm_l);
        uint16_t temp = ((uint16_t)tmp_msg._msg_1.content.msg.temp_h << 8 | tmp_msg._msg_1.content.msg.temp_l);

        engine_rpm.apply((float)rpm);
        engine_fuel = (float)tmp_msg._msg_1.content.msg.main_pwm;
        engine_temp = (float)temp;
        engine_status = tmp_msg._msg_1.content.msg.status;
    }
}

void UPower::uart_power_send()
{
    static uint32_t _last_send_ms = millis();
    uint32_t tnow = millis();
    if (tnow - _last_send_ms > 200) {
        _last_send_ms = tnow;
        if (send_counter>0) {
            send_counter--;
            if (uart_power.get_msg_apm2power()._msg_1.need_send) {
                uart_power.write();
                uart_power.get_msg_apm2power()._msg_1.need_send = true;
            }
        } else {
            throttle_update();
            uart_power.write();
        }
    }
}

void UPower::throttle_update()
{
    bool dont_send_throttle = false;
    switch (state) {
        case HB1_PowerAction_GROUND_EngineSTART_PRE:
        case HB1_PowerAction_GROUND_EngineSTART:
            dont_send_throttle = true;
            break;
        case HB1_PowerAction_None:
        case HB1_PowerAction_RocketON:
        case HB1_PowerAction_EnginePullUP:
        case HB1_PowerAction_EngineON:
        case HB1_PowerAction_EngineOFF:
        case HB1_PowerAction_ParachuteON:
        case HB1_PowerAction_GROUND_RocketON:
        case HB1_PowerAction_GROUND_EngineON:
        case HB1_PowerAction_GROUND_EngineOFF:
        case HB1_PowerAction_GROUND_EngineFULL:
        case HB1_PowerAction_GROUND_EngineMID:
        default:
            break;
    }

    if (dont_send_throttle) {return;}
    float throttle_out = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle_HB1);
    HB1_apm2power &tmp_msg = HB1_uart_power.get_msg_apm2power();
    tmp_msg._msg_1.need_send = true;
    // tmp_msg.set_throttle(throttle_out);
    uint16_t rpm_half = 17500 + (uint16_t)(throttle_out*500.f*0.5f);
    if (throttle_out < g2.hb1_engine60_min.get()) {
        rpm_half = 17500;
    }
    tmp_msg.set_rpm_half(rpm_half);
    tmp_msg.make_sum();
    tmp_msg._msg_1.print = false;
    return;
}

bool UPower::running()
{
    if (plane.g2.hb1_rpm_used == 1 && engine_rpm.get() > 1000.f) {
        return true;
    }
    return false;
}

int8_t UPower::engine_type() 
{
    return (plane.g2.hb1_power_type.get());
}

void UPower::set_Action(Power_Action_t action, bool Force_set = false)
{
    if ((state == action) && !Force_set) {
        return;
    } else {
        state = action;
    }
    state_timer = millis();
    SRV_Channels::set_output_scaled(SRV_Channel::k_parachute_HB1, 0);
    switch (state) {
        case PowerAction_t::None:
            plane.HB1_engine_startcount = 0;
            gcs().send_text(MAV_SEVERITY_INFO, "Power None");
            relay.off(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            break;
        case PowerAction_t::RocketON:
            gcs().send_text(MAV_SEVERITY_INFO, "Rocket ON");
            relay.on(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            plane.umission.already_takeoff = true;
            SRV_Channels::set_output_scaled(SRV_Channel::k_launcher_HB1, 100);
            break;
        case PowerAction_t::GROUND_RocketON:
            gcs().send_text(MAV_SEVERITY_INFO, "G Rocket ON");
            relay.on(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            break;
        case PowerAction_t::EnginePullUP:
            gcs().send_text(MAV_SEVERITY_INFO, "Engine Pull up");
            relay.off(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            break;
        case PowerAction_t::GROUND_EngineSTART_PRE:
            engine_startcount = 1;
            gcs().send_text(MAV_SEVERITY_INFO, "Engine Prepare");
            relay.off(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            break;
        case PowerAction_t::GROUND_EngineSTART:
            engine_startcount = 1;
            gcs().send_text(MAV_SEVERITY_INFO, "G Engine Starting");
            relay.off(0);
            relay.off(1);
            relay.off(2);
            relay.on(3);
            break;
        case PowerAction_t::GROUND_EngineON:
            gcs().send_text(MAV_SEVERITY_INFO, "G Engine ON");
            relay.off(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            break;
        case PowerAction_t::EngineON:
            gcs().send_text(MAV_SEVERITY_INFO, "Engine ON");
            relay.off(0);
            relay.off(1);
            relay.off(2);
            relay.off(3);
            break;
        case PowerAction_t::EngineOFF:
            gcs().send_text(MAV_SEVERITY_INFO, "Engine OFF");
            relay.off(0);
            relay.off(1);
            relay.on(2);
            relay.off(3);
            break;
        case PowerAction_t::GROUND_EngineOFF:
            gcs().send_text(MAV_SEVERITY_INFO, "G Engine OFF");
            relay.off(0);
            relay.off(1);
            relay.on(2);
            relay.off(3);
            break;
        case PowerAction_t::ParachuteON:
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
    msg_apm2power_set();
}

void UPower::msg_apm2power_set() {
    switch (state) {
        case HB1_PowerAction_RocketON:
        case HB1_PowerAction_GROUND_RocketON:
            msg_apm2power_set_rocket_on();
            return;
        default:
            break;
    }

    HB1_apm2power &tmp_msg = uart_power.get_msg_apm2power();
    tmp_msg._msg_1.need_send = false;
    switch (state) {
        case HB1_PowerAction_GROUND_EngineSTART:
            tmp_msg.set_engine_start();
            tmp_msg._msg_1.need_send = true;
            break;
        case HB1_PowerAction_GROUND_EngineSTART_PRE:
            tmp_msg.set_engine_stop();
            tmp_msg._msg_1.need_send = true;
            break;
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
        send_counter = 0;
        return;
    }
    send_counter = 1;

    tmp_msg.make_sum();
    tmp_msg._msg_1.print = true;
    return;
}

void Plane::msg_apm2power_set_rocket_on() {
    HB1_apm2rocket &tmp_msg = plane.umission.uart_mission.get_msg_apm2rocket();
    tmp_msg.set_rocket_on();
    tmp_msg.make_sum();
    tmp_msg._msg_1.print = true;
    tmp_msg._msg_1.need_send = true;
    plane.umission.send_counter = 2;
    return;
}