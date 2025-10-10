#include "Plane.h"

void Plane::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    auto &sm = AP::serialmanager();
    uart_output = sm.find_serial(AP_SerialManager::SerialProtocol_OUTPUT, 0);
    _user_throttle_out = 0.0f;
    _user_airspeed_target = 0.0f;
    _user_climbrate_p = 0.0f;
    _user_pitch_target = 0.0f;
    _user_land_flag = true;
    _user_rel_alt_filt.init(100.0f, 20);
}

void Plane::userhook_FastLoop()
{
    _user_rel_alt_filt.push(plane.relative_altitude);
    // put your 400Hz code here
    if (uart_output == nullptr) {return;}
    static uint32_t _last_send_ms = millis();
    if (millis() - _last_send_ms > 5) {
        _last_send_ms = millis();
    } else {
        return;
    }
    for (uint8_t i_servo = 1; i_servo <=20; i_servo++) {
        SRV_Channel *this_channel = SRV_Channels::srv_channel(i_servo-1);
        if (this_channel == nullptr) {
            continue;
        }
        uint16_t pwm = this_channel->get_output_pwm();
        if (pwm == 0) {
            pwm = 1500;
        }
        float pwm_value = constrain_float((float)pwm, 1000.f, 2000.f);
        int16_t servo_angle = (pwm_value - 1500.f)*12.f;//+-4500
        
        uart_output->write(0xAA);
        uart_output->write(i_servo);
        uart_output->write((uint8_t)(servo_angle&0xFF));
        uart_output->write((uint8_t)((servo_angle>>8)&0xFF));
        uart_output->write(0xFF);
    }

    uint16_t thr_left = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleLeft)*10.f;//100
    uint16_t thr_right = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleRight)*10.f;

    thr_left = 3000;
    thr_right = 3000;

    uart_output->write(0xAA);
    uart_output->write(0x21);
    uart_output->write((uint8_t)(thr_left&0xFF));
    uart_output->write((uint8_t)((thr_left>>8)&0xFF));
    uart_output->write(0xFF);

    uart_output->write(0xAA);
    uart_output->write(0x22);
    uart_output->write((uint8_t)(thr_left&0xFF));
    uart_output->write((uint8_t)((thr_left>>8)&0xFF));
    uart_output->write(0xFF);

    uart_output->write(0xAA);
    uart_output->write(0x23);
    uart_output->write((uint8_t)(thr_right&0xFF));
    uart_output->write((uint8_t)((thr_right>>8)&0xFF));
    uart_output->write(0xFF);

    uart_output->write(0xAA);
    uart_output->write(0x24);
    uart_output->write((uint8_t)(thr_right&0xFF));
    uart_output->write((uint8_t)((thr_right>>8)&0xFF));
    uart_output->write(0xFF);


    // thr_left = 65535/2 + SRV_Channels::get_output_scaled(SRV_Channel::k_throttleLeft)*277;//0~65535对应-90°到90°范围桨距角
    // thr_right = 65535/2 + SRV_Channels::get_output_scaled(SRV_Channel::k_throttleRight)*277;

    thr_left = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*10.f;//100
    thr_right = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*10.f;

    uart_output->write(0xAA);
    uart_output->write(101);
    uart_output->write((uint8_t)(thr_left&0xFF));
    uart_output->write((uint8_t)((thr_left>>8)&0xFF));
    uart_output->write(0xFF);

    uart_output->write(0xAA);
    uart_output->write(102);
    uart_output->write((uint8_t)(thr_left&0xFF));
    uart_output->write((uint8_t)((thr_left>>8)&0xFF));
    uart_output->write(0xFF);

    uart_output->write(0xAA);
    uart_output->write(103);
    uart_output->write((uint8_t)(thr_right&0xFF));
    uart_output->write((uint8_t)((thr_right>>8)&0xFF));
    uart_output->write(0xFF);

    uart_output->write(0xAA);
    uart_output->write(104);
    uart_output->write((uint8_t)(thr_right&0xFF));
    uart_output->write((uint8_t)((thr_right>>8)&0xFF));
    uart_output->write(0xFF);
}

void Plane::userhook_SlowLoop() {
    // 1Hz code
    // float tmp_airspeed;
    // if (ahrs.airspeed_estimate(tmp_airspeed)) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "airspeed %f", tmp_airspeed);
    // }
    // Vector3f velned;
    // if (ahrs.get_velocity_NED(velned)) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "velned (%f, %f, %f)", velned.x, velned.y, velned.z);
    // }
    // float _height;
    // ahrs.get_relative_position_D_home(_height);
    // gcs().send_text(MAV_SEVERITY_INFO, "home _height %f", _height);

    // gcs().send_text(MAV_SEVERITY_INFO, "home alt %d", int(ahrs.get_home().alt));

    // gcs().send_text(MAV_SEVERITY_INFO, "GPS healthy: %d", gps.is_healthy());

    // gcs().send_text(MAV_SEVERITY_INFO, "EKF type: %d", ahrs.get_ekf_type());

    userhook_param_check();
}

void Plane::userhook_auto_takeoff() {
    _user_climbrate_p = g2.user_climbrate_p_takeoff;

    Vector3f vel;
    if (ahrs.get_velocity_NED(vel)) {
        ;
    }
    float climb_rate_current = -vel.z;

    if ( (_user_rel_alt_filt.get() > 0.3f || climb_rate_current > 0.5f) && _user_land_flag) {
        _user_land_flag = false;
        g2.user_thr_pid.reset_I();
    }

    if (_user_land_flag) {
        _user_airspeed_target = g2.user_airspeed_target_takeoff;
        _user_pitch_target = 5.0f;
        g2.user_pth_pid.reset_I();
        userhook_calc_throttle();
    } else {
        float h = plane.relative_altitude;
        float cruiseVset = g2.user_airspeed_target_curise;
        float cruiseHset = g2.user_altitude_target_cruise;
        float takeoffVset = g2.user_airspeed_target_takeoff;
        _user_airspeed_target = MIN((cruiseVset-takeoffVset)/cruiseHset*h, cruiseVset-takeoffVset) + takeoffVset;
        userhook_calc_pitch();
        userhook_calc_throttle();
    }
    nav_pitch_cd = _user_pitch_target*100.f;
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _user_throttle_out);
}

void Plane::userhook_auto_cruise() {
    _user_airspeed_target = g2.user_airspeed_target_curise;
    _user_climbrate_p = g2.user_climbrate_p_curise;
    _user_land_flag = false;
    userhook_calc_pitch();
    userhook_calc_throttle();
    nav_pitch_cd = _user_pitch_target*100.f;
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _user_throttle_out);
}

void Plane::userhook_auto_land() {
    static uint32_t _last_call_ms = millis();
    float dt = (float)(millis() - _last_call_ms)*0.001f;
    _last_call_ms = millis();
    if (dt > 0.1f) {
        dt = 0.1f;
    }

    target_altitude.amsl_cm = ahrs.get_home().alt;

    float h = plane.relative_altitude;
    _user_airspeed_target = g2.user_airspeed_target_land;
    float cruiseHset = g2.user_altitude_target_cruise;
    _user_climbrate_p = g2.user_climbrate_land_p1 - (cruiseHset - h)*g2.user_climbrate_land_p2/cruiseHset;
    if (_user_rel_alt_filt.get() < 0.3f && !_user_land_flag) {
        _user_land_flag = true;
        _user_pitch_target = degrees(ahrs.get_pitch());
        _user_throttle_out = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    }
    if (_user_land_flag) {
        _user_pitch_target = _user_pitch_target + constrain_float(5.0f - _user_pitch_target, -1.0f, 1.0f)*dt;
        _user_throttle_out = _user_throttle_out + constrain_float(0.0f - _user_throttle_out, -1.0f, 1.0f)*dt;
    } else {
        float cruiseVset = g2.user_airspeed_target_curise;
        float landVset = g2.user_airspeed_target_land;
        _user_airspeed_target = MIN((cruiseVset-landVset)/cruiseHset*h, cruiseVset-landVset) + landVset;
        userhook_calc_pitch();
        userhook_calc_throttle();
    }
    nav_pitch_cd = _user_pitch_target*100.f;
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _user_throttle_out);
}

void Plane::userhook_calc_pitch() {
    static uint32_t _last_call_ms = millis();
    float dt = (float)(millis() - _last_call_ms)*0.001f;
    _last_call_ms = millis();
    if (dt > 0.1f) {
        dt = 0.1f;
        g2.user_pth_pid.set_integrator(ahrs.get_pitch() - radians(5.0f));
    }

    float TAS = ahrs.get_EAS2TAS();
    float alt_err = constrain_float((float)calc_altitude_error_cm() * 0.01f, -3.0f, 3.0f);
    float climb_rate_target = alt_err*_user_climbrate_p;
    float airspeed_current = 10.0f;
    if (ahrs.airspeed_estimate(airspeed_current)) {
        airspeed_current = MAX(10.0f, airspeed_current);
    }
    float gamma_target = asinf(climb_rate_target/airspeed_current/TAS);
    gamma_target = constrain_float(gamma_target, -0.05f, 0.05f);

    Vector3f vel;
    if (ahrs.get_velocity_NED(vel)) {
        ;
    }
    float climb_rate_current = -vel.z;
    float gamma_current = asinf(climb_rate_current/airspeed_current/TAS);

    float gamma_error = constrain_float(gamma_target - gamma_current, -0.17f, 0.17f);

    float theta_out = g2.user_pth_pid.update_all(degrees(gamma_error), 0.0f, dt);
    _user_pitch_target = theta_out + 5.0f;


    static uint32_t _last_log_ms = millis();
    if (millis() - _last_log_ms > 100) {
        // gcs().send_text(MAV_SEVERITY_INFO, "tar alt cm %f ", (float)calc_altitude_error_cm());

        _last_log_ms = millis();
        AP::logger().WriteStreaming("UPTH",
                                    "TimeUS,target,actual,ff,P,I,D,srate,dmod",
                                    "s--------",
                                    "F--------",
                                    "Qffffffff",
                                    AP_HAL::micros64(),
                                    (float)g2.user_pth_pid.get_pid_info().target,
                                    (float)g2.user_pth_pid.get_pid_info().actual,
                                    (float)g2.user_pth_pid.get_pid_info().FF,
                                    (float)g2.user_pth_pid.get_pid_info().P,
                                    (float)g2.user_pth_pid.get_pid_info().I,
                                    (float)g2.user_pth_pid.get_pid_info().D,
                                    (float)g2.user_pth_pid.get_pid_info().slew_rate,
                                    (float)g2.user_pth_pid.get_pid_info().Dmod);
        AP::logger().WriteStreaming("UPH2",
                                    "TimeUS,gamtgt,gamcrt,gamerr,clbtgt,clbcrt,thtout,alterr,dt",
                                    "s--------",
                                    "F--------",
                                    "Qffffffff",
                                    AP_HAL::micros64(),
                                    (float)gamma_target,
                                    (float)gamma_current,
                                    (float)gamma_error,
                                    (float)climb_rate_target,
                                    (float)climb_rate_current,
                                    (float)theta_out,
                                    (float)alt_err,
                                    (float)dt);
    }
}

void Plane::userhook_calc_throttle() {
    static uint32_t _last_call_ms = millis();
    float dt = (float)(millis() - _last_call_ms)*0.001f;
    _last_call_ms = millis();
    if (dt > 0.1f) {
        dt = 0.1f;
        g2.user_thr_pid.reset_I();
        gcs().send_text(MAV_SEVERITY_INFO, "user_thr_pid.reset_I()");
    }

    float airspeed_current = 1.0f;
    if (ahrs.airspeed_estimate(airspeed_current)) {
        airspeed_current = MAX(1.0f, airspeed_current);
    }

    _user_throttle_out = 25.f + 100.f * g2.user_thr_pid.update_all(_user_airspeed_target , airspeed_current, dt);
    constrain_float(_user_throttle_out, 0.0f, 100.f);
    if (throttle_suppressed) {
        _user_throttle_out = 0.0f;
        g2.user_thr_pid.reset_I();
    }

    static uint32_t _last_log_ms = millis();
    if (millis() - _last_log_ms > 100) {
        _last_log_ms = millis();
        AP::logger().WriteStreaming("UTHR",
                                    "TimeUS,target,actual,ff,P,I,D,srate,dmod",
                                    "s--------",
                                    "F--------",
                                    "Qffffffff",
                                    AP_HAL::micros64(),
                                    (float)g2.user_thr_pid.get_pid_info().target,
                                    (float)g2.user_thr_pid.get_pid_info().actual,
                                    (float)g2.user_thr_pid.get_pid_info().FF,
                                    (float)g2.user_thr_pid.get_pid_info().P,
                                    (float)g2.user_thr_pid.get_pid_info().I,
                                    (float)g2.user_thr_pid.get_pid_info().D,
                                    (float)g2.user_thr_pid.get_pid_info().slew_rate,
                                    (float)g2.user_thr_pid.get_pid_info().Dmod);
    }
}

void Plane::userhook_param_check() {
    FD_CAN_2 *can_2 = nullptr;
    for (uint8_t i_can = 0; i < AP::can().get_num_drivers(); i_can++) {
        if (AP::can().get_driver_type(i_can) == AP_CAN::Protocol::FDCAN1) {
            can_2 = (FD_CAN_2*)AP::can().get_driver(i_can);
            break;
        }
    }

    static uint8_t mot_1 = g2.user_mot_1.get();
    if (mot_1 != g2.user_mot_1.get()) {
        mot_1 = g2.user_mot_1.get();
        if (mot_1) {
            if (can_2 != nullptr && can_2->_bms != nullptr) {
                can_2->_bms->do_power_on(1);
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 1 POWER ON");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 1 FAIL");
            }
        } else {
            if (can_2 != nullptr && can_2->_bms != nullptr) {
                can_2->_bms->do_power_off(1);
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 1 POWER OFF");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 1 FAIL");
            }
        }
    }
    static uint8_t mot_2 = g2.user_mot_2.get();
    if (mot_2 != g2.user_mot_2.get()) {
        mot_2 = g2.user_mot_2.get();
        if (mot_2) {
            if (can_2 != nullptr && can_2->_bms != nullptr) {
                can_2->_bms->do_power_on(2);
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 2 POWER ON");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 2 FAIL");
            }
        } else {
            if (can_2 != nullptr && can_2->_bms != nullptr) {
                can_2->_bms->do_power_off(2);
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 2 POWER OFF");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 2 FAIL");
            }
        }
    }
    static uint8_t mot_3 = g2.user_mot_3.get();
    if (mot_3 != g2.user_mot_3.get()) {
        mot_3 = g2.user_mot_3.get();
        if (mot_3) {
            if (can_2 != nullptr && can_2->_bms != nullptr) {
                can_2->_bms->do_power_on(3);
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 3 POWER ON");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 3 FAIL");
            }
        } else {
            if (can_2 != nullptr && can_2->_bms != nullptr) {
                can_2->_bms->do_power_off(3);
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 3 POWER OFF");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 3 FAIL");
            }
        }
    }
    static uint8_t mot_4 = g2.user_mot_4.get();
    if (mot_4 != g2.user_mot_4.get()) {
        mot_4 = g2.user_mot_4.get();
        if (mot_4) {
            if (can_2 != nullptr && can_2->_bms != nullptr) {
                can_2->_bms->do_power_on(4);
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 4 POWER ON");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 4 FAIL");
            }
        } else {
            if (can_2 != nullptr && can_2->_bms != nullptr) {
                can_2->_bms->do_power_off(4);
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 4 POWER OFF");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "MOT 4 FAIL");
            }
        }
    }
    static uint8_t blower = g2.user_blower.get();
    if (blower != g2.user_blower.get()) {
        blower = g2.user_blower.get();
        if (blower) {
            if (can_2 != nullptr && can_2->_blower != nullptr) {
                can_2->_blower->do_power_on(4);
                gcs().send_text(MAV_SEVERITY_INFO, "BLOWER POWER ON");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "BLOWER FAIL");
            }
        } else {
            if (can_2 != nullptr && can_2->_blower != nullptr) {
                can_2->_blower->do_power_off(4);
                gcs().send_text(MAV_SEVERITY_INFO, "BLOWER POWER OFF");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "BLOWER FAIL");
            }
        }
    }
}
