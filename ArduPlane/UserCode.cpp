#include "Plane.h"

void Plane::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    auto &sm = AP::serialmanager();
    uart_output = sm.find_serial(AP_SerialManager::SerialProtocol_OUTPUT, 0);
    _commanded_throttle = 0.0f;
}

void Plane::userhook_FastLoop()
{
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
}


void Plane::userhook_calc_throttle() {
    static uint32_t _last_call_ms = millis();
    float dt = (float)(millis() - _last_call_ms)*0.001f;
    if (dt > 0.1f) {
        dt = 0.1f;
        g2.user_thr_pid.reset_I();
    }
    float d_alt = (float)calc_altitude_error_cm() * 0.01f;
    float target_climb_rate = constrain_float(d_alt*0.5f, -0.5f, 0.5f);
    Vector3f vel;
    if (ahrs.get_velocity_NED(vel)) {
        ;
    }
    float current_climb_rate = -vel.z;
    // float delta_climb_rate = constrain_float(target_climb_rate - current_climb_rate, -0.5f, 0.5f);

    // _commanded_throttle = _commanded_throttle + delta_climb_rate*10.0f*dt;
    // constrain_float(_commanded_throttle, 0.0f, 100.f);
    _commanded_throttle = 50.f + 50.f * g2.user_thr_pid.update_all(target_climb_rate , current_climb_rate, dt);
    constrain_float(_commanded_throttle, 0.0f, 100.f);
    if (throttle_suppressed) {
        _commanded_throttle = 0.0f;
        g2.user_thr_pid.reset_I();
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _commanded_throttle);

    _last_call_ms = millis();

    static uint32_t _last_log_ms = millis();
    if (millis() - _last_log_ms > 100) {
        // gcs().send_text(MAV_SEVERITY_INFO, "%f | %f | %f", d_alt, target_climb_rate, current_climb_rate);
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