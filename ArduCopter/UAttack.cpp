#include "Copter.h"

const AP_Param::GroupInfo UAttack::var_info[] = {

    AP_SUBGROUPINFO(attack_throttle_pid, "THR_", 0, UAttack, AC_PID),
    AP_SUBGROUPINFO(attack_roll_pid    , "RLL_", 1, UAttack, AC_PID),
    AP_SUBGROUPINFO(attack_velx_pid    , "VELX_", 2, UAttack, AC_PID),
    AP_SUBGROUPINFO(attack_vely_pid    , "VELY_", 3, UAttack, AC_PID),
    AP_GROUPINFO("UPRINT",     4, UAttack, print,                   0),
    AP_GROUPINFO("TCAM_TYPE",  5, UAttack, use_target_cam_type,     0),
    AP_GROUPINFO("FILT_Y_HZ",  6, UAttack, filt_yaw_hz,             5.0f),
    AP_GROUPINFO("FILT_P_HZ",  7, UAttack, filt_pithc_hz,           5.0f),

    AP_SUBGROUPPTR(_Target_ptr_cam_QD,   "TQD_",   8, UAttack,  FD_Target_QD),
    AP_GROUPEND
};

UAttack::UAttack()
{
    AP_Param::setup_object_defaults(this, var_info);

    _last_yaw = 0.0f;
    _last_yaw_sample = 0.0f;
}

// initialise
void UAttack::init()
{
    udelay.init();
    _active = false;
    bf_info.x = 0.0f;
    bf_info.y = 0.0f;
    ef_info.x = 0.0f;
    ef_info.y = 0.0f;
    ef_rate_info.x = 0.0f;
    ef_rate_info.y = 0.0f;
    display_info.new_data = false;
    display_info.p1 = 0.0f;
    display_info.p2 = 0.0f;
    display_info.p3 = 0.0f;
    display_info.p4 = 0.0f;
    display_info.p11 = 0.0f;
    display_info.p12 = 0.0f;
    display_info.p13 = 0.0f;
    display_info.p14 = 0.0f;
    display_info.p21 = 0.0f;
    display_info.p22 = 0.0f;
    display_info.p23 = 0.0f;
    display_info.p24 = 0.0f;
    display_info.count = 0;
    _target_vel_x = 0.0f;
    _target_vel_y = 0.0f;
    _target_roll_angle = 0.0f;
    _Target_ptr_cam = nullptr;
    _Target_ptr_cam_QD = nullptr;
    _last_control_ms = millis();
    _last_reset_ms = 0;
    _last_log_ms = 0;
    _reset = true;
    _running = false;
    init_target();

    _yaw_sample_filter.set_cutoff_frequency(30.f, filt_yaw_hz.get());
    _pitch_sample_filter.set_cutoff_frequency(30.f, filt_pithc_hz.get());
    gcs().send_text(MAV_SEVERITY_WARNING, "Target FILT HZ [%0.0f, %0.0f]", filt_yaw_hz.get(), filt_pithc_hz.get());
}

void UAttack::udpate_control_value(){
    update_target_vel_x();
    update_target_vel_y();
    if (millis() - _last_reset_ms < 1500) {
        _target_vel_x = 0.0f;
        _target_vel_y = 0.0f;
    }
    _last_control_ms = millis();
}

void UAttack::update_log() {
    if (!is_active()) {return;}
    if (millis() - _last_log_ms < 100) {return;}
    _last_log_ms = millis();
    AP::logger().WriteStreaming("UTGT",
                                "TimeUS,bfx,bfy,efx,efy,efrx,efry,start,rate",
                                "s---------",
                                "F---------",
                                "Qfffffffff",
                                AP_HAL::micros64(),
                                (float)bf_info.x,
                                (float)bf_info.y,
                                (float)ef_info.x,
                                (float)ef_info.y,
                                (float)ef_rate_info.x,
                                (float)ef_rate_info.y,
                                (float)_running,
                                (float)display_info.count_log);

    AP::logger().WriteStreaming("UVEX",
                                "TimeUS,target,actual,ff,P,I,D,srate,dmod",
                                "s--------",
                                "F--------",
                                "Qffffffff",
                                AP_HAL::micros64(),
                                (float)attack_velx_pid.get_pid_info().target,
                                (float)attack_velx_pid.get_pid_info().actual,
                                (float)attack_velx_pid.get_pid_info().FF,
                                (float)attack_velx_pid.get_pid_info().P,
                                (float)attack_velx_pid.get_pid_info().I,
                                (float)attack_velx_pid.get_pid_info().D,
                                (float)attack_velx_pid.get_pid_info().slew_rate,
                                (float)attack_velx_pid.get_pid_info().Dmod);

    AP::logger().WriteStreaming("UVEY",
                                "TimeUS,target,actual,ff,P,I,D,srate,dmod",
                                "s--------",
                                "F--------",
                                "Qffffffff",
                                AP_HAL::micros64(),
                                (float)attack_vely_pid.get_pid_info().target,
                                (float)attack_vely_pid.get_pid_info().actual,
                                (float)attack_vely_pid.get_pid_info().FF,
                                (float)attack_vely_pid.get_pid_info().P,
                                (float)attack_vely_pid.get_pid_info().I,
                                (float)attack_vely_pid.get_pid_info().D,
                                (float)attack_vely_pid.get_pid_info().slew_rate,
                                (float)attack_vely_pid.get_pid_info().Dmod);

}

const Vector2f& UAttack::get_bf_info() {
    return bf_info;
}

const Vector2f& UAttack::get_ef_info() {
    return ef_info;
}

const Vector2f& UAttack::get_ef_rate_info() {
    return ef_rate_info;
}


void UAttack::init_target()
{
    bool use_cam = use_target_cam_type.get()>0;

    if (use_cam) {
         // 1:QD
        if (use_target_cam_type.get() == 1) {
            _Target_ptr_cam_QD= new FD_Target_QD();
            if (_Target_ptr_cam_QD->init()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target QD init");
                _Target_ptr_cam = _Target_ptr_cam_QD;
                AP_Param::load_object_from_eeprom(_Target_ptr_cam_QD, FD_Target_QD::var_info);
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target QD Fail");
                _Target_ptr_cam_QD= nullptr;
            }
        }
        else {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target CAM UNKNOW %d", use_target_cam_type.get());
            _Target_ptr_cam = nullptr;
        }
    }

}

void UAttack::start()
{
    _running = true;
    copter.uattack.attack_throttle_pid.reset_I();
    copter.uattack.attack_throttle_pid.reset_filter();
    copter.uattack.attack_throttle_pid.set_integrator(_throttle_filt.get());
    copter.uattack.attack_roll_pid.reset_I();
    copter.uattack.attack_roll_pid.reset_filter();
    copter.uattack.attack_roll_pid.set_integrator(degrees(_roll_filt.get()));
    if (attack_angle.get() <= 0.0f) {
        _attack_angle_target = -degrees(_pitch_filt.get());
    } else {
        _attack_angle_target = attack_angle.get();
    }
    gcs().send_text(MAV_SEVERITY_INFO, "ATT ANGLE: %f", _attack_angle_target);
    gcs().send_text(MAV_SEVERITY_INFO, "ATT THROTTLE: %f", _throttle_filt.get());
    gcs().send_text(MAV_SEVERITY_INFO, "ATT ROLL: %f", degrees(_roll_filt.get()));
    _last_control_ms = millis();
}

void UAttack::stop()
{
    _running = false;
}

// called at 100 Hz
void UAttack::update()
{
    update_cam();
    update_control();
    update_log();
}

void UAttack::update_cam()
{
    // for log purpose
    static uint32_t last_count_ms = millis();
    uint32_t tnow_ms = millis();
    if (tnow_ms - last_count_ms > 1000) {
        display_info.count_log = display_info.count;
        display_info.count = 0;
        last_count_ms = tnow_ms;
        //update filter cutoff HZ in flight
        _yaw_sample_filter.set_cutoff_frequency(60.f, filt_yaw_hz.get());
        _pitch_sample_filter.set_cutoff_frequency(60.f, filt_pithc_hz.get());
        _ef_rate_x_filter.set_cutoff_frequency(60.f, 1.0f);
        _ef_rate_y_filter.set_cutoff_frequency(60.f, 1.0f);
    }

    if (_Target_ptr_cam != nullptr) {
        _Target_ptr_cam->update();
    }

    if (_Target_ptr_cam != nullptr && _Target_ptr_cam->is_valid()) {
        if (current_idx != 1) {
            gcs().send_text(MAV_SEVERITY_INFO, "Change to CAM");
        }
        current_idx = 1;
    } else {
        if (current_idx != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "No Valid Target");
            _yaw_sample_filter.reset();
            _pitch_sample_filter.reset();
            _ef_rate_x_filter.reset();
            _ef_rate_y_filter.reset();
            _yaw_filter.reset();
            _pitch_filter.reset();
        }
        current_idx = 0;
        _reset = true;
        _last_reset_ms = millis();
    }
}

void UAttack::update_control()
{
    float p1 = 0;
    float p2 = 0;
    if (current_idx == 1) {
        if (_Target_ptr_cam->get_info(p1, p2)) {
            handle_info(p1, p2);
            if (_running) {
                udpate_control_value();
            }
        }
    } else if (current_idx == 2) {
        if (_Target_ptr_loc->get_info(p1, p2)) {
            handle_info(p1, p2);
            if (_running) {
                udpate_control_value();
            }
        }
    } else {
        _target_pitch_rate = 0.0f;
        _target_roll_angle = 0.0f;
        _target_yaw_rate = 0.0f;
    }

}


void UAttack::handle_info(float p1, float p2) {

    display_info.p3 = p1;
    display_info.p4 = p2;

    float _roll = AP::ahrs().get_roll();
    float _pitch = AP::ahrs().get_pitch();
    float _yaw = AP::ahrs().get_yaw();

    // if (!udelay.get_idx(10-1, _roll, _pitch, _yaw)) {
    //     _roll = AP::ahrs().get_roll();
    //     _pitch = AP::ahrs().get_pitch();
    //     _yaw = AP::ahrs().get_yaw();
    // }

    bf_info.x = p1; // yaw degree
    bf_info.y = p2; // pitch degree

    if (p2 < -90.f) {
        p2 = -180.0f - p2;
    } else if (p2 > 90.0f) {
        p2 = 180.0f - p2;
    }

    Vector3f target_unit = Vector3f(1.0f, 0.0f, 0.0f);
    Matrix3f tmp_target_cam_m;
    tmp_target_cam_m.from_euler(0.0f, radians(p2), radians(p1));
    Matrix3f tmp_cam_body_m;
    tmp_cam_body_m.from_euler(0.0f, radians(0.0f), radians(0.0f));
    Matrix3f tmp_body_earth_m;
    tmp_body_earth_m.from_euler(_roll, _pitch, _yaw);
    Matrix3f tmp_target_earth_m = tmp_body_earth_m*tmp_cam_body_m*tmp_target_cam_m;
    Vector3f ef_unit = tmp_target_earth_m*target_unit;

    // static uint32_t last_info_ms = millis();
    // if (millis() - last_info_ms > 1000) {
    //     last_info_ms = millis();
    //     gcs().send_text(MAV_SEVERITY_INFO, "KKKKKK (%f, %f, %f)", ef_unit.x, ef_unit.y, ef_unit.z);
    //     gcs().send_text(MAV_SEVERITY_INFO, "VVVVVV (%f, %f, %f)", degrees(_roll), degrees(_pitch), degrees(_yaw));
    // }

    float angle_pitch = wrap_180(degrees(atan2f(-ef_unit.z, ef_unit.xy().length())));
    float angle_yaw =   wrap_180(degrees(atan2f( ef_unit.y, ef_unit.x)));

    ef_info.x = angle_yaw;
    ef_info.y = angle_pitch;

    float delta_yaw = wrap_180(wrap_360(angle_yaw) - wrap_360(_last_yaw));
    _last_yaw = angle_yaw;
    _last_yaw_sample += delta_yaw;

    if (_reset) {
        _last_yaw_sample = _last_yaw;
        _yaw_sample_filter.reset();
        _pitch_sample_filter.reset();
        _ef_rate_x_filter.reset();
        _ef_rate_y_filter.reset();
        _yaw_filter.reset();
        _pitch_filter.reset();
        _reset = false;
    }

    _yaw_sample_filter.apply(_last_yaw_sample);
    _pitch_sample_filter.apply(angle_pitch);

    _yaw_filter.update(_yaw_sample_filter.get(), millis());
    _pitch_filter.update(_pitch_sample_filter.get(), millis());


    _ef_rate_x_filter.apply(_yaw_filter.slope()*1000.f);
    _ef_rate_y_filter.apply(_pitch_filter.slope()*1000.f);

    ef_rate_info.x = _ef_rate_x_filter.get();
    ef_rate_info.y = _ef_rate_y_filter.get();

    display_info.new_data = true;
    display_info.count++;
}

// m/s
void UAttack::update_target_vel_x() {
    float dt = (millis() - _last_control_ms);
    dt = dt * 0.001f;
    // if (dt > 1.0f) {attack_roll_pid.reset_I();}
    if (dt > 0.05f) {dt = 0.05f;}

    Vector3f vel_ned;
    if (copter.ahrs.get_velocity_NED(vel_ned)) {
        ;
    }

    // Vector3f vel_ef_xy = Vector3f(vel_ned.x, vel_ned.y, 0.0f);
    // Matrix3f tmp_body_earth_m;
    // tmp_body_earth_m.from_euler(0.0f, radians(0.0f), AP::ahrs().get_yaw());
    // tmp_body_earth_m.transpose();
    // Vector3f vel_bf_xy = tmp_body_earth_m*vel_ef_xy;

    // _target_vel_x = attack_velx_pid.update_all(0.0f, vel_bf_xy.y, dt);
    // attack_roll_pid.reset_I();
    // attack_roll_pid.reset_filter();
}

// m/s
void UAttack::update_target_vel_y() {
    float dt = (millis() - _last_control_ms);
    dt = dt * 0.001f;
    // if (dt > 1.0f) {attack_roll_pid.reset_I();}
    if (dt > 0.05f) {dt = 0.05f;}

    Vector3f vel_ned;
    if (copter.ahrs.get_velocity_NED(vel_ned)) {
        ;
    }

    // Vector3f vel_ef_xy = Vector3f(vel_ned.x, vel_ned.y, 0.0f);
    // Matrix3f tmp_body_earth_m;
    // tmp_body_earth_m.from_euler(0.0f, radians(0.0f), AP::ahrs().get_yaw());
    // tmp_body_earth_m.transpose();
    // Vector3f vel_bf_xy = tmp_body_earth_m*vel_ef_xy;

    // _target_vel_y = attack_vely_pid.update_all(0.0f, vel_bf_xy.y, dt);
    // attack_roll_pid.reset_I();
    // attack_roll_pid.reset_filter();
}

void UAttack::handle_attack_msg(const mavlink_message_t &msg) {
    if (_Target_ptr_cam != nullptr) {
        _Target_ptr_cam->handle_msg(msg);
    }
}


// UDelay
void UAttack::UDelay::init()
{
    _idx = 0;
    for (uint16_t i = 0; i < UDELAY_BUFFER; i++) {
        _buffer[i].roll = 0.0f;
        _buffer[i].pitch = 0.0f;
        _buffer[i].yaw = 0.0f;
        _buffer[i].time_ms = 0;
    }
}

void UAttack::UDelay::push()
{
    _idx += 1;
    if (_idx >= UDELAY_BUFFER) {
        _idx = 0;
    }
    _buffer[_idx].roll  = AP::ahrs().get_roll();
    _buffer[_idx].pitch = AP::ahrs().get_pitch();
    _buffer[_idx].yaw   = AP::ahrs().get_yaw();
    _buffer[_idx].time_ms = millis();
}

bool UAttack::UDelay::get_idx(uint16_t step, float &roll, float &pitch, float &yaw) 
{
    uint16_t this_idx = 0;
    if (_idx >= step) {
        this_idx = _idx - step;
    } else {
        this_idx = UDELAY_BUFFER + _idx - step;
    }
    roll = _buffer[this_idx].roll;
    pitch = _buffer[this_idx].pitch;
    yaw = _buffer[this_idx].yaw;
    if (millis() - _buffer[this_idx].time_ms > 500) {
        return false;
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "%d", (millis()-_buffer[this_idx].time_ms));
    return true;
}
