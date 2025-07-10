#include "Copter.h"

const AP_Param::GroupInfo UGimbal::var_info[] = {

    AP_SUBGROUPINFO(lock_yaw_pid,     "CYAW_", 0, UGimbal, AC_PID),
    AP_SUBGROUPINFO(lock_pitch_pid,   "CPTH_", 1, UGimbal, AC_PID),
    AP_GROUPINFO("UPRINT",     2, UGimbal, print,                   0),
    AP_GROUPINFO("CAM_USE",    3, UGimbal, use_gimbal_cam,          0),
    AP_GROUPINFO("LOC_USE",    4, UGimbal, use_gimbal_loc,          0),
    AP_GROUPINFO("FILT_Y_HZ",  5, UGimbal, filt_yaw_hz,             2.0f),
    AP_GROUPINFO("FILT_P_HZ",  6, UGimbal, filt_pithc_hz,           2.0f),
    AP_SUBGROUPPTR(_Gimbal_ptr_loc,   "TL_",    7, UGimbal,  FD_Gimbal_Loc),
    AP_SUBGROUPPTR(_Gimbal_ptr_cam,   "TC_",    8, UGimbal,  FD_Gimbal_HaoFu),

    AP_GROUPEND
};

UGimbal::UGimbal()
{
    AP_Param::setup_object_defaults(this, var_info);

    _last_yaw = 0.0f;
    _last_yaw_sample = 0.0f;
}

// initialise
void UGimbal::init()
{
    udelay.init();
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

    _ret_valid = false;

    _yaw_sample_filter.set_cutoff_frequency(filt_yaw_hz.get());
    _pitch_sample_filter.set_cutoff_frequency(filt_pithc_hz.get());
    gcs().send_text(MAV_SEVERITY_WARNING, "Target FILT HZ [%0.0f, %0.0f]", filt_yaw_hz.get(), filt_pithc_hz.get());

    init_gimbal();
}


void UGimbal::update_log() {
    static uint32_t _last_log_ms = millis();
    if (millis() - _last_log_ms > 100) {
        _last_log_ms = millis();
        AP::logger().WriteStreaming("UGB1",
                                    "TimeUS,bfx,bfy,efx,efy,efrx,efry,gpth,gyaw",
                                    "s--------",
                                    "F--------",
                                    "Qffffffff",
                                    AP_HAL::micros64(),
                                    (float)bf_info.x,
                                    (float)bf_info.y,
                                    (float)ef_info.x,
                                    (float)ef_info.y,
                                    (float)ef_rate_info.x,
                                    (float)ef_rate_info.y,
                                    (float)_gimbal_pitch_rate,
                                    (float)_gimbal_yaw_rate);

        // AP::logger().WriteStreaming("UGB2",
        //                             "TimeUS,angt,angm,agrt,agrm",
        //                             "s----",
        //                             "F----",
        //                             "Qffff",
        //                             AP_HAL::micros64(),
        //                             (float)_attack_angle_target,
        //                             (float)_attack_angle_measure,
        //                             (float)_attack_angle_rate_target,
        //                             (float)_attack_angle_rate_measure);

        AP::logger().WriteStreaming("UGBY",
                                    "TimeUS,target,actual,ff,P,I,D,srate,dmod",
                                    "s--------",
                                    "F--------",
                                    "Qffffffff",
                                    AP_HAL::micros64(),
                                    (float)lock_yaw_pid.get_pid_info().target,
                                    (float)lock_yaw_pid.get_pid_info().actual,
                                    (float)lock_yaw_pid.get_pid_info().FF,
                                    (float)lock_yaw_pid.get_pid_info().P,
                                    (float)lock_yaw_pid.get_pid_info().I,
                                    (float)lock_yaw_pid.get_pid_info().D,
                                    (float)lock_yaw_pid.get_pid_info().slew_rate,
                                    (float)lock_yaw_pid.get_pid_info().Dmod);

        AP::logger().WriteStreaming("UGBP",
                                    "TimeUS,target,actual,ff,P,I,D,srate,dmod",
                                    "s--------",
                                    "F--------",
                                    "Qffffffff",
                                    AP_HAL::micros64(),
                                    (float)lock_pitch_pid.get_pid_info().target,
                                    (float)lock_pitch_pid.get_pid_info().actual,
                                    (float)lock_pitch_pid.get_pid_info().FF,
                                    (float)lock_pitch_pid.get_pid_info().P,
                                    (float)lock_pitch_pid.get_pid_info().I,
                                    (float)lock_pitch_pid.get_pid_info().D,
                                    (float)lock_pitch_pid.get_pid_info().slew_rate,
                                    (float)lock_pitch_pid.get_pid_info().Dmod);

    }

}

const Vector2f& UGimbal::get_bf_info() {
    return bf_info;
}

const Vector2f& UGimbal::get_ef_info() {
    return ef_info;
}

const Vector2f& UGimbal::get_ef_rate_info() {
    return ef_rate_info;
}

bool UGimbal::have_target() {
    if (_Gimbal_ptr == nullptr) {
        return false;
    } else {
        return _ret_valid;
    }
    return false;
}

void UGimbal::init_gimbal()
{
    bool use_cam = use_gimbal_cam.get();
    bool use_loc = use_gimbal_loc.get();

    if (use_cam) {
        _Gimbal_ptr_cam = new FD_Gimbal_HaoFu();
        if (_Gimbal_ptr_cam->init()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Gimbal Cam init");
            _Gimbal_ptr = _Gimbal_ptr_cam;
            AP_Param::load_object_from_eeprom(_Gimbal_ptr_cam, FD_Gimbal_HaoFu::var_info);
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING, "Gimbal Cam Fail");
            _Gimbal_ptr_cam = nullptr;
            _Gimbal_ptr = nullptr;
        }
    } else if (use_loc) {
        _Gimbal_ptr_loc = new FD_Gimbal_Loc();
        if (_Gimbal_ptr_loc->init()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Gimbal Loc init");
            _Gimbal_ptr = _Gimbal_ptr_loc;
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING, "Gimbal Loc Fail");
            _Gimbal_ptr_loc = nullptr;
            _Gimbal_ptr = nullptr;
        }
    }
    else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Gimbal TYPE UNKNOW");
        _Gimbal_ptr = nullptr;
    }
}


// called at 100 Hz
void UGimbal::update()
{
    if (_Gimbal_ptr == nullptr) {return;}
    gimbal_ret_update();
    gimbal_control_update();
}

void UGimbal::gimbal_ret_update()
{
    // for log purpose
    static uint32_t last_count_ms = millis();
    uint32_t tnow_ms = millis();
    if (tnow_ms - last_count_ms > 1000) {
        display_info.count_log = display_info.count;
        display_info.count = 0;
        last_count_ms = tnow_ms;
    }

    if (_Gimbal_ptr != nullptr) {
        _Gimbal_ptr->update();
        _Gimbal_ptr->get_attitude_euler(_cam_roll, _cam_pitch, _cam_bf_yaw);
        _cam_yaw = wrap_2PI(AP::ahrs().get_yaw() + _cam_bf_yaw);
    } else {
        _gimbal_pitch_rate = 0.0f;
        _gimbal_yaw_rate = 0.0f;
        return;
    }

    _ret_valid = _Gimbal_ptr->have_target();
    if (_ret_valid) {
        ;
    }

    float p1 = 0;
    float p2 = 0;
    if (_Gimbal_ptr->get_info(p1, p2)) {
        handle_info_final(p1, p2);
    }
}

void UGimbal::handle_info_final(float p1, float p2) {
    static uint32_t last_info_ms = millis();
    float dt = (float)(millis() - last_info_ms) * 0.001f;
    last_info_ms = millis();

    display_info.p3 = p1;
    display_info.p4 = p2;

    // float _roll = AP::ahrs().get_roll();
    // float _pitch = AP::ahrs().get_pitch();
    // float _yaw = AP::ahrs().get_yaw();
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
    Matrix3f tmp_cam_earth_m;
    tmp_cam_earth_m.from_euler(_cam_roll, _cam_pitch, _cam_yaw);
    Matrix3f tmp_target_earth_m = tmp_cam_earth_m*tmp_target_cam_m;
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

    _yaw_sample_filter.apply(_last_yaw_sample, dt);
    _pitch_sample_filter.apply(angle_pitch, dt);

    _yaw_filter.update(_yaw_sample_filter.get(), millis());
    _pitch_filter.update(_pitch_sample_filter.get(), millis());

    ef_rate_info.x = _yaw_filter.slope()*1000.f;
    ef_rate_info.y = _pitch_filter.slope()*1000.f;

    display_info.new_data = true;
    display_info.count++;
}

void UGimbal::gimbal_control_update()
{
    // for log purpose
    static uint32_t last_set_ms = millis();
    uint32_t tnow_ms = millis();
    if (tnow_ms - last_set_ms > 1000) {
        last_set_ms = tnow_ms;
        //update filter cutoff HZ in flight
        _yaw_sample_filter.set_cutoff_frequency(filt_yaw_hz.get());
        _pitch_sample_filter.set_cutoff_frequency(filt_pithc_hz.get());
    }


    switch (_state) {
        default:
        case Gimbal_State::Ahead:
        {
            float target_pitch = -45.0f;
            float target_yaw = degrees(AP::ahrs().get_yaw());
            do_gimbal_attitude_control(target_pitch, target_yaw);
        }
        break;
        case Gimbal_State::Search:
        {
            float target_pitch = -45.0f;
            float target_yaw = degrees(_cam_yaw);
            do_gimbal_attitude_control(target_pitch, target_yaw);
            _gimbal_yaw_rate = 10.0f;
            if (have_target()) {
                set_state(Gimbal_State::Lock);
            }
        }
        break;
        case Gimbal_State::Lock:
        {
            if (have_target()) {
                float target_pitch = wrap_360(degrees(_cam_pitch) + bf_info.y);
                float target_yaw = wrap_360(degrees(_cam_yaw) + bf_info.x);
                do_gimbal_attitude_control(target_pitch, target_yaw);
            } else {
                set_state(Gimbal_State::Search);
            }
        }
        break;
    }

    update_gimbal_control();
    update_log();
}

void UGimbal::update_gimbal_control() {
    // send uart ;
    if (_Gimbal_ptr != nullptr) {
        _Gimbal_ptr->do_rate_control(_gimbal_pitch_rate, _gimbal_yaw_rate);
    }
}

void UGimbal::do_gimbal_attitude_control(float target_gimbal_pitch, float target_gimbal_yaw) {
    static uint32_t _last_control_ms = millis();
    float dt = (millis() - _last_control_ms);
    dt = dt * 0.001f;
    if (dt > 0.2f) {
        dt = 0.2f;
        lock_pitch_pid.reset_I();
        lock_yaw_pid.reset_I();
    }
    _last_control_ms = millis();

    update_gimbal_pitch_rate(target_gimbal_pitch, dt);
    update_gimbal_yaw_rate(target_gimbal_yaw, dt);
}

// degree/second
void UGimbal::update_gimbal_pitch_rate(float target_gimbal_pitch, float dt) {
    _gimbal_pitch_rate = degrees(lock_pitch_pid.update_all(radians(target_gimbal_pitch), _cam_pitch, dt));
}

// degree/second
void UGimbal::update_gimbal_yaw_rate(float target_gimbal_yaw, float dt) {
    _gimbal_yaw_rate = degrees(lock_yaw_pid.update_all(radians(target_gimbal_yaw), _cam_yaw, dt));
}

void UGimbal::handle_gimbal_msg(const mavlink_message_t &msg) {
    if (_Gimbal_ptr != nullptr) {
        _Gimbal_ptr->handle_msg(msg);
    }
}

void UGimbal::set_state(Gimbal_State state_in) {
    if (_state == state_in) {return;}
    _state = state_in;
    switch (_state) {
        default:
        case Gimbal_State::Ahead:
        {
            gcs().send_text(MAV_SEVERITY_INFO, "[Gimbal] Ahead");
        }
        break;
        case Gimbal_State::Search:
        {
            gcs().send_text(MAV_SEVERITY_INFO, "[Gimbal] Search");
        }
        break;
        case Gimbal_State::Lock:
        {
            gcs().send_text(MAV_SEVERITY_INFO, "[Gimbal] Lock");
        }
        break;
    }
}

// UDelay
void UGimbal::UDelay::init()
{
    _idx = 0;
    for (uint16_t i = 0; i < UDELAY_BUFFER; i++) {
        _buffer[i].roll = 0.0f;
        _buffer[i].pitch = 0.0f;
        _buffer[i].yaw = 0.0f;
        _buffer[i].time_ms = 0;
    }
}

void UGimbal::UDelay::push()
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

bool UGimbal::UDelay::get_idx(uint16_t step, float &roll, float &pitch, float &yaw) 
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
