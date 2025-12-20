#include "Copter.h"

const AP_Param::GroupInfo UAttack::var_info[] = {

    AP_SUBGROUPINFO(attack_velz_pid    , "VELZ_", 0, UAttack, AC_PID),
    AP_GROUPINFO("UPRINT",     1, UAttack, print,                   0),
    AP_GROUPINFO("TCAM_TYPE",  2, UAttack, use_target_cam_type,     0),
    AP_GROUPINFO("FILT_Y_HZ",  3, UAttack, filt_yaw_hz,             5.0f),
    AP_GROUPINFO("FILT_P_HZ",  4, UAttack, filt_pithc_hz,           5.0f),
    AP_GROUPINFO("GUN_PITCH",  5, UAttack, gun_pitch,               10.0f),
    AP_GROUPINFO("AIM_PITCH",  6, UAttack, aim_pitch,               10.0f),
    AP_GROUPINFO("AIM_YAW",    7, UAttack, aim_yaw,                 3.0f),

    AP_SUBGROUPPTR(_Target_ptr_cam_QD,   "TQD_",  8, UAttack,  FD_Target_QD),
    AP_GROUPEND
};

UAttack::UAttack()
{
    AP_Param::setup_object_defaults(this, var_info);

    // _last_yaw = 0.0f;
    // _last_yaw_sample = 0.0f;
}

// initialise
void UAttack::init()
{
    // udelay.init();
    _active = false;
    ef_cam_info.x = 0.0f;
    ef_cam_info.y = 0.0f;
    ef_gun_info.x = 0.0f;
    ef_gun_info.y = 0.0f;
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
    _target_vel_z = 0.0f;
    _Target_ptr_cam = nullptr;
    _Target_ptr_cam_QD = nullptr;
    _last_control_ms = millis();
    _last_reset_ms = 0;
    _last_log_ms = 0;
    _reset = true;
    _running = false;
    _yaw_off = 0.0f;
    init_target();

    // float sample_freq = 30.0f;
    _yaw_sample_filter.set_cutoff_frequency(filt_yaw_hz.get());
    _pitch_sample_filter.set_cutoff_frequency(filt_pithc_hz.get());
    _delta_yaw_filter.set_cutoff_frequency(filt_yaw_hz.get());
    gcs().send_text(MAV_SEVERITY_WARNING, "Target FILT HZ [%0.0f, %0.0f]", filt_yaw_hz.get(), filt_pithc_hz.get());
    _yaw_filter.init(30, 100);
    _pitch_filter.init(30, 100);
}

void UAttack::udpate_control_value(){
    update_target_vel_x();
    update_target_vel_y();
    update_target_vel_z();
    update_target_angle_yaw();

    _last_control_ms = millis();
}

void UAttack::update_log() {
    if (!is_active()) {return;}
    if (millis() - _last_log_ms < 100) {return;}
    _last_log_ms = millis();
    AP::logger().WriteStreaming("UTGT",
                                "TimeUS,efcx,efcy,efax,efay,rate,tyaw",
                                "s------",
                                "F------",
                                "Qffffff",
                                AP_HAL::micros64(),
                                (float)ef_cam_info.x,
                                (float)ef_cam_info.y,
                                (float)ef_gun_info.x,
                                (float)ef_gun_info.y,
                                (float)_running,
                                (float)display_info.count_log,
                                (float)_target_angle_yaw);


    AP::logger().WriteStreaming("UVEZ",
                                "TimeUS,target,actual,ff,P,I,D,srate,dmod",
                                "s--------",
                                "F--------",
                                "Qffffffff",
                                AP_HAL::micros64(),
                                (float)attack_velz_pid.get_pid_info().target,
                                (float)attack_velz_pid.get_pid_info().actual,
                                (float)attack_velz_pid.get_pid_info().FF,
                                (float)attack_velz_pid.get_pid_info().P,
                                (float)attack_velz_pid.get_pid_info().I,
                                (float)attack_velz_pid.get_pid_info().D,
                                (float)attack_velz_pid.get_pid_info().slew_rate,
                                (float)attack_velz_pid.get_pid_info().Dmod);

}

const Vector2f& UAttack::get_ef_cam_info() {
    return ef_cam_info;
}

const Vector2f& UAttack::get_ef_gun_info() {
    return ef_gun_info;
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
    copter.uattack.attack_velz_pid.reset_I();
    copter.uattack.attack_velz_pid.reset_filter();
    gcs().send_text(MAV_SEVERITY_INFO, "Track start");
    _last_control_ms = millis();
}

void UAttack::stop()
{
    _running = false;
}

void UAttack::reset()
{
    copter.uattack.attack_velz_pid.reset_I();
    copter.uattack.attack_velz_pid.reset_filter();
    gcs().send_text(MAV_SEVERITY_INFO, "Track reset");
    _last_control_ms = millis();
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
        _yaw_sample_filter.set_cutoff_frequency(filt_yaw_hz.get());
        _pitch_sample_filter.set_cutoff_frequency(filt_pithc_hz.get());
        _ef_rate_x_filter.set_cutoff_frequency(1.0f);
        _ef_rate_y_filter.set_cutoff_frequency(1.0f);
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
            _delta_yaw_filter.reset();
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
            udpate_control_value();
        }
    } else {
        _target_vel_x = 0.0f;
        _target_vel_y = 0.0f;
        _target_vel_z = 0.0f;
    }

}

void UAttack::handle_info(float p1, float p2) {

    if (_reset) {
        // _yaw_sample_filter.reset();
        // _pitch_sample_filter.reset();
        // _delta_yaw_filter.reset();
        _reset = false;
    }

    display_info.p3 = p1;
    display_info.p4 = p2;

    float _roll = AP::ahrs().get_roll();
    float _pitch = AP::ahrs().get_pitch();
    // float _yaw = AP::ahrs().get_yaw();

    _yaw_sample_filter.apply(p1, 0.03f);
    _pitch_sample_filter.apply(p2, 0.03f);

    ef_cam_info.x = _yaw_sample_filter.get(); // yaw degree
    ef_cam_info.y = _pitch_sample_filter.get(); // pitch degree

    if (p2 < -90.f) {
        p2 = -180.0f - p2;
    } else if (p2 > 90.0f) {
        p2 = 180.0f - p2;
    }

    // Vector3f cam_unit = Vector3f(1.0f, 0.0f, 0.0f);
    // Matrix3f tmp_target_earth_m;
    // tmp_target_earth_m.from_euler(0.0f, radians(p2), radians(p1));
    // Vector3f ef_cam_unit = tmp_target_earth_m*cam_unit;

    Vector3f gun_unit = Vector3f(1.0f, 0.0f, 0.0f);
    Matrix3f tmp_body_gun_m;
    tmp_body_gun_m.from_euler(0.0f, radians(gun_pitch.get()), 0.0f);
    Matrix3f tmp_earth_body_m;
    tmp_earth_body_m.from_euler(_roll, _pitch, 0.0f);
    Vector3f ef_gun_unit = tmp_earth_body_m*tmp_body_gun_m*gun_unit;

    float angle_pitch = wrap_180(degrees(atan2f(-ef_gun_unit.z, ef_gun_unit.xy().length())));
    float angle_yaw   = wrap_180(degrees(atan2f( ef_gun_unit.y, ef_gun_unit.x)));

    _yaw_filter.push(angle_yaw);
    _pitch_filter.push(angle_pitch);

    ef_gun_info.x = _yaw_filter.get();
    ef_gun_info.y = _pitch_filter.get();

    display_info.new_data = true;
    display_info.count++;
}

// m/s
void UAttack::update_target_vel_x() {
    ;
}

// m/s
void UAttack::update_target_vel_y() {
    ;
}

void UAttack::set_yaw_off(float yaw_off)
{
    _yaw_off = yaw_off;
    gcs().send_text(MAV_SEVERITY_INFO, "UATK: YAW off: %0.1f", _yaw_off);
}

// m/s
void UAttack::update_target_vel_z() {
    float dt = (millis() - _last_control_ms);
    dt = dt * 0.001f;
    if (dt > 1.0f) {attack_velz_pid.reset_I();}
    if (dt > 0.05f) {dt = 0.05f;}

    Vector3f vel_ned;
    if (copter.ahrs.get_velocity_NED(vel_ned)) {
        ;
    }

    float target_pitch = aim_pitch.get();
    float current_delta_pitch = wrap_180(ef_gun_info.y - ef_cam_info.y);

    float norm_in = 10.0f;

    _target_vel_z = attack_velz_pid.update_all(target_pitch/norm_in, current_delta_pitch/norm_in, dt);
}

// degree
void UAttack::update_target_angle_yaw() {
    // float dt = (millis() - _last_control_ms);
    // dt = dt * 0.001f;
    // if (dt > 0.05f) {dt = 0.05f;}

    _target_angle_yaw = wrap_360(_yaw_off + degrees(AP::ahrs().get_yaw()) + wrap_180(ef_cam_info.x - ef_gun_info.x));
}


void UAttack::handle_attack_msg(const mavlink_message_t &msg) {
    if (_Target_ptr_cam != nullptr) {
        _Target_ptr_cam->handle_msg(msg);
    }
}
