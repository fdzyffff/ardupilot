#include "Plane.h"
//#include "UCam.h"

UAttack::UCam()
{
    ;
}

// initialise
void UAttack::init()
{
    _active = false;
    _cam_state = 0;
    raw_info.x = 0.0f;
    raw_info.y = 0.0f;
    correct_info.x = 0.0f;
    correct_info.y = 0.0f;
    display_info_new = false;
    display_info_p1 = 0.0f;
    display_info_p2 = 0.0f;
    display_info_p3 = 0.0f;
    display_info_p4 = 0.0f;
    display_info_count = 0;
    _target_pitch_rate_cds = 0.0f;
    _target_yaw_rate_cds = 0.0f;
    _target_roll_angle_cd = 0.0f;
    _port = NULL;
    init_cam_port();
}



void UAttack::udpate_value(){
    // if (copter.g2.user_parameters.cam_pixel_x.get() < 50.f) {copter.g2.user_parameters.cam_pixel_x.set_and_save(50.f);}
    // if (copter.g2.user_parameters.cam_pixel_y.get() < 50.f) {copter.g2.user_parameters.cam_pixel_y.set_and_save(50.f);}
    // if (copter.g2.user_parameters.cam_angle_x.get() < 30.f) {copter.g2.user_parameters.cam_angle_x.set_and_save(30.f);}
    // if (copter.g2.user_parameters.cam_angle_y.get() < 30.f) {copter.g2.user_parameters.cam_angle_y.set_and_save(30.f);}
    // if (copter.g2.user_parameters.fly_yaw_tc.get() < 0.1f) {copter.g2.user_parameters.fly_yaw_tc.set_and_save(0.1f);}
    update_target_pitch_rate_cds();
    update_target_yaw_rate_cd();
    update_target_roll_angle_cd();
}

const Vector2f& UAttack::get_raw_info() {
    return raw_info;
}

const Vector2f& UAttack::get_correct_info() {
    return correct_info;
}


void UAttack::init_cam_port()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    _cam_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_CamMavlink, 0);
    if (_cam_port != nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "CAM Uart init");
        _cam_port = new UCam_Port_Mavlink(*this, _cam_port);
        _port_type = 1;
        return;
    }

    _cam_port_type = 0;
    gcs().send_text(MAV_SEVERITY_WARNING, "CAM init FAILED");
}

void UAttack::cam_update()
{
    if (get_port() == NULL) {return;}
    if (_Ucam_port == nullptr) {return;}
    _Ucam_port->update();
}


void UAttack::do_cmd(float p1, float p2, float p3, float p4)
{
    if (get_port() == NULL) {return;}
    if (_Ucam_port == nullptr) {return;}
    _Ucam_port->do_cmd(p1, p2, p3, p4);
}


// update
void UAttack::update()
{
    cam_update();
    time_out_check();
}

void UAttack::time_out_check() {
    if (!_UCam_ptr->valid()) { 
        _target_pitch_rate_cds = 0.0f;
        _target_roll_angle_cd = 0.0f;
        _target_yaw_rate_cds = 0.0f;
        _active = false;
        return;
    } else {
        _active = true;
    }
}

void UAttack::update_target_pitch_rate_cds() {
    _target_pitch_rate_cds = 1.0f * correct_info.y;
    // gcs().send_text(MAV_SEVERITY_INFO, "%f", _target_pitch_rate_cds);
}

void UAttack::update_target_roll_angle_cd() {
    _target_roll_angle_cd = 0.0f;
}

void UAttack::update_target_yaw_rate_cd() {
    _target_yaw_rate_cds = 1.0f * correct_info.x;
}
