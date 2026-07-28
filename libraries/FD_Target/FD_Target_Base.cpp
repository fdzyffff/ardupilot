#include "FD_Target.h"

#include <AP_AHRS/AP_AHRS.h>

FD_Target_Base::FD_Target_Base() :
    _last_ms(0),
    _new_data(false),
    _valid(false),
    _p1(0.0f),
    _p2(0.0f),
    _type(0)
{
}

void FD_Target_Base::handle_info(float p1, float p2)
{
    _valid = true;
    _last_ms = AP_HAL::millis();
    _p1 = p1;
    _p2 = p2;
    _new_data = true;
}

bool FD_Target_Base::get_info(float &p1, float &p2)
{
    if (!_new_data) {
        return false;
    }

    _new_data = false;
    if (_type == 0) {
        p1 = _p1;
        p2 = _p2;
        return true;
    }

    if (_type == 1) {
        const Vector3f target_axis(1.0f, 0.0f, 0.0f);
        Matrix3f target_from_yaw_frame;
        target_from_yaw_frame.from_euler(0.0f, radians(_p2), radians(_p1));
        Matrix3f yaw_frame_from_body;
        yaw_frame_from_body.from_euler(AP::ahrs().get_roll(), AP::ahrs().get_pitch(), 0.0f);
        yaw_frame_from_body.transpose();
        Vector3f target_body = yaw_frame_from_body * target_from_yaw_frame * target_axis;
        target_body.normalize();
        p1 = degrees(wrap_180(atan2f(target_body.y, target_body.x)));
        p2 = degrees(wrap_180(atan2f(-target_body.z, target_body.xy().length())));
        return true;
    }

    if (_type == 2) {
        const Vector3f target_axis(1.0f, 0.0f, 0.0f);
        Matrix3f target_from_yaw_frame;
        target_from_yaw_frame.from_euler(0.0f, radians(_p2), radians(_p1));
        Matrix3f yaw_frame_from_body;
        yaw_frame_from_body.from_euler(AP::ahrs().get_roll(), AP::ahrs().get_pitch(), 0.0f);
        yaw_frame_from_body.transpose();
        Matrix3f body_from_camera;
        body_from_camera.from_euler(radians(0.0f), radians(90.0f), radians(0.0f));
        body_from_camera.transpose();
        Vector3f target_body = body_from_camera * yaw_frame_from_body * target_from_yaw_frame * target_axis;
        target_body.normalize();
        p1 = degrees(wrap_180(atan2f(target_body.y, target_body.x)));
        p2 = degrees(wrap_180(atan2f(-target_body.z, target_body.xy().length())));
        return true;
    }

    return false;
}

void FD_Target_Base::recover_info()
{
    _new_data = true;
}

uint8_t FD_Target_Base::get_type()
{
    return _type;
}

void FD_Target_Base::set_type(uint8_t type_in)
{
    _type = type_in;
}

void FD_Target_Base::set_valid(bool valid_in)
{
    _valid = valid_in;
}
