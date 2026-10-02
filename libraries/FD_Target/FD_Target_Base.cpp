#include "FD_Target.h"

#include <AP_AHRS/AP_AHRS.h>

FD_Target_Base::FD_Target_Base() :
    _last_ms(0),
    _new_data(false),
    _valid(false),
    _p1(0.0f),
    _p2(0.0f),
    _has_rate(false),
    _new_rate(false),
    _r1(0.0f),
    _r2(0.0f),
    _fps(0.0f),
    _fps_last_ms(0),
    _type(0)
{
}

// 目标源直接给出 LOS 角速率（不再对角度做差分）
void FD_Target_Base::handle_info_rate(float r1, float r2)
{
    _has_rate = true;
    _r1 = r1;
    _r2 = r2;
    _new_rate = true;
}

bool FD_Target_Base::get_info_rate(float &r1, float &r2)
{
    if (!_has_rate || !_new_rate) {
        return false;
    }
    _new_rate = false;
    r1 = _r1;
    r2 = _r2;
    return true;
}

void FD_Target_Base::handle_info(float p1, float p2)
{
    _valid = true;
    _last_ms = AP_HAL::millis();

    // 统计有效数据到达帧率（一阶低通平滑）
    if (_fps_last_ms != 0) {
        const float dt_ms = (float)(_last_ms - _fps_last_ms);
        if (dt_ms > 0.0f) {
            const float inst_fps = 1000.0f / dt_ms;
            _fps = _fps * 0.8f + inst_fps * 0.2f;
        }
    }
    _fps_last_ms = _last_ms;

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
        // HUOJIAN: 变量名按矩阵转置后的实际作用命名（A_from_B = 把 B 系向量转成 A 系）
        Matrix3f yaw_frame_from_target;
        yaw_frame_from_target.from_euler(0.0f, radians(_p2), radians(_p1));
        Matrix3f body_from_yaw_frame;
        body_from_yaw_frame.from_euler(AP::ahrs().get_roll_rad(), AP::ahrs().get_pitch_rad(), 0.0f);
        body_from_yaw_frame.transpose();
        Vector3f target_body = body_from_yaw_frame * yaw_frame_from_target * target_axis;
        target_body.normalize();
        p1 = degrees(wrap_180(atan2f(target_body.y, target_body.x)));
        p2 = degrees(wrap_180(atan2f(-target_body.z, target_body.xy().length())));
        return true;
    }

    if (_type == 2) {
        const Vector3f target_axis(1.0f, 0.0f, 0.0f);
        // HUOJIAN: 相机安装 pitch+90（光轴沿机体 -Z）；转置后 camera_from_body 才是"机体系→相机系"
        Matrix3f yaw_frame_from_target;
        yaw_frame_from_target.from_euler(0.0f, radians(_p2), radians(_p1));
        Matrix3f body_from_yaw_frame;
        body_from_yaw_frame.from_euler(AP::ahrs().get_roll_rad(), AP::ahrs().get_pitch_rad(), 0.0f);
        body_from_yaw_frame.transpose();
        Matrix3f camera_from_body;
        camera_from_body.from_euler(radians(0.0f), radians(90.0f), radians(0.0f));
        camera_from_body.transpose();
        Vector3f target_camera = camera_from_body * body_from_yaw_frame * yaw_frame_from_target * target_axis;
        target_camera.normalize();
        p1 = degrees(wrap_180(atan2f(target_camera.y, target_camera.x)));
        p2 = degrees(wrap_180(atan2f(-target_camera.z, target_camera.xy().length())));
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
