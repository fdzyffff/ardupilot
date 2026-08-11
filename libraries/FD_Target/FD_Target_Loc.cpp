#include "FD_Target_Loc.h"

#include <AP_AHRS/AP_AHRS.h>

const AP_Param::GroupInfo FD_Target_Loc::var_info[] = {
    AP_GROUPINFO("TOUT", 0, FD_Target_Loc, target_timeout, 0),
    AP_GROUPINFO("DOUT", 1, FD_Target_Loc, target_distout, 30),
    AP_GROUPINFO("NRAD", 2, FD_Target_Loc, nav_radius, 300),
    AP_GROUPINFO("ELOC", 3, FD_Target_Loc, use_external_loc, 0),
    AP_GROUPEND
};

FD_Target_Loc::FD_Target_Loc() :
    _last_target_ms(0),
    last_update_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool FD_Target_Loc::init()
{
    _valid = false;
    set_type(2);
    return true;
}

void FD_Target_Loc::update()
{
    bool have_position = AP::ahrs().get_location(_current_loc);
    if (!have_position) {
        _valid = false;
        return;
    }

    if (_valid && (AP_HAL::millis() - _last_ms) > 16) {
        Vector3f off_ef = _current_loc.get_distance_NED(_target_loc);

        off_ef.normalized();
        Matrix3f tmp_earth_yaw_m;
        tmp_earth_yaw_m.from_euler(radians(0.0f), radians(0.0f), AP::ahrs().get_yaw_rad());
        tmp_earth_yaw_m.transpose();
        Vector3f off_eyf = tmp_earth_yaw_m * off_ef;
        off_eyf.normalized();

        float p1 = degrees(wrap_180(atan2f(off_eyf.y, off_eyf.x)));
        float p2 = degrees(wrap_180(atan2f(-off_eyf.z, off_eyf.xy().length())));
        handle_info(p1, p2);
    }

    uint32_t now_ms = AP_HAL::millis();
    if ((target_timeout.get() > 0) &&
        (now_ms - _last_target_ms > (uint32_t)target_timeout.get())) {
        _valid = false;
    }

    if (_valid && AP::ahrs().get_location(_current_loc) &&
        (_current_loc.get_distance(_target_loc) < target_distout.get())) {
        _valid = false;
    }

    if (now_ms - last_update_ms > 1000) {
        last_update_ms = now_ms;
    }
}

void FD_Target_Loc::handle_msg(const mavlink_message_t &msg)
{
    Location temp_loc;
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
        mavlink_command_int_t packet;
        mavlink_msg_command_int_decode(&msg, &packet);
        if (packet.command == MAV_CMD_USER_1) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
            temp_loc.lat = packet.x;
            temp_loc.lng = packet.y;
            temp_loc.set_alt_cm(packet.z * 100.0f, Location::AltFrame::ABSOLUTE);
            set_target_loc(temp_loc);
        }
    }

    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(&msg, &packet);
        if (packet.command == MAV_CMD_USER_1) {
            if (((int16_t)packet.param1 == 150) &&
                ((int16_t)packet.param2 == 1079) &&
                ((int16_t)packet.param3 == 1500)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
            }
            temp_loc.lat = -353633135;
            temp_loc.lng = 1491670418;
            temp_loc.set_alt_cm(58300.0f, Location::AltFrame::ABSOLUTE);
            set_target_loc(temp_loc);
        }
    }
}

void FD_Target_Loc::set_target_loc(Location &loc_in)
{
    _target_loc = loc_in;
    _valid = true;
    _last_target_ms = AP_HAL::millis();
}

Location &FD_Target_Loc::get_target_loc()
{
    return _target_loc;
}

void FD_Target_Loc::handle_info_test(float p1, float p2)
{
}
