#include "FD_Target.h"

// Convenience macros //////////////////////////////////////////////////////////
//
const AP_Param::GroupInfo FD_Target_Loc::var_info[] = {

    AP_GROUPINFO("TOUT", 0, FD_Target_Loc, target_timeout, 0),
    AP_GROUPINFO("DOUT", 1, FD_Target_Loc, target_distout, 30),
    AP_GROUPINFO("NRAD", 2, FD_Target_Loc, nav_radius, 300),
    AP_GROUPINFO("ELOC", 3, FD_Target_Loc, use_external_loc, 0),

    AP_GROUPEND
};

FD_Target_Loc::FD_Target_Loc()
{
    AP_Param::setup_object_defaults(this, var_info);
    return;
}

bool FD_Target_Loc::init() {
    _valid = false;
    set_type(1);
    return true;
}

void FD_Target_Loc::update() {
    bool have_position = AP::ahrs().get_location(_current_loc);
    if (!have_position) {
        _valid = false;
        return;
    }

    if (_valid && (AP_HAL::millis() - _last_ms) > 16) {
        Vector3f off_ef = _current_loc.get_distance_NED(_target_loc);

        Matrix3f tmp_earth_yaw_m;
        tmp_earth_yaw_m.from_euler(radians(0.0f), radians(0.0f), AP::ahrs().get_yaw());
        // tmp_earth_yaw_m.from_euler(radians(0.0f), radians(0.0f), radians(0.0f));
        tmp_earth_yaw_m.transpose();
        Vector3f off_eyf = tmp_earth_yaw_m*off_ef;
        off_eyf.normalized();

        float p1 = degrees(wrap_180(atan2f( off_eyf.y, off_eyf.x))); // x-axis, degrees
        float p2 = degrees(wrap_180(atan2f(-off_eyf.z, off_eyf.xy().length()))); // y-axis, degrees

        handle_info(p1, p2);
    }

    uint32_t tnow = AP_HAL::millis(); // 只能放这里，handle_info会更新_last_ms的值，如果tnow赋值在其之前，则会小于_last_ms。SITL仿不出来，它周期是50Hz太低了
    if ((target_timeout.get() > 0) && (tnow - _last_target_ms > (uint32_t)target_timeout.get())) {
        _valid = false;
    }


    if (_valid && AP::ahrs().get_location(_current_loc) && (_current_loc.get_distance(_target_loc) < target_distout.get())) {
        _valid = false;
    }

    // for print purpose
    if (tnow - last_update_ms > 1000) {
        //gcs().send_text(MAV_SEVERITY_INFO, "raw: %d, att: %d, arspd: %d", pk0_count, pk1_count, pk2_count);
        last_update_ms = tnow;
    }

}

void FD_Target_Loc::handle_msg(const mavlink_message_t &msg)
{
    Location temp_loc;
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
        // decode packet
        mavlink_command_int_t packet;
        mavlink_msg_command_int_decode(&msg, &packet);
        switch(packet.command) {
            case MAV_CMD_USER_1:
                gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
                temp_loc.lat = packet.x;
                temp_loc.lng = packet.y;
                // temp_loc.set_alt_cm(packet.z*100.f, Location::AltFrame::ABOVE_HOME);
                // temp_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
                temp_loc.set_alt_cm(packet.z*100.f, Location::AltFrame::ABSOLUTE);
                set_target_loc(temp_loc);
                // gcs().send_text(MAV_SEVERITY_INFO,"x %f", (float)packet.x);
                // gcs().send_text(MAV_SEVERITY_INFO,"y %f", (float)packet.y);
                // gcs().send_text(MAV_SEVERITY_INFO,"z %f", (float)packet.z);
                break;
            default:
                break;
        }
    }

    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(&msg, &packet);
        switch(packet.command) {
            case MAV_CMD_USER_1: {
                    if ((int16_t)packet.param1 == 150 && (int16_t)packet.param2 == 1079 && (int16_t)packet.param3 == 1500 )
                    {
                        gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
                    }
                    temp_loc.lat = 399778929;
                    temp_loc.lng = 1163409769;
                    // temp_loc.set_alt_cm(packet.z*100.f, Location::AltFrame::ABOVE_HOME);
                    // temp_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
                    temp_loc.set_alt_cm(5300.f, Location::AltFrame::ABSOLUTE);
                    set_target_loc(temp_loc);
                    // gcs().send_text(MAV_SEVERITY_INFO,"x %f", (float)packet.x);
                    // gcs().send_text(MAV_SEVERITY_INFO,"y %f", (float)packet.y);
                }
                // gcs().send_text(MAV_SEVERITY_INFO,"z %f", (float)packet.z);
                break;
            default:
                break;
        }
    }
}

void FD_Target_Loc::set_target_loc(Location &loc_in)
{
    _target_loc = loc_in;
    _valid = true;
    _last_target_ms = AP_HAL::millis();
}

Location& FD_Target_Loc::get_target_loc() {
    return _target_loc;
}


void FD_Target_Loc::handle_info_test(float p1, float p2) {
    // FD_DYT_NEW_msg_DYTTELEM &tmp_msg = FD1_uart_ptr->get_msg_DYTTELEM();
    // tmp_msg._msg_1.updated = true;
    // tmp_msg._msg_1.content.msg.target_x = (int16_t)(p1/0.005f);
    // tmp_msg._msg_1.content.msg.target_y = (int16_t)(p2/0.005f);
}
