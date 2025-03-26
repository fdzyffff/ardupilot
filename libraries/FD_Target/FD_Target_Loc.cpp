#include "FD_Target.h"

// Convenience macros //////////////////////////////////////////////////////////
//
const AP_Param::GroupInfo FD_Target_Loc::var_info[] = {

    AP_GROUPINFO("TOUT", 0, FD_Target_Loc, target_timeout, 0),

    AP_GROUPEND
};

FD_Target_Loc::FD_Target_Loc()
{
    AP_Param::setup_object_defaults(this, var_info);
    return;
}

bool FD_Target_Loc::init() {
    _have_target = false;
    return true;
}

void FD_Target_Loc::update() {
    static uint32_t last_update_ms = millis();

    if (_have_target && (millis() - _last_ms) > 33) {
        bool have_position = AP::ahrs().get_location(current_loc);
        if (!have_position) {
            return;
        }
        Vector3f off_ef = current_loc.get_distance_NED(target_loc);

        Matrix3f tmp_earth_m;
        tmp_earth_m.from_euler(AP::ahrs().get_roll(), AP::ahrs().get_pitch(), AP::ahrs().get_yaw());
        tmp_earth_m.transpose();
        Vector3f off_bf = tmp_earth_m*off_ef;
        off_bf.normalized();

        float p1 = degrees(wrap_180(atan2f( off_bf.y, off_bf.x))); // x-axis, degrees
        float p2 = degrees(wrap_180(atan2f(-off_bf.z, off_bf.x))); // y-axis, degrees

        handle_info(p1, p2);
    }

    uint32_t tnow = millis(); // 只能放这里，handle_info会更新_last_ms的值，如果tnow赋值在其之前，则会小于_last_ms。SITL仿不出来，它周期是50Hz太低了
    if ((target_timeout > 0) && (tnow - _last_ms > (uint32_t)target_timeout)) {
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
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
        // decode packet
        // decode packet
        mavlink_command_int_t packet;
        mavlink_msg_command_int_decode(&msg, &packet);
        switch(packet.command) {
            case MAV_CMD_USER_1:
                gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
                target_loc.lat = packet.x;
                target_loc.lng = packet.y;
                // target_loc.set_alt_cm(packet.z*100.f, Location::AltFrame::ABOVE_HOME);
                // target_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
                target_loc.set_alt_cm(packet.z*100.f, Location::AltFrame::ABSOLUTE);
                _have_target = true;
                // gcs().send_text(MAV_SEVERITY_INFO,"x %f", (float)packet.x);
                // gcs().send_text(MAV_SEVERITY_INFO,"y %f", (float)packet.y);
                // gcs().send_text(MAV_SEVERITY_INFO,"z %f", (float)packet.z);
                break;
            default:
                break;
        }
    }

}

void FD_Target_Loc::handle_info_test(float p1, float p2) {
    // FD_DYT_NEW_msg_DYTTELEM &tmp_msg = FD1_uart_ptr->get_msg_DYTTELEM();
    // tmp_msg._msg_1.updated = true;
    // tmp_msg._msg_1.content.msg.target_x = (int16_t)(p1/0.005f);
    // tmp_msg._msg_1.content.msg.target_y = (int16_t)(p2/0.005f);
}
