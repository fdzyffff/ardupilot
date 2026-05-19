#include "Plane.h"

void Plane::userhook_init()
{
    uart.init();
    uattack.init();
    udelay.init();
}

void Plane::userhook_100Hz()
{
    uattack.update();
    uart.update();
    udelay.push();
}

void Plane::userhook_1Hz()
{
    uattack.do_print();
    uart.do_print();

    // static uint8_t tt = 0;
    // if (uart.get_port() != nullptr) {
    //     // get_port()->write(uart_msg_LS_status._msg_1.content.data, sizeof(uart_msg_LS_status._msg_1.content.data));
    //     uart.get_port()->write(tt++);
    //     gcs().send_text(MAV_SEVERITY_WARNING, "Uart send");
    // }
    
}

// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Plane::position_ok() const
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status;
    if (AP::ahrs().get_filter_status(filt_status)) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    }
    return false;
}

void Plane::user_handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_LS_CMD) {
        mavlink_ls_cmd_t packet;
        mavlink_msg_ls_cmd_decode(&msg, &packet);
        bool use_alt = packet.flag & (1<<0);
        bool use_latlng = packet.flag & (1<<1);
        bool use_yaw = packet.flag & (1<<2);
        // bool use_radius = packet.flag & (1<<3);
        switch(packet.type) {
        case 1:
            {
                if (plane.set_mode(mode_qguided, ModeReason::GCS_COMMAND)) {
                    Location target_loc{plane.current_loc};
                    if (is_flying()) {
                        target_loc.offset_bearing(degrees(ahrs.groundspeed_vector().angle()), plane.quadplane.stopping_distance());
                    }
                    float target_yaw_cd = 0.0f;
                    if (use_yaw) {
                        target_yaw_cd = packet.yaw_cd;
                    } else {
                        target_yaw_cd = degrees(AP::ahrs().get_yaw()) * 100;
                    }

                    if (use_alt && is_flying()) {
                        target_loc.alt = packet.alt;
                    }
                    if (use_latlng && is_flying()) {
                        target_loc.lat = packet.lat;
                        target_loc.lng = packet.lng;
                    }
                    mode_qguided.do_guide(target_loc, target_yaw_cd);
                }
            }
            break;
        case 2:
            {
                if (plane.set_mode(mode_guided, ModeReason::GCS_COMMAND)) {
                    Location target_loc{plane.current_loc};
                    if (use_alt) {
                        target_loc.alt = packet.alt;
                    }
                    if (use_latlng) {
                        target_loc.lat = packet.lat;
                        target_loc.lng = packet.lng;
                    }
                    set_guided_WP(target_loc);
                }
            }
            break;
        case 3:
            {
                if (plane.set_mode(mode_qguided, ModeReason::GCS_COMMAND)) {
                    float target_yaw_cd = 0.0f;
                    if (use_yaw) {
                        target_yaw_cd = packet.yaw_cd;
                    } else {
                        target_yaw_cd = degrees(AP::ahrs().get_yaw()) * 100;
                    }

                    float target_alt_m = 1.0f;
                    if (use_alt) {
                        target_alt_m = (float)packet.alt * 0.01f;
                    }

                    mode_qguided.do_takeoff(target_alt_m, target_yaw_cd);
                }
            }
            break;
        case 4:
            {
                if (plane.set_mode(mode_rtl, ModeReason::GCS_COMMAND)) {
                    Location target_loc{plane.next_WP_loc};
                    if (use_latlng) {
                        plane.next_WP_loc.lat = packet.lat;
                        plane.next_WP_loc.lng = packet.lng;
                    }
                }
            }
            break;
        case 5:
            {
                if (plane.set_mode(mode_qland, ModeReason::GCS_COMMAND)) {
                    ;
                }
            }
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "Unknow ls cmd [%d]", packet.type);
        }
    }
}