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
    update_collision();
}

void Plane::userhook_1Hz()
{
    uattack.do_print();
    uart.do_print();
    update_ls_status();

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
        AP::logger().WriteStreaming("LCMD",
                                    "TimeUS,Type,Flag,P1,Rad,Yaw,Lat,Lng,Alt",
                                    "s----dDUm",
                                    "F----BUUB",
                                    "QBBBhiiii",
                                    AP_HAL::micros64(),
                                    packet.type,
                                    packet.flag,
                                    packet.p1,
                                    packet.radius,
                                    packet.yaw_cd,
                                    packet.lat,
                                    packet.lng,
                                    packet.alt);
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
                        if (quadplane.ahrs_view == nullptr) {
                            target_yaw_cd = degrees(AP::ahrs().get_yaw()) * 100;
                        } else {
                            target_yaw_cd = (float)quadplane.ahrs_view->yaw_sensor;
                        }
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
                        if (quadplane.ahrs_view == nullptr) {
                            target_yaw_cd = degrees(AP::ahrs().get_yaw()) * 100;
                        } else {
                            target_yaw_cd = (float)quadplane.ahrs_view->yaw_sensor;
                        }
                    }

                    float target_alt_m = 2.5f;
                    if (use_alt) {
                        target_alt_m = (float)packet.alt * 0.01f;
                    }

                    mode_qguided.do_takeoff(target_alt_m, target_yaw_cd);
                }
            }
            break;
        case 4:
            {
                if (!use_latlng) {
                    gcs().send_text(MAV_SEVERITY_WARNING, "RTL Loc rejected: no lat/lng");
                    break;
                }

                Location target_loc{plane.current_loc};
                target_loc.lat = packet.lat;
                target_loc.lng = packet.lng;
                if (!target_loc.check_latlng()) {
                    gcs().send_text(MAV_SEVERITY_WARNING, "RTL Loc rejected: invalid lat/lng");
                    break;
                }

                if (!AP::ahrs().home_is_set()) {
                    gcs().send_text(MAV_SEVERITY_WARNING, "RTL Loc rejected: home not set");
                    break;
                }

                int32_t target_alt_cm = plane.home.alt;
                // if (use_alt) {
                //     const int64_t alt_error_cm = (int64_t)packet.alt - (int64_t)plane.home.alt;
                //     if (llabs(alt_error_cm) <= 5000) {
                //         target_alt_cm = packet.alt;
                //     } else {
                //         gcs().send_text(MAV_SEVERITY_WARNING, "RTL Alt Partly received: use home alt");
                //     }
                // }
                target_loc.set_alt_cm(target_alt_cm, Location::AltFrame::ABSOLUTE);

                gcs().send_text(MAV_SEVERITY_WARNING, "New Return Loc");
                gcs().send_text(MAV_SEVERITY_WARNING, "|- %d, %d, %d", int(target_loc.lat), int(target_loc.lng), int(target_loc.alt));
                plane.mode_rtl.set_return_loc(target_loc);
                plane.mode_qrtl.set_return_loc(target_loc);
                
                // if (plane.set_mode(mode_rtl, ModeReason::GCS_COMMAND)) {
                //     ;
                // }
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
            gcs().send_text(MAV_SEVERITY_WARNING, "Unknow ls cmd [%d]", packet.type);
        }
    }
}

void Plane::update_collision() 
{
    if (is_flying() && ins.get_accel_peak_hold_neg_x() > 20 && control_mode == &mode_external && mode_external.is_angle_mode()) {
        if (!collision_triggered) {
            gcs().send_text(MAV_SEVERITY_INFO, "Collision %.0f",ins.get_accel_peak_hold_neg_x());
        }
        collision_triggered = true;
        collision_trigger_ms = millis();
    } else {
        if (millis() - collision_trigger_ms > 5000) {
            collision_triggered = false;
        }
    }
}

void Plane::update_ls_status()
{
    gcs().send_message(MSG_LS_STATUS);
}

void Plane::send_ls_status(mavlink_channel_t chan)
{
    uint8_t status = 0;// 1: mc mode, 2: fw mode, 3: takeoff, 4: return, 5: land, 6: external control
    if (control_mode == &mode_qguided) {
        status = 1;
        if (mode_qguided.is_takeoff) {
            status = 3;
        }
    }
    if (control_mode == &mode_guided || (control_mode == &mode_external && !mode_external.is_angle_mode())) {
        status = 2;
    }
    if (control_mode == &mode_rtl || control_mode == &mode_qrtl) {
        status = 4;
    }
    if (control_mode == &mode_qland) {
        status = 5;
    }
    if (control_mode == &mode_external && mode_external.is_angle_mode()) {
        status = 6;
    }
    mavlink_msg_ls_status_send(
        chan,
        status,
        collision_triggered);
}
