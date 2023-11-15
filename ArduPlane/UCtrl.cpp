#include "Plane.h"

void UCtrl::init() {
    uart_msg_ctrl.init();
    
    uart_msg_ctrl.get_msg_init().set_enable();
    uart_msg_ctrl.get_msg_control().set_enable();
    uart_msg_ctrl.get_msg_mission().set_enable();
    uart_msg_ctrl.get_msg_guide().set_enable();
    uart_msg_ctrl.get_msg_info().set_enable();

    uart_msg_ctrl_send.init();
    uart_msg_ctrl_send.get_msg_info().set_enable();

    _valid = false;
    _last_msg_update_ms = 0;
    _target_loc.lat = plane.current_loc.lat;
    _target_loc.lng = plane.current_loc.lng;
    _target_loc.alt = plane.target_altitude.amsl_cm;
}


void UCtrl::update() {

    while (uart_msg_ctrl.port_avaliable() > 0) {
        uart_msg_ctrl.read();
        msg_handle_init(); 
        msg_handle_control(); 
        msg_handle_mission();
        msg_handle_guide(); 
    }

    ctrl_send();

    uart_msg_ctrl_send.write();

    update_valid();
}

void UCtrl::msg_handle_init() {
    static uint32_t _last_print_ms = 0;
    if (uart_msg_ctrl.get_msg_init()._msg_1.updated) {
        if (millis() - _last_print_ms > 3000) {
            gcs().send_text(MAV_SEVERITY_INFO, "INFO: init");
            _last_print_ms = millis();
        }
        uart_msg_ctrl.get_msg_init()._msg_1.updated = false;
    }
}

void UCtrl::msg_handle_control() {
    static uint32_t _last_print_ms = 0;
    FD1_msg_control &tmp_msg = uart_msg_ctrl.get_msg_control();
    if (tmp_msg._msg_1.updated) {
        if (tmp_msg._msg_1.content.msg.device_id != plane.g.sysid_this_mav) {
            tmp_msg._msg_1.updated = false;
            return;
        }
        if (millis() - _last_print_ms > 3000) {
        gcs().send_text(MAV_SEVERITY_INFO, "INFO: control");
            _last_print_ms = millis();
        }
        tmp_msg._msg_1.updated = false;
    } else {
        return;
    }
    bool do_print = false;
    bool do_print2 = false;
    switch (tmp_msg._msg_1.content.msg.cmd1) {
        case 0x01: //自主控制
            plane.set_mode(plane.mode_auto, ModeReason::GCS_COMMAND);
            break;
        case 0x02: //人工操纵
            plane.set_mode(plane.mode_fbwa, ModeReason::GCS_COMMAND);
            break;
        case 0x03: //增稳飞行
            plane.set_mode(plane.mode_stabilize, ModeReason::GCS_COMMAND);
            break;
        case 0x04: //姿态保持
            plane.set_mode(plane.mode_acro, ModeReason::GCS_COMMAND);
            break;
        case 0x05: //人工修正
            do_print = true;
            break;
        case 0x06: //左盘旋
            do_loiter_left();
            break;
        case 0x07: //右盘旋
            do_loiter_right();
            break;
        case 0x08: //起飞
            plane.set_mode(plane.mode_takeoff, ModeReason::GCS_COMMAND);
            break;
        case 0x09: //返航
            plane.set_mode(plane.mode_rtl, ModeReason::GCS_COMMAND);
            break;
        case 0x0A: //着舰模式
            do_print = true;
            break;
        case 0x0B: //复飞
            do_print = true;
            break;
        case 0x0C: //正向着陆
            do_print = true;
            break;
        case 0x0D: //反向着陆
            do_print = true;
            break;
        case 0x0E: //发动机停车
            do_print = true;
            break;
        case 0x0F: //爬升开
            do_cruise_up();
            do_print = true;
            break;
        case 0x10: //爬升关
            do_cruise_alt((float)plane.current_loc.alt * 0.01f);
            do_print = true;
            break;
        case 0x30: //刹车
            do_print = true;
            break;
        case 0x31: //松刹车
            do_print = true;
            break;
        case 0x32: //收起落架
            do_print = true;
            break;
        case 0x33: //放起落架
            do_print = true;
            break;
        case 0x35: //桨距增
            do_print = true;
            break;
        case 0x39: //桨距减
            do_print = true;
            break;
        case 0x3A: //释放控制权
            do_print = true;
            break;
        case 0x3B: //接管控制权
            do_print = true;
            break;
        case 0x3C: //解锁
            plane.arming.arm(AP_Arming::Method::MAVLINK);
            do_print = true;
            break;
        case 0x3E: //弹射
            do_print = true;
            break;

        default:
            break;
    }

    if (tmp_msg._msg_1.content.msg.cmd1 == 0x00) {
        switch (tmp_msg._msg_1.content.msg.cmd2) {
            case 0x01: //高度保持
                do_cruise_alt(tmp_msg._msg_1.content.msg.content2);
                do_print2 = true;
                break;
            case 0x02: //速度保持
                do_cruise_speed(tmp_msg._msg_1.content.msg.content2);
                do_print2 = true;
                break;
            case 0x03: //航点切换
                do_print = true;
                break;
            case 0x04: //盘旋半径装订
                do_print = true;
                break;
            case 0x05: //飞机重量装订
                do_print = true;
                break;
            case 0x06: //机场气压高度装订
                do_print = true;
                break;
            default:
                break;
        }
    }

    float temp_pitch = ((float)tmp_msg._msg_1.content.msg.ctrl_pitch) /65.535f; 
    float temp_roll = ((float)tmp_msg._msg_1.content.msg.ctrl_roll) /65.535f; 
    float temp_throttle = ((float)tmp_msg._msg_1.content.msg.ctrl_throttle) /65.535f; 
    float temp_yaw = ((float)tmp_msg._msg_1.content.msg.ctrl_yaw) /65.535f; 

    uint32_t tnow = millis();
    RC_Channels::set_override(0, constrain_int16(1500 + (int16_t)temp_pitch,1000,2000), tnow);      // pitch
    RC_Channels::set_override(1, constrain_int16(1500 + (int16_t)temp_roll,1000,2000), tnow);       // roll
    RC_Channels::set_override(2, constrain_int16(1000 + (int16_t)temp_throttle,1000,2000), tnow);   // throttle
    RC_Channels::set_override(3, constrain_int16(1500 + (int16_t)temp_yaw,1000,2000), tnow);        // yaw

    if (do_print) {
        gcs().send_text(MAV_SEVERITY_INFO, "cmd1:%x,cmd2:%x",tmp_msg._msg_1.content.msg.cmd1,tmp_msg._msg_1.content.msg.cmd2);
    }
    if (do_print2) {
        gcs().send_text(MAV_SEVERITY_INFO, "cmd1:%x,cmd2:%x,cont:%f",tmp_msg._msg_1.content.msg.cmd1,tmp_msg._msg_1.content.msg.cmd2,tmp_msg._msg_1.content.msg.content2);
    }
}

void UCtrl::msg_handle_mission() {
    static uint32_t _last_print_ms = 0;
    FD1_msg_mission &tmp_msg = uart_msg_ctrl.get_msg_mission();
    if (tmp_msg._msg_1.updated) {
        if (tmp_msg._msg_1.content.msg.device_id != plane.g.sysid_this_mav) {
            tmp_msg._msg_1.updated = false;
            return;
        }
        if (millis() - _last_print_ms > 3000) {
            gcs().send_text(MAV_SEVERITY_INFO, "INFO: mission");
            _last_print_ms = millis();
        }

        gcs().send_text(MAV_SEVERITY_INFO, "wp_id: %d", tmp_msg._msg_1.content.msg.wp_id);
        gcs().send_text(MAV_SEVERITY_INFO, "lat: %f", tmp_msg._msg_1.content.msg.lat);
        gcs().send_text(MAV_SEVERITY_INFO, "lng: %f", tmp_msg._msg_1.content.msg.lng);
        gcs().send_text(MAV_SEVERITY_INFO, "alt: %f", tmp_msg._msg_1.content.msg.alt);
        // gcs().send_text(MAV_SEVERITY_INFO, "A plane.mission.num_commands(): %d", plane.mission.num_commands());
        // if (plane.mission.num_commands() == 0) {
        //     plane.mission.write_home_to_storage();
        // }
        // gcs().send_text(MAV_SEVERITY_INFO, "plane.mission.num_commands(): %d", plane.mission.num_commands());

        if ( (plane.mission.num_commands() != tmp_msg._msg_1.content.msg.wp_id) 
            && (plane.mission.num_commands() != 0) ) {
            gcs().send_text(MAV_SEVERITY_INFO, "INFO: clean current mission, (%d / %d)", tmp_msg._msg_1.content.msg.wp_id, plane.mission.num_commands());
            plane.mission.clear();
            // plane.mission.write_home_to_storage();
        }

        if (plane.mission.num_commands() == 0) {
            AP_Mission::Mission_Command tmp_cmd;
            tmp_cmd.id = MAV_CMD_NAV_TAKEOFF;
            tmp_cmd.content.location.lat = (int32_t)(tmp_msg._msg_1.content.msg.lat*1.0e7);
            tmp_cmd.content.location.lng = (int32_t)(tmp_msg._msg_1.content.msg.lng*1.0e7);
            tmp_cmd.content.location.set_alt_cm(100, Location::AltFrame::ABOVE_HOME);
            plane.mission.add_cmd(tmp_cmd);
            // plane.mission.write_home_to_storage();
        }

        if (plane.mission.num_commands() == tmp_msg._msg_1.content.msg.wp_id) {
            AP_Mission::Mission_Command tmp_cmd;
            tmp_cmd.id = MAV_CMD_NAV_WAYPOINT;
            tmp_cmd.content.location.lat = (int32_t)(tmp_msg._msg_1.content.msg.lat*1.0e7);
            tmp_cmd.content.location.lng = (int32_t)(tmp_msg._msg_1.content.msg.lng*1.0e7);
            tmp_cmd.content.location.set_alt_cm((int32_t)(tmp_msg._msg_1.content.msg.alt*100.f), Location::AltFrame::ABSOLUTE);
            plane.mission.add_cmd(tmp_cmd);
            gcs().send_text(MAV_SEVERITY_INFO, "[%d] WP (%d / %d) saved", 1, tmp_msg._msg_1.content.msg.wp_id, plane.mission.num_commands());
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "INFO: wrong WP ID, (%d / %d)", tmp_msg._msg_1.content.msg.wp_id, plane.mission.num_commands());
        }
        gcs().send_text(MAV_SEVERITY_INFO, "plane.mission.num_commands(): %d", plane.mission.num_commands());





    // uint16_t line_index = tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.line_index;
    // uint16_t point_index = tmp_msg._msg_1.content.msg.remote_cmd.cmd_wp.point_index;  

    // line_index = MIN(0xFF,line_index);// now for control_id
    // point_index = MIN(0xFF,point_index);
    // tmp_cmd.p1 = (line_index << 8) | (point_index & 0x00FF);

    // if (HB1_Status.wp_to_renew || g2.hb1_num_wp == 0 || g2.hb1_num_interim != 0|| g2.hb1_num_land != 0) {
    //     plane.mission.clear();
    //     plane.mission.add_cmd(tmp_cmd);
    //     plane.mission.write_home_to_storage();
    //     g2.hb1_num_wp.set_and_save(0);
    //     g2.hb1_num_interim.set_and_save(0);
    //     g2.hb1_num_land.set_and_save(0);
    //     HB1_Status.wp_to_renew = false;
    // }
    // if (plane.mission.add_cmd(tmp_cmd)) {
    //     g2.hb1_num_wp.set_and_save(g2.hb1_num_wp.get()+1);
    //     gcs().send_text(MAV_SEVERITY_INFO, "[%d] WP (%d / %d) saved", line_index, g2.hb1_num_wp.get(), plane.mission.num_commands());
    //     HB1_lastWP_cmd = tmp_cmd;
    // }




        tmp_msg._msg_1.updated = false;
    }
}

void UCtrl::msg_handle_guide() {
    static uint32_t _last_print_ms = 0;
    FD1_msg_guide &tmp_msg = uart_msg_ctrl.get_msg_guide();
    if (tmp_msg._msg_1.updated) {
        if (millis() - _last_print_ms > 3000) {
        gcs().send_text(MAV_SEVERITY_INFO, "INFO: guide");
            _last_print_ms = millis();
        }
        tmp_msg._msg_1.updated = false;
    }
}


// void UCtrl::msg_handle() {
//     if (uart_msg_ts.get_msg_ts_in()._msg_1.updated) {

//         // copy to uart_msg_ts_route for following uart and mav uses
//         memcpy(uart_msg_mission.get_msg_ts_route()._msg_1.content.data, 
//             uart_msg_ts.get_msg_ts_in()._msg_1.content.data, 
//             uart_msg_ts.get_msg_ts_in()._msg_1.length*sizeof(uint8_t)+3);
//         uart_msg_mission.get_msg_ts_route()._msg_1.length = uart_msg_ts.get_msg_ts_in()._msg_1.length;
//         uart_msg_mission.get_msg_ts_route()._msg_1.updated = true;
//         uart_msg_mission.get_msg_ts_route()._msg_1.need_send = true;

//         // put handle code here

//         // gcs().send_text(MAV_SEVERITY_WARNING, "uart_msg_ts in %d %d", uart_msg_ts.get_msg_ts_in()._msg_1.length, uart_msg_mission.get_msg_ts_route()._msg_1.length);
//         // FD1_uart_ts_AHRS_test();
//         FD1_msg_ts &tmp_msg = uart_msg_ts.get_msg_ts_in();
//         if (tmp_msg._msg_1.content.msg.header.id == 0x40) {
//             uint8_t target_ret = tmp_msg._msg_1.content.msg.sub_msg.msg_40.sub_k1.target_ret;
//             if (target_ret & 0b00100000) {
//                 int32_t lat = tmp_msg._msg_1.content.msg.sub_msg.msg_40.sub_t1.target_lat;
//                 int32_t lng = tmp_msg._msg_1.content.msg.sub_msg.msg_40.sub_t1.target_lng;
//                 int32_t alt = tmp_msg._msg_1.content.msg.sub_msg.msg_40.sub_t1.target_alt*100;
//                 Location temp_loc(lat, lng, alt, Location::AltFrame::ABSOLUTE);
//                 set_target_loc(temp_loc);
//                 if (plane.control_mode == &plane.mode_auto) {
//                     plane.set_mode(plane.mode_gimbalfollow, ModeReason::MISSION_END);
//                 }
//             }
//         }
//         uart_msg_ts.get_msg_ts_in()._msg_1.updated = false;
//     }
// }

void UCtrl::ctrl_send() {
    static uint32_t _last_send_ms = millis();
    if (millis() - _last_send_ms > 100) {
        _last_send_ms = millis();
    } else {
        return;
    }

    if (plane.gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        return;
    }
    static uint8_t n_count = 0;

    SITL::SIM* _sitl = AP::sitl();
    if (_sitl == nullptr) {
        return;
    }
    if (_sitl) {
        const struct SITL::sitl_fdm &fdm = _sitl->state;

        // FD2_msg_ue4_ahrs &tmp_msg = FD2_uart_msg_ue4.get_msg_ue4_ahrs();
        // tmp_msg._msg_1.content.msg.header.head_1 = FD2_msg_ue4_ahrs::PREAMBLE1;
        // tmp_msg._msg_1.content.msg.header.head_2 = FD2_msg_ue4_ahrs::PREAMBLE2;
        // tmp_msg._msg_1.content.msg.vehicle_id = 0;
        // tmp_msg._msg_1.content.msg.lat = fdm.latitude * 1e7;
        // tmp_msg._msg_1.content.msg.lng = fdm.longitude * 1e7;
        // tmp_msg._msg_1.content.msg.alt = fdm.altitude*100;
        // tmp_msg._msg_1.content.msg.roll = fdm.rollDeg;
        // tmp_msg._msg_1.content.msg.pitch = fdm.pitchDeg;
        // tmp_msg._msg_1.content.msg.yaw = fdm.yawDeg;
        // tmp_msg._msg_1.content.msg.sum_check = 0;



        // calculate the absolute altitude.
        int32_t abs_alt = plane.current_loc.alt;
        if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,abs_alt)) {
            ;
        }

        // calculate the sink rate.
        float _attack_angle = 0.0f;
        float _side_slip_angle = 0.0f;
        Vector3f vel = Vector3f(fdm.speedN, fdm.speedE, fdm.speedD);
        // if (plane.ahrs.get_velocity_NED(vel)) {
            Matrix3f tmp_m;
            tmp_m.from_euler(plane.ahrs.roll, plane.ahrs.pitch, plane.ahrs.yaw);
            Vector3f new_vel = tmp_m.transposed()*vel;
            _attack_angle = wrap_180(degrees(-atan2f(new_vel.z, new_vel.x)));
            _side_slip_angle = wrap_180(degrees(atan2f(new_vel.y, new_vel.x)));
            // gcs().send_text(MAV_SEVERITY_INFO, "%f, %f", _attack_angle, _side_slip_angle);
        // }

        FD1_msg_info &tmp_msg = uart_msg_ctrl_send.get_msg_info();

        tmp_msg._msg_1.content.msg.header.head_1 = FD1_msg_info::PREAMBLE1;
        tmp_msg._msg_1.content.msg.header.head_2 = FD1_msg_info::PREAMBLE2;
        tmp_msg._msg_1.content.msg.device_id = FD1_msg_info::FRAMETYPE;
        tmp_msg._msg_1.content.msg.frame_id = FD1_msg_info::FRAMEID;
        tmp_msg._msg_1.content.msg.device_class = 0;
        tmp_msg._msg_1.content.msg.device_type = 0x01;
        tmp_msg._msg_1.content.msg.device_id = plane.g.sysid_this_mav;
        tmp_msg._msg_1.content.msg.gcs_class = 0x01;
        tmp_msg._msg_1.content.msg.gcs_id = 0x01;
        tmp_msg._msg_1.content.msg.info.lng = ((float)plane.current_loc.lng)*1e-7;
        tmp_msg._msg_1.content.msg.info.lat = ((float)plane.current_loc.lat)*1e-7;
        tmp_msg._msg_1.content.msg.info.alt = ((float)abs_alt)*0.01f;
        tmp_msg._msg_1.content.msg.info.roll = ((float)plane.ahrs.roll_sensor)/100.f;
        tmp_msg._msg_1.content.msg.info.pitch = ((float)plane.ahrs.pitch_sensor)/100.f;
        tmp_msg._msg_1.content.msg.info.yaw = ((float)plane.ahrs.yaw_sensor)/100.f;
        tmp_msg._msg_1.content.msg.info.ve = vel.y;
        tmp_msg._msg_1.content.msg.info.vn = vel.x;
        tmp_msg._msg_1.content.msg.info.vu = -vel.z;
        tmp_msg._msg_1.content.msg.info.airspeed = plane.airspeed.get_airspeed();
        tmp_msg._msg_1.content.msg.info.gps_speed = ((float)plane.gps.ground_speed());
        tmp_msg._msg_1.content.msg.info.attack_angle = _attack_angle;
        tmp_msg._msg_1.content.msg.info.side_slip_angle = _side_slip_angle;
        tmp_msg._msg_1.content.msg.info.climb_rate = -vel.z;
        tmp_msg._msg_1.content.msg.info.wind_speed = _sitl->wind_speed.get();
        tmp_msg._msg_1.content.msg.info.wind_direction = _sitl->wind_direction.get();
        tmp_msg._msg_1.content.msg.info.flap_left = SRV_Channels::get_output_norm(SRV_Channel::k_flap)*25.0f;
        tmp_msg._msg_1.content.msg.info.flap_right = -SRV_Channels::get_output_norm(SRV_Channel::k_flap)*25.0f;
        tmp_msg._msg_1.content.msg.info.elevator_left = SRV_Channels::get_output_norm(SRV_Channel::k_elevator)*25.0f;
        tmp_msg._msg_1.content.msg.info.elevator_right = -SRV_Channels::get_output_norm(SRV_Channel::k_elevator)*25.0f;
        tmp_msg._msg_1.content.msg.info.rudder_left = SRV_Channels::get_output_norm(SRV_Channel::k_rudder)*25.0f;
        tmp_msg._msg_1.content.msg.info.rudder_right = -SRV_Channels::get_output_norm(SRV_Channel::k_rudder)*25.0f;
        tmp_msg._msg_1.content.msg.info.flight_stage = 0;
        tmp_msg._msg_1.content.msg.info.flight_mode = 0;
        tmp_msg._msg_1.content.msg.info.landinggear_state = plane.is_flying()?1:0;
        tmp_msg._msg_1.content.msg.info.refly_state = 0;
        tmp_msg._msg_1.content.msg.info.fule = 100.f;
        tmp_msg._msg_1.content.msg.info.rpm = 100.f;
        tmp_msg._msg_1.content.msg.info.wp_distance = 1000.f;
        tmp_msg._msg_1.content.msg.count = n_count;

        tmp_msg.sum_check();
        tmp_msg._msg_1.content.msg.sum_check = 0;
        tmp_msg._msg_1.need_send = true;
        n_count++;
    }
}

void UCtrl::update_valid() {
    if (_last_msg_update_ms == 0 || millis() - _last_msg_update_ms > 1000) {
        set_valid(false);
    } else {
        set_valid(true);
    }

    if (!valid()) {
        //for protect purpose
        _target_loc.lat = plane.current_loc.lat;
        _target_loc.lng = plane.current_loc.lng;
        _target_loc.alt = plane.target_altitude.amsl_cm;
    }
}

void UCtrl::set_valid(bool v_in)
{
    if (v_in == _valid) {
        return;
    }
    _valid = v_in;
    if (_valid) {
        gcs().send_text(MAV_SEVERITY_INFO, "Target Valid");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Target lost");
    }
}

void UCtrl::set_target_loc(Location& loc_in) 
{
    Vector3f temp_pos;
    Vector3f target_pos;
    if (loc_in.get_vector_from_origin_NEU(temp_pos)) {
        _target_pos.apply(temp_pos);
        _target_loc = Location(_target_pos.get(), Location::AltFrame::ABOVE_ORIGIN);
        // _target_loc.alt = plane.target_altitude.amsl_cm;
        _last_msg_update_ms = millis();
        set_valid(true);
    }
}

void UCtrl::do_loiter_left()
{
    plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND);
    plane.mode_loiter.do_circle_ccw();
}

void UCtrl::do_loiter_right()
{
    plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND);
    plane.mode_loiter.do_circle_cw();
}

void UCtrl::do_cruise_up()
{
    do_cruise();
    plane.mode_cruise.change_target_altitude_cm(5000.f*100.f);
}

void UCtrl::do_cruise()
{
    plane.set_mode(plane.mode_cruise, ModeReason::GCS_COMMAND);
}

void UCtrl::do_cruise_alt(float targe_alt_m)
{
    do_cruise();
    plane.mode_cruise.change_target_altitude_cm((int32_t)targe_alt_m*100 - plane.target_altitude.amsl_cm);
    gcs().send_text(MAV_SEVERITY_INFO, "plane.target_altitude.amsl_cm %d", plane.target_altitude.amsl_cm);
}

void UCtrl::do_cruise_speed(float target_speed_ms)
{
    do_cruise();
    float speed_ms = target_speed_ms;
    if (speed_ms >= plane.aparm.airspeed_min.get() && speed_ms <= plane.aparm.airspeed_max.get())  {
        // airspeed_cruise_cm.set(speed_ms * 100f);
        gcs().send_text(MAV_SEVERITY_INFO, "Set airspeed %f m/s", speed_ms);
    }
}


