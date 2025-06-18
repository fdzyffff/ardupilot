#include "Copter.h"


UBIM::UBIM()
{
    // AP_Param::setup_object_defaults(this, var_info);
}

// initialise
void UBIM::init()
{
    if (uart_bim.init()) {
        uart_bim.get_msg_BIMCMD().set_enable();
        uart_bim.get_msg_BIMSTATUS().set_enable();
        gcs().send_text(MAV_SEVERITY_INFO, "BIM INIT");
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Err: BIM INIT FAIL");
    }
}


void UBIM::update_log()
{
    // AP::logger().WriteStreaming("UATK",
    //                             "TimeUS,bfx,bfy,efx,efy,efrx,efry,tpth,trll,tyaw",
    //                             "s---------",
    //                             "F---------",
    //                             "Qfffffffff",
    //                             AP_HAL::micros64(),
    //                             (float)bf_info.x,
    //                             (float)bf_info.y,
    //                             (float)ef_info.x,
    //                             (float)ef_info.y,
    //                             (float)ef_rate_info.x,
    //                             (float)ef_rate_info.y,
    //                             (float)_target_pitch_rate,
    //                             (float)_target_roll_angle,
    //                             (float)_target_yaw_rate);

    // AP::logger().WriteStreaming("UAT2",
    //                             "TimeUS,angt,angm,agrt,agrm",
    //                             "s----",
    //                             "F----",
    //                             "Qffff",
    //                             AP_HAL::micros64(),
    //                             (float)_attack_angle_target,
    //                             (float)_attack_angle_measure,
    //                             (float)_attack_angle_rate_target,
    //                             (float)_attack_angle_rate_measure);

    // AP::logger().WriteStreaming("UATH",
    //                             "TimeUS,target,actual,ff,P,I,D,srate,dmod",
    //                             "s--------",
    //                             "F--------",
    //                             "Qffffffff",
    //                             AP_HAL::micros64(),
    //                             (float)attack_throttle_pid.get_pid_info().target,
    //                             (float)attack_throttle_pid.get_pid_info().actual,
    //                             (float)attack_throttle_pid.get_pid_info().FF,
    //                             (float)attack_throttle_pid.get_pid_info().P,
    //                             (float)attack_throttle_pid.get_pid_info().I,
    //                             (float)attack_throttle_pid.get_pid_info().D,
    //                             (float)attack_throttle_pid.get_pid_info().slew_rate,
    //                             (float)attack_throttle_pid.get_pid_info().Dmod);

    // AP::logger().WriteStreaming("UARL",
    //                             "TimeUS,target,actual,ff,P,I,D,srate,dmod",
    //                             "s--------",
    //                             "F--------",
    //                             "Qffffffff",
    //                             AP_HAL::micros64(),
    //                             (float)attack_roll_pid.get_pid_info().target,
    //                             (float)attack_roll_pid.get_pid_info().actual,
    //                             (float)attack_roll_pid.get_pid_info().FF,
    //                             (float)attack_roll_pid.get_pid_info().P,
    //                             (float)attack_roll_pid.get_pid_info().I,
    //                             (float)attack_roll_pid.get_pid_info().D,
    //                             (float)attack_roll_pid.get_pid_info().slew_rate,
    //                             (float)attack_roll_pid.get_pid_info().Dmod);

}
// called at 100 Hz

void UBIM::update()
{
    // for log purpose
    static uint32_t last_count_ms = millis();
    uint32_t tnow_ms = millis();
    if (tnow_ms - last_count_ms > 1000) {
        update_log();
    }

    update_msg_cmd();
    update_msg_send();
}

void UBIM::update_msg_cmd()
{
    while (uart_bim.port_avaliable() > 0) {
        uint8_t temp = uart_bim.read_byte();
        uart_bim.parse(temp);
        // gcs().send_text(MAV_SEVERITY_INFO, "temp %x", temp);

        FD1_msg_BIMCMD &tmp_msg = uart_bim.get_msg_BIMCMD();
        if (tmp_msg._msg_1.updated) {
            if (tmp_msg._msg_1.content.msg.uav_id != (uint32_t)copter.g.sysid_this_mav.get()) {
                return;
            }
            
            _plat_switch_cmd = tmp_msg._msg_1.content.msg.plat_switch_cmd[0];
            switch (_plat_switch_cmd) {
                case 0xA0:
                    {
                        _plat_switch_act = switch_back_to_wp();
                        break;
                    }
                case 0x28:
                    {
                        _plat_switch_act = switch_hover();
                        break;
                    }
                case 0x40:
                    {
                        _plat_switch_act = switch_unlock();
                        break;
                    }
                case 0x42:
                    {
                        _plat_switch_act = switch_manual();
                        break;
                    }
                case 0x44:
                    {
                        _plat_switch_act = switch_land();
                        break;
                    }
                default:
                    _plat_switch_act = false;
                    break;
            }

            _plat_input_cmd = tmp_msg._msg_1.content.msg.plat_input_cmd[0];
            switch (_plat_input_cmd) {
                case 0x56:
                    {
                        _plat_input_act = cmd_add_wp();
                        break;
                    }
                case 0x72:
                    {
                        _plat_input_act = cmd_set_pos();
                        break;
                    }
                case 0x74:
                    {
                        _plat_input_act = cmd_set_speed();
                        break;
                    }
                case 0x76:
                    {
                        _plat_input_act = cmd_set_alt();
                        break;
                    }
                case 0x78:
                    {
                        _plat_input_act = cmd_set_yaw();
                        break;
                    }
                case 0x7A:
                    {
                        _plat_input_act = cmd_set_pos_offset();
                        break;
                    }
                default:
                    _plat_input_act = false;
                    break;
            }
            tmp_msg._msg_1.updated = false;
        }
    }
}

void UBIM::update_msg_send()
{
    if (!uart_bim.initialized()) {return;}
    static uint32_t last_send_ms = millis();
    uint32_t tnow_ms = millis();
    if (tnow_ms - last_send_ms > 100) {
        last_send_ms = tnow_ms;

        FD1_msg_BIMSTATUS &tmp_msg = uart_bim.get_msg_BIMSTATUS();
        tmp_msg._msg_1.need_send = true;

        tmp_msg._msg_1.content.msg.length = 150;
        tmp_msg._msg_1.content.msg.idx = 0x01;
        tmp_msg._msg_1.content.msg.version = 0x00;
        tmp_msg._msg_1.content.msg.flag_sim = 0;
        tmp_msg._msg_1.content.msg.uav_type = 2;
        tmp_msg._msg_1.content.msg.uav_id = copter.g.sysid_this_mav.get();
        tmp_msg._msg_1.content.msg.lng = 0.0;
        tmp_msg._msg_1.content.msg.lat = 0.0;
        if (copter.position_ok()) {
            tmp_msg._msg_1.content.msg.lng = (int32_t)(((float)copter.current_loc.lng) * (214.7483647f/180.f));
            tmp_msg._msg_1.content.msg.lat = (int32_t)(((float)copter.current_loc.lat) * (214.7483647f/180.f));
        }
        float tmp_alt = 0.0f;
        if (copter.ahrs_view->get_relative_position_D_origin(tmp_alt))
        {
            ;
        }
        tmp_msg._msg_1.content.msg.alt_baro = (int16_t)tmp_alt;
        tmp_msg._msg_1.content.msg.pitch = (int16_t)(degrees(copter.ahrs_view->pitch) * (65535.f/180.f));
        tmp_msg._msg_1.content.msg.roll = (int16_t)(degrees(copter.ahrs_view->roll) * (65535.f/180.f));
        tmp_msg._msg_1.content.msg.yaw = (int16_t)(wrap_360(degrees(copter.ahrs_view->yaw)) * (65535.f/360.f));
        tmp_msg._msg_1.content.msg.power_rest = 99;
        tmp_msg._msg_1.content.msg.dist_roll = 0;
        tmp_msg._msg_1.content.msg.target_speed = 0;
        tmp_msg._msg_1.content.msg.target_alt = 0;
        tmp_msg._msg_1.content.msg.next_wp_id = copter.mode_auto.mission.get_current_nav_index();
        tmp_msg._msg_1.content.msg.next_wp_dist = copter.flightmode->wp_distance();
        tmp_msg._msg_1.content.msg.plat_switch_cmd = _plat_switch_cmd;
        tmp_msg._msg_1.content.msg.plat_switch_act = _plat_switch_act;
        tmp_msg._msg_1.content.msg.plat_input_cmd = _plat_input_cmd;
        memcpy(tmp_msg._msg_1.content.msg.plat_input_param, uart_bim.get_msg_BIMCMD()._msg_1.content.msg.plat_input_param.data, 28);
        tmp_msg._msg_1.content.msg.plat_input_act = _plat_input_act;
        tmp_msg._msg_1.content.msg.pos_x = 0.0f;
        tmp_msg._msg_1.content.msg.pos_y = 0.0f;
        tmp_msg._msg_1.content.msg.pos_z = 0.0f;
        if (copter.position_ok()) {
            Vector3f current_pos;
            if (copter.ahrs_view->get_relative_position_NED_origin(current_pos))
            {
                ;
            }
            tmp_msg._msg_1.content.msg.pos_x = (int32_t)( current_pos.y*100.f);
            tmp_msg._msg_1.content.msg.pos_y = (int32_t)( current_pos.x*100.f);
            tmp_msg._msg_1.content.msg.pos_z = (int32_t)(-current_pos.z*100.f);
        }
        tmp_msg._msg_1.content.msg.control_mode = uav_manual?2:1;
        tmp_msg._msg_1.content.msg.uav_moving_status = 0;
        if (copter.position_ok()) {
            Vector3f tmp_vec;
            if (copter.ahrs_view->get_velocity_NED(tmp_vec))
            {
                if (copter.flightmode->mode_number() == Mode::Number::LAND) {
                    tmp_msg._msg_1.content.msg.uav_moving_status = 3;
                }
                else if (tmp_vec.xy().length()*100.f > 20.f) {
                    tmp_msg._msg_1.content.msg.uav_moving_status = 1;
                } else {
                    tmp_msg._msg_1.content.msg.uav_moving_status = 0;
                }
            }
        }
        tmp_msg._msg_1.content.msg.arm_status = uav_unlock?1:0;
        tmp_msg._msg_1.content.msg.copter_speed = 0.0f;
        if (copter.position_ok()) {
            Vector3f tmp_vec;
            if (copter.ahrs_view->get_velocity_NED(tmp_vec))
            {
                ;
            }
            tmp_msg._msg_1.content.msg.copter_speed = tmp_vec.xy().length()*100.f;
        }
        tmp_msg.sum_check();
        uart_bim.get_port()->write(tmp_msg._msg_1.content.data, sizeof(tmp_msg._msg_1.content.data));
    }
}