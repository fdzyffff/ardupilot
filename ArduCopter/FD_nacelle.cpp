#include "Copter.h"

void UserNacelle::Init() {
    FD1_uart_msg_nacelle.init();
    FD1_uart_msg_nacelle.get_msg_gcs2nacelle().set_enable();
    FD1_uart_msg_nacelle.get_msg_nacelle2gcs().set_enable();

    FD1_uart_msg_gcs.init();
    // FD1_uart_msg_gcs.set_port(hal.serial(1));
    FD1_uart_msg_gcs.get_msg_gcs2nacelle().set_enable();
    FD1_uart_msg_gcs.get_msg_nacelle2gcs().set_enable();

    user_stat.nacelle_byte_count = 0;
    user_stat.gcs_byte_count = 0;
    user_stat.nacelle_valid_byte_count = 0;
    user_stat.gcs_valid_byte_count = 0;
}

void UserNacelle::Update() {
    // msg from nacelle uart, route to the gcs uart
    while (FD1_uart_msg_nacelle.port_avaliable() > 0) {
        uint8_t temp = FD1_uart_msg_nacelle.port_read();
        nacelle_read(temp);
    }

    // msg from gcs uart, route to the nacelle uart
    while (FD1_uart_msg_gcs.port_avaliable() > 0) {
        uint8_t temp = FD1_uart_msg_gcs.port_read();
        gcs_read(temp);
        // FD1_uart_msg_nacelle.write();  
    }
}


void UserNacelle::nacelle_read(uint8_t temp) 
{
    user_stat.nacelle_byte_count++;
    FD1_uart_msg_nacelle.read(temp);
    if (copter.g2.user_parameters.stat_print.get() & (1<<1)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "down: %x", temp);
    }
    nacelle_handle_and_route();
    if (FD1_uart_msg_gcs.initialized()) {
        FD1_uart_msg_gcs.port_write(temp);
    }
    // FD1_uart_msg_gcs.write();  
}

void UserNacelle::gcs_read(uint8_t temp) 
{
    user_stat.gcs_byte_count++;
    FD1_uart_msg_gcs.read(temp);
    if (copter.g2.user_parameters.stat_print.get() & (1<<1)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "up: %x", temp);
    }
    gcs_handle_and_route();
    FD1_uart_msg_nacelle.port_write(temp);
    // FD1_uart_msg_nacelle.write();  
}

void UserNacelle::gcs_handle_and_route() {
    if (FD1_uart_msg_gcs.get_msg_gcs2nacelle()._msg_1.updated) {
        user_stat.gcs_valid_byte_count+=FD1_uart_msg_gcs.get_msg_gcs2nacelle()._msg_1.length;
        // copy to FD1_uart_msg_nacelle_route for following uart and mav uses
        memcpy(FD1_uart_msg_nacelle.get_msg_gcs2nacelle()._msg_1.content.data, 
            FD1_uart_msg_gcs.get_msg_gcs2nacelle()._msg_1.content.data, 
            FD1_uart_msg_gcs.get_msg_gcs2nacelle()._msg_1.length*sizeof(uint8_t)+3);
        FD1_uart_msg_nacelle.get_msg_gcs2nacelle()._msg_1.length = FD1_uart_msg_gcs.get_msg_gcs2nacelle()._msg_1.length;
        FD1_uart_msg_nacelle.get_msg_gcs2nacelle()._msg_1.updated = true;
        FD1_uart_msg_nacelle.get_msg_gcs2nacelle()._msg_1.need_send = true;

        // gcs().send_text(MAV_SEVERITY_WARNING, "FD1_uart_msg_gcs in %d", FD1_uart_msg_gcs.get_msg_gcs2nacelle()._msg_1.length);
        // put handle code here
        FD1_uart_msg_gcs.get_msg_gcs2nacelle()._msg_1.updated = false;
    }
}

void UserNacelle::nacelle_handle_and_route() {
    if (FD1_uart_msg_nacelle.get_msg_nacelle2gcs()._msg_1.updated) {
        user_stat.nacelle_valid_byte_count+=FD1_uart_msg_nacelle.get_msg_nacelle2gcs()._msg_1.length;
        // copy to FD1_uart_msg_nacelle_route for following uart and mav uses
        memcpy(FD1_uart_msg_gcs.get_msg_nacelle2gcs()._msg_1.content.data, 
            FD1_uart_msg_nacelle.get_msg_nacelle2gcs()._msg_1.content.data, 
            FD1_uart_msg_nacelle.get_msg_nacelle2gcs()._msg_1.length*sizeof(uint8_t)+3);
        FD1_uart_msg_gcs.get_msg_nacelle2gcs()._msg_1.length = FD1_uart_msg_nacelle.get_msg_nacelle2gcs()._msg_1.length;
        FD1_uart_msg_gcs.get_msg_nacelle2gcs()._msg_1.updated = true;
        FD1_uart_msg_gcs.get_msg_nacelle2gcs()._msg_1.need_send = true;

        nacelle_update_status();
        // put handle code here

        FD1_uart_msg_nacelle.get_msg_nacelle2gcs()._msg_1.updated = false;
        copter.gcs().send_message(MSG_NACELLE);
    }
}

void UserNacelle::print_info()
{
    if (copter.g2.user_parameters.stat_print.get() & (1<<0)) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "up %d/s[%d], down %d/s[%d]", user_stat.gcs_byte_count, user_stat.gcs_valid_byte_count, user_stat.nacelle_byte_count, user_stat.nacelle_valid_byte_count);
    }
    if (copter.g2.user_parameters.stat_print.get() & (1<<2)) { // 2
        gcs().send_text(MAV_SEVERITY_WARNING, "p: %f y:%f", status.pitch_angle*0.01f, status.yaw_angle*0.01f);
    }
    user_stat.nacelle_byte_count = 0;
    user_stat.gcs_byte_count = 0;
    user_stat.nacelle_valid_byte_count = 0;
    user_stat.gcs_valid_byte_count = 0;
}

void UserNacelle::handle_msg(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_MY_OPTIC_DATA: {
            // decode message
            mavlink_my_optic_data_t packet;
            mavlink_msg_my_optic_data_decode(&msg, &packet);
            for (uint8_t i=0; i< sizeof(packet.data); i++) {
                gcs_read(packet.data[i]);
            }
            break;        
        }
        default:
            break;
    }
}

void UserNacelle::nacelle_update_status() {
    status.last_msg_ms = millis();
    status.pitch_angle = -(float)FD1_uart_msg_nacelle.get_msg_nacelle2gcs()._msg_1.content.msg.sub_msg.msg_m.pitch_angle;
    status.yaw_angle_body = (float)FD1_uart_msg_nacelle.get_msg_nacelle2gcs()._msg_1.content.msg.sub_msg.msg_m.yaw_angle;
    status.yaw_angle = wrap_360_cd(status.yaw_angle_body + degrees(copter.ahrs_view->yaw)*100.f);
}

bool UserNacelle::valid() {
    return !(millis() - status.last_msg_ms > 5000);
}

float UserNacelle::get_yaw() {
    return status.yaw_angle;;
}

float UserNacelle::get_pitch() {
    return status.pitch_angle;;
}

// void Copter::FD1_uart_nacelle_AHRS_test()
// {
//     FD1_msg_nacelle2gcs &tmp_msg = FD1_uart_msg_nacelle.get_msg_nacelle2gcs();

//     if (tmp_msg._msg_1.content.msg.header.id == 0xB1) {
//         gcs().send_text(MAV_SEVERITY_WARNING, "roll_angle: %d", tmp_msg._msg_1.content.msg.sub_msg.msg_m.roll_angle);
//         gcs().send_text(MAV_SEVERITY_WARNING, "pitch_angle: %d", tmp_msg._msg_1.content.msg.sub_msg.msg_m.pitch_angle);
//         gcs().send_text(MAV_SEVERITY_WARNING, "yaw_angle: %d", tmp_msg._msg_1.content.msg.sub_msg.msg_m.yaw_angle);
//     }
// }

