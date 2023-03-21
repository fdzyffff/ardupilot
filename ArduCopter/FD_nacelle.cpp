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
    if (copter.g2.user_parameters.stat_print.get() & (2<<0)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "down: %x", temp);
    }
    nacelle_handle_and_route();
    FD1_uart_msg_gcs.get_port()->write(temp);
    // FD1_uart_msg_gcs.write();  
}

void UserNacelle::gcs_read(uint8_t temp) 
{
    user_stat.gcs_byte_count++;
    FD1_uart_msg_gcs.read(temp);
    if (copter.g2.user_parameters.stat_print.get() & (2<<0)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "up: %x", temp);
    }
    gcs_handle_and_route();
    FD1_uart_msg_nacelle.get_port()->write(temp);
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

        // put handle code here

        // gcs().send_text(MAV_SEVERITY_WARNING, "FD1_uart_msg_nacelle in %d %d", FD1_uart_msg_nacelle.get_msg_nacelle_in()._msg_1.length, FD1_uart_msg_gcs.get_msg_nacelle_route()._msg_1.length);
        // FD1_uart_nacelle_AHRS_test();
        FD1_uart_msg_nacelle.get_msg_nacelle2gcs()._msg_1.updated = false;
        gcs().send_text(MAV_SEVERITY_WARNING, "nacelle route");
        copter.gcs().send_message(MSG_NACELLE);
    }
}

void UserNacelle::print_info()
{
    if (copter.g2.user_parameters.stat_print.get() & (1<<0)) { // 1
        gcs().send_text(MAV_SEVERITY_WARNING, "up %d/s[%d], down %d/s[%d]", user_stat.gcs_byte_count, user_stat.gcs_valid_byte_count, user_stat.nacelle_byte_count, user_stat.nacelle_valid_byte_count);
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

// void Copter::FD1_uart_nacelle_AHRS_test()
// {
//     FD1_msg_nacelle2gcs &tmp_msg = FD1_uart_msg_nacelle.get_msg_nacelle2gcs();

//     if (tmp_msg._msg_1.content.msg.header.id == 0xB1) {
//         gcs().send_text(MAV_SEVERITY_WARNING, "roll_angle: %d", tmp_msg._msg_1.content.msg.sub_msg.msg_m.roll_angle);
//         gcs().send_text(MAV_SEVERITY_WARNING, "pitch_angle: %d", tmp_msg._msg_1.content.msg.sub_msg.msg_m.pitch_angle);
//         gcs().send_text(MAV_SEVERITY_WARNING, "yaw_angle: %d", tmp_msg._msg_1.content.msg.sub_msg.msg_m.yaw_angle);
//     }
// }

