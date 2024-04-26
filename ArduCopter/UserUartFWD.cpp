#include "Copter.h"

UserUartFWD::UserUartFWD(enum AP_SerialManager::SerialProtocol protocol):
    _protocol(protocol)
{
    _port = NULL;
    _initialized = false;
}

void UserUartFWD::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(_protocol, 0))) {
        gcs().send_text(MAV_SEVERITY_INFO, "Gimbal FWD Init, type %d", copter.g2.user_parameters.forward_type.get());
        _initialized = true;
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Gimbal FWD Init Failed");
    }
}

void UserUartFWD::handle_msg(const mavlink_message_t &msg)
{
    if (!_initialized) {return;}
        // gcs().send_text(MAV_SEVERITY_INFO, "%d -> %d" , msg.sysid, msg.msgid);
    // if (_target_sys_id != 0 && _target_sys_id != msg.sysid) {return;}
    if (msg.msgid == MAVLINK_MSG_ID_MY_UART_FORWARD) {
        // decode packet
        mavlink_my_uart_forward_t my_uart_forward;
        mavlink_msg_my_uart_forward_decode(&msg, &my_uart_forward);
        if (copter.g2.user_parameters.forward_print.get() == 1) {
            gcs().send_text(MAV_SEVERITY_INFO, "data_len receive %d", my_uart_forward.data_len);
        }
        if (copter.g2.user_parameters.forward_type.get() == 0) {
            _port->write(my_uart_forward.data, my_uart_forward.data_len);
        } else if (copter.g2.user_parameters.forward_type.get() == 1) {            
            for (uint16_t i=0; i<my_uart_forward.data_len; i++) {
                copter.ugimbal.read_command_byte(my_uart_forward.data[i]);
                FD1_msg_gcs2gimbal &apm2gimbal = copter.ugimbal.get_msg_apm2gimbal();
                if (apm2gimbal._msg_1.need_send) {
                    apm2gimbal._msg_1.need_send = false;
                    _port->write(apm2gimbal._msg_1.content.data, apm2gimbal._msg_1.length);
                }
            }
        }
    }
}

void UserUartFWD::set_target_sysid(uint16_t id_in)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Change Source ID %d -> %d", _target_sys_id, id_in);
    _target_sys_id = id_in;
}

void UserUartFWD::update()
{
    if (!_initialized) {return;}
    read_uart();
    send_mav();
}

void UserUartFWD::read_uart() 
{
    if (!_initialized) {return;}
    while(_port->available() > 0) {
        uint8_t temp = _port->read();
        push_byte(temp);
        copter.ugimbal.read_status_byte(temp);
    }
    // _port->write(0xF1);
    // _port->write(0xF2);
    // _port->write(0xF3);
}

// void UserUartFWD::send_uart()
// {
//     if (!_initialized) {return;}
//     temp_msg = copter.ugimbal.get_msg_apm2gimbal();
//     if (temp_msg._msg_1.need_send) {
//         temp_msg.swap_message();
//          _port->write(temp_msg._msg_1.content.data, temp_msg._msg_1.length);
//         temp_msg._msg_1.updated = false;
//         temp_msg._msg_1.need_send = false;
//     }
// }

void UserUartFWD::push_byte(uint8_t temp) 
{
    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        data_buffer_instance[i].push(temp);
    }

    for (uint8_t i=0; i<NUM_MY_DATA; i++) {
        data_buffer_instance[i]._id = i;
        data_buffer_instance[i].update();
    }

}

void UserUartFWD::send_mav()
{
    static uint32_t _last_send_ms = 0;
    uint32_t tnow = millis();
    // uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    if (tnow - _last_send_ms > 100) {
        _last_send_ms = tnow;
        for (uint8_t i=0; i<gcs().num_gcs(); i++) {
            mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
            // if (mask & (1U<<i)) {
                if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
                    data_buffer_instance[i].set_active();
                    mavlink_my_uart_forward_t my_uart_forward;
                    my_uart_forward.data_len = data_buffer_instance[i].get_data(my_uart_forward.data);
                    if (my_uart_forward.data_len > 0 && (copter.g2.user_parameters.forward_print.get() == 1)) {
                        gcs().send_text(MAV_SEVERITY_INFO, "data_len send %d", my_uart_forward.data_len );
                    }
                    if (my_uart_forward.data_len > 0) {
                        mavlink_msg_my_uart_forward_send(
                            channel,
                            my_uart_forward.data_len,
                            my_uart_forward.data);
                    }
    // mavlink_msg_heartbeat_send(
    //     channel,
    //     gcs().frame_type(),
    //     MAV_AUTOPILOT_ARDUPILOTMEGA,
    //     0,
    //     gcs().custom_mode(),
    //     0);
                }
            // }
        }
    }
}

void User_data_buffer::set_active()
{
    if (!_active) {
        gcs().send_text(MAV_SEVERITY_INFO, "Fwd chan(%d) connect", _id);
    }
    _active = true;
    _last_active_ms = millis();
}

void User_data_buffer::update() 
{
    uint32_t tnow = millis();
    if (tnow - _last_active_ms > 3000) {
        if (_active) {
            gcs().send_text(MAV_SEVERITY_INFO, "Fwd chan(%d) disconnect", _id);
            _active = false;
        }
    }
}

//  - - - data_idx - - -
//  a a a 0        0 0 0
uint16_t User_data_buffer::get_data(uint8_t (&data)[200])
{
    // gcs().send_text(MAV_SEVERITY_INFO, "data_idx %d", data_idx);
    uint16_t avaliable_data_len = MIN(data_idx, 200-1);
    memcpy(data, _data, avaliable_data_len);
    data_idx -= (avaliable_data_len);
    // re-organize data, move rest data to 0 positon
    for (uint8_t i=0; i<data_idx; i++) {
        _data[i] = _data[i+avaliable_data_len];
    }
    return avaliable_data_len;
} 

void User_data_buffer::push(uint8_t c)
{
    if (!_active) {return;}
    static uint32_t _last_log_ms = 0;
    if (data_idx < 500-1) {
        _data[data_idx] = c;
        data_idx++;
    } else {
        uint32_t tnow = millis();
        if (tnow - _last_log_ms > 5000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Fwd chan(%d) is full", _id);
            _last_log_ms = tnow;
        }
    }
}