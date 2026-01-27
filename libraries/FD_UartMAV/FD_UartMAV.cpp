#include "FD_UartMAV.h"

// Convenience macros //////////////////////////////////////////////////////////
//
const AP_Param::GroupInfo FD_UartMAV::var_info[] = {
    AP_GROUPINFO("_Port",     0, FD_UartMAV, port_num,         -1),
    AP_GROUPINFO("_Print",    1, FD_UartMAV, info_print,        0),
    AP_GROUPINFO("_SYS_ID",   2, FD_UartMAV, sys_id,            1),
    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

FD_UartMAV *FD_UartMAV::_singleton;

FD_UartMAV::FD_UartMAV()
{
    _initialized = false;
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
    return;
}

void FD_UartMAV::init() {
    last_check_ms = millis();

    _initialized = false;

    if (port_num.get() <0) {
        return;
    }

    _port = AP::serialmanager().get_serial_by_id(port_num.get());
    if (_port == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "SERIAL%d !port", port_num.get());
        return;
    }

    const auto *uart_state = AP::serialmanager().get_state_by_id(port_num.get());
    if (!uart_state) {
        gcs().send_text(MAV_SEVERITY_INFO, "SERIAL%d !state", port_num.get());
        return;
    }
    if (uart_state->protocol.get() != 2) {
        gcs().send_text(MAV_SEVERITY_INFO, "SERIAL%d is used with PROTOCOL%d", port_num.get(), uart_state->protocol.get());
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Uart Mav in SERIAL%d", port_num.get());


    _initialized = true;
}

bool FD_UartMAV::initialized()
{
    return _initialized;
}

void FD_UartMAV::update() {
    if (!_initialized) {
        if ((port_num.get() >=0) && (millis() - last_check_ms > 10000)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Uart Mav try");
            init();
        }
        return;
    }
    read_uart();
    send_mav();
}

void FD_UartMAV::read_uart() 
{
    // if (!_initialized) {return;}
    // while(_port->available() > 0) {
    //     uint8_t temp = _port->read();
    //     data_buffer_instance.push(temp);
    // }
    // // _port->write(0xF1);
    // // _port->write(0xF2);
    // // _port->write(0xF3);
}

void FD_UartMAV::send_mav()
{
    send_raw_gps();
}

void FD_UartMAV::send_raw_gps()
{
    if (!_initialized) {
        return;
    }

    static uint32_t last_gps_ms = AP_HAL::millis();
    if (AP_HAL::millis() - last_gps_ms < 100) {
        return;
    }
    last_gps_ms = AP_HAL::millis();

    mavlink_message_t msg;
    AP::gps().pack_mavlink_gps_raw(sys_id.get(), sys_id.get(), msg);
    send_msg(&msg);
}

void FD_UartMAV::handle_msg(const mavlink_message_t &msg)
{
    // if (!_initialized) {return;}
    // if (msg.msgid == MAVLINK_MSG_ID_MY_UART_FORWARD) {
    //     // decode packet
    //     mavlink_my_uart_forward_t my_uart_forward;
    //     mavlink_msg_my_uart_forward_decode(&msg, &my_uart_forward);

    //     if (info_print.get() == 1) {
    //         gcs().send_text(MAV_SEVERITY_INFO, "data_len receive %d", my_uart_forward.data_len);
    //     }
    //     _port->write(my_uart_forward.data, my_uart_forward.data_len);
    // }
}

void FD_UartMAV::set_target_sysid(uint16_t id_in)
{
    ;
}

void FD_UartMAV::send_msg(mavlink_message_t *msg)
{
    uint8_t ck[2];

    ck[0] = (uint8_t)(msg->checksum & 0xFF);
    ck[1] = (uint8_t)(msg->checksum >> 8);
    // XXX use the right sequence here

        uint8_t header_len;
        // uint8_t signature_len;
        
        if (msg->magic == MAVLINK_STX_MAVLINK1) {
            header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1;
            // signature_len = 0;
            // we can't send the structure directly as it has extra mavlink2 elements in it
            uint8_t buf[MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1];
            buf[0] = msg->magic;
            buf[1] = msg->len;
            buf[2] = msg->seq;
            buf[3] = msg->sysid;
            buf[4] = msg->compid;
            buf[5] = msg->msgid & 0xFF;
            _port->write(buf, header_len);
        } else {
            header_len = MAVLINK_CORE_HEADER_LEN + 1;
            // signature_len = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED)?MAVLINK_SIGNATURE_BLOCK_LEN:0;
            uint8_t buf[MAVLINK_CORE_HEADER_LEN + 1];
            buf[0] = msg->magic;
            buf[1] = msg->len;
            buf[2] = msg->incompat_flags;
            buf[3] = msg->compat_flags;
            buf[4] = msg->seq;
            buf[5] = msg->sysid;
            buf[6] = msg->compid;
            buf[7] = msg->msgid & 0xFF;
            buf[8] = (msg->msgid >> 8) & 0xFF;
            buf[9] = (msg->msgid >> 16) & 0xFF;
            _port->write(buf, header_len);
        }

    _port->write((uint8_t *)_MAV_PAYLOAD(msg), msg->len);
    _port->write((uint8_t *)ck, 2);
}

namespace AP {

FD_UartMAV &fd_uartmav()
{
    return *FD_UartMAV::get_singleton();
}

};
