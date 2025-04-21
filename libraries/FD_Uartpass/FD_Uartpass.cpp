#include "FD_Uartpass.h"

// Convenience macros //////////////////////////////////////////////////////////
//
const AP_Param::GroupInfo FD_Uartpass::var_info[] = {
    AP_GROUPINFO("_Port",   0, FD_Uartpass, port_num,        -1),
    AP_GROUPINFO("_Baud",   1, FD_Uartpass, port_baud,        57),
    AP_GROUPINFO("_Print",  2, FD_Uartpass, info_print,       0),
    AP_GROUPINFO("_ID",     3, FD_Uartpass, source_sys_id,    0),
    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

FD_Uartpass::FD_Uartpass()
{
    _initialized = false;
    AP_Param::setup_object_defaults(this, var_info);
    return;
}

void FD_Uartpass::init() {
    last_check_ms = millis();

    _initialized = false;

    if (port_num.get() <=0) {
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
    if (uart_state->protocol.get() != 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "SERIAL%d is used", port_num.get());
        return;
    }

    _port->begin(AP_SerialManager::map_baudrate(port_baud.get()));

    gcs().send_text(MAV_SEVERITY_INFO, "Uart Pass in SERIAL%d", port_num.get());
    _initialized = true;
}

void FD_Uartpass::update() {
    if (!_initialized) {
        if ((port_num.get() >=0) && (millis() - last_check_ms > 10000)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Uart Pass try");
            init();
        }
        return;
    }
    read_uart();
    data_buffer_instance.set_active();
    data_buffer_instance.update();
    send_mav();
}

void FD_Uartpass::read_uart() 
{
    if (!_initialized) {return;}
    while(_port->available() > 0) {
        uint8_t temp = _port->read();
        data_buffer_instance.push(temp);
    }
    // _port->write(0xF1);
    // _port->write(0xF2);
    // _port->write(0xF3);
}

void FD_Uartpass::send_mav()
{
    static uint32_t _last_send_ms = 0;
    uint32_t tnow = millis();
    uint16_t mask = GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask();
    if (tnow - _last_send_ms > 100) {
        _last_send_ms = tnow;
        mavlink_my_uart_forward_t my_uart_forward;
        my_uart_forward.data_len = data_buffer_instance.get_data(my_uart_forward.data);
        if (my_uart_forward.data_len > 0 && (info_print.get() == 1)) {
            gcs().send_text(MAV_SEVERITY_INFO, "data_len send %d", my_uart_forward.data_len );
        }
        for (uint8_t i=0; i<gcs().num_gcs(); i++) {
            mavlink_channel_t channel = (mavlink_channel_t)(MAVLINK_COMM_0 + i);
            if (mask & (1U<<i)) {
                if (comm_get_txspace(channel) >= GCS_MAVLINK::packet_overhead_chan(channel) + 255) {
                    if (my_uart_forward.data_len > 0) {
                        mavlink_msg_my_uart_forward_send(
                            channel,
                            my_uart_forward.data_len,
                            my_uart_forward.data);
                    }
                }
            }
        }
    }
}

void FD_Uartpass::handle_msg(const mavlink_message_t &msg)
{
    if (!_initialized) {return;}
    if (msg.msgid == MAVLINK_MSG_ID_MY_UART_FORWARD) {
        // decode packet
        mavlink_my_uart_forward_t my_uart_forward;
        mavlink_msg_my_uart_forward_decode(&msg, &my_uart_forward);

        if (info_print.get() == 1) {
            gcs().send_text(MAV_SEVERITY_INFO, "data_len receive %d", my_uart_forward.data_len);
        }
        _port->write(my_uart_forward.data, my_uart_forward.data_len);
    }
}

void FD_Uartpass::set_target_sysid(uint16_t id_in)
{
    ;
}