#include "Plane.h"

void Plane::FD1_uart_ep4_init() {
    FD1_uart_msg_ep4.init();
    FD1_uart_msg_ep4.get_msg_ep4_in().set_enable();
    FD1_uart_msg_ep4.get_msg_ep4_out().set_enable();

    FD1_uart_msg_ep4_route.init();
    FD1_uart_msg_ep4_route.get_msg_ep4_in().set_enable();

    ep4_state.last_update_ms = 0;
}

void Plane::FD1_uart_ep4_update() {
    float ep4_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle_EP4, ep4_throttle);

    if (FD1_uart_msg_ep4.initialized()) {
        FD1_uart_msg_ep4.read();
        FD1_uart_ep4_handle_and_route();
        FD1_uart_ep4_send();
        FD1_uart_msg_ep4.write();
    }

    // time out check for msg from ep4, will be sent via uart (optionally) and mav
    if (FD1_uart_msg_ep4_route.get_msg_ep4_in()._msg_1.updated) {
        if (ep4_state.last_update_ms > 0 && millis()-ep4_state.last_update_ms > 2000) {
            FD1_uart_msg_ep4_route.get_msg_ep4_in()._msg_1.updated = false;
        }
    }

    // route to uart
    if (FD1_uart_msg_ep4_route.initialized()) {
        FD1_uart_msg_ep4_route.write();
    }
}

void Plane::FD1_uart_ep4_handle_and_route() {
    FD1_msg_ep4_in &tmp_msg = FD1_uart_msg_ep4.get_msg_ep4_in();
    if (tmp_msg._msg_1.updated) {
        ep4_state.last_update_ms = millis();

        // copy to FD1_uart_msg_ep4_route for following uart and mav uses
        memcpy(FD1_uart_msg_ep4_route.get_msg_ep4_in()._msg_1.content.data, 
            FD1_uart_msg_ep4.get_msg_ep4_in()._msg_1.content.data, 
            FD1_uart_msg_ep4.get_msg_ep4_in()._msg_1.length*sizeof(uint8_t));
        FD1_uart_msg_ep4_route.get_msg_ep4_in()._msg_1.updated = true;
        FD1_uart_msg_ep4_route.get_msg_ep4_in()._msg_1.need_send = true;

        tmp_msg._msg_1.updated = false;
    }
}

void Plane::FD1_uart_ep4_send() {
    float ep4_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    static uint8_t n_count = 0;

    FD1_msg_ep4_out &tmp_msg = FD1_uart_msg_ep4.get_msg_ep4_out();

    float ep4_throttle_min = 0.0f;
    if (arming.is_armed()) {
        if (control_mode != &mode_manual) {
            ep4_throttle_min = constrain_float(g2.ep4_throttle_min, 0.0f, 50.f);
        }
    }
    ep4_throttle = constrain_float(ep4_throttle, ep4_throttle_min, 100.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle_EP4, ep4_throttle);

    tmp_msg._msg_1.content.msg.sum_check = 0;
    tmp_msg._msg_1.content.msg.damper = (uint16_t)(ep4_throttle*10.0f);
    tmp_msg._msg_1.content.msg.empty1 = 0;
    tmp_msg._msg_1.content.msg.count = n_count;
    tmp_msg._msg_1.content.msg.header.head_1 = FD1_msg_ep4_out::PREAMBLE1;
    tmp_msg._msg_1.content.msg.header.head_2 = FD1_msg_ep4_out::PREAMBLE2;

    for (int8_t i = 2; i < tmp_msg._msg_1.length - 1; i++) {
        tmp_msg._msg_1.content.msg.sum_check += tmp_msg._msg_1.content.data[i];
    }
    tmp_msg._msg_1.need_send = true;
    n_count++;
}


// for mavlink
void Plane::send_ep4_ecu(mavlink_channel_t chan) {
    FD1_msg_ep4_in &tmp_msg = plane.FD1_uart_msg_ep4_route.get_msg_ep4_in();
    if (tmp_msg._msg_1.updated) {
        tmp_msg.swap_message();
        mavlink_msg_ep4_ecu_send(chan, 0, 0, tmp_msg._msg_1.length, tmp_msg._msg_1.content.data);
        tmp_msg.swap_message();
    }
}

void Plane::send_esc_telemetry_mavlink_fake(mavlink_channel_t chan) {
    FD1_msg_ep4_in &tmp_msg = plane.FD1_uart_msg_ep4_route.get_msg_ep4_in();
    if (tmp_msg._msg_1.updated) {
        // arrays to hold output
        uint8_t temperature[4] {};
        uint16_t voltage[4] {};
        uint16_t current[4] {};
        uint16_t current_tot[4] {};
        uint16_t rpm[4] {};
        uint16_t count[4] {};

        rpm[0] = tmp_msg._msg_1.content.msg.rpm;
        rpm[1] = MAX(tmp_msg._msg_1.content.msg.fuel_pressure, 0);
        voltage[0] = (uint16_t)(MAX(tmp_msg._msg_1.content.msg.cylinder_temp1, 0));
        voltage[1] = (uint16_t)(MAX(tmp_msg._msg_1.content.msg.cylinder_temp2, 0));
        current[0] = (uint16_t)(MAX(tmp_msg._msg_1.content.msg.venting_temp1, 0));
        current[1] = (uint16_t)(MAX(tmp_msg._msg_1.content.msg.venting_temp2, 0));

        mavlink_msg_esc_telemetry_5_to_8_send(chan, temperature, voltage, current, current_tot, rpm, count);
    }
}
