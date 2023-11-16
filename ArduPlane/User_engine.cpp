#include "Plane.h"
#include "Plane.h"

void Plane::engine_init()
{
    uengine.init();
}

void Plane::engine_update()
{
    engine_quad_state_update();
    uengine.update();
}

void Plane::engine_quad_state_update() {
    static bool in_vtol = plane.quadplane.in_vtol_mode();
    if (in_vtol != plane.quadplane.in_vtol_mode()) {
        if (plane.arming.is_armed() && plane.is_flying()) {
            if (plane.quadplane.in_vtol_mode()) {
                uengine.stop();
            } else {
                uengine.start();
            }
        }
        in_vtol = plane.quadplane.in_vtol_mode();
    }
}

// for mavlink
void Plane::send_engine_ecu(mavlink_channel_t chan) {
    // FD1_msg_ep4_in &tmp_msg = plane.ep4_ctrl.uart_msg_ep4_route.get_msg_ep4_in();
    // // gcs().send_text(MAV_SEVERITY_INFO, "update, %d", tmp_msg._msg_1.updated);
    // if (tmp_msg._msg_1.updated) {
    //     tmp_msg.swap_message();
    //     mavlink_msg_ep4_ecu_send(chan, 0, 0, tmp_msg._msg_1.length, tmp_msg._msg_1.content.data);
    //     tmp_msg.swap_message();
    // }
}

void Plane::send_esc_telemetry_mavlink_fake(mavlink_channel_t chan) {
    if (uengine.valid()) {
        // arrays to hold output
        uint8_t temperature[4] {};
        uint16_t voltage[4] {};
        uint16_t current[4] {};
        uint16_t current_tot[4] {};
        uint16_t rpm[4] {};
        uint16_t count[4] {};

        rpm[0] = uengine.status.rpm;
        rpm[1] = MAX(uengine.status.fuel_pressure, 0);
        voltage[0] = (uint16_t)(MAX(uengine.status.cylinder_temp1, 0));
        voltage[1] = (uint16_t)(MAX(uengine.status.cylinder_temp2, 0));
        current[0] = (uint16_t)(MAX(uengine.status.venting_temp1, 0));
        current[1] = (uint16_t)(MAX(uengine.status.venting_temp2, 0));
        mavlink_msg_esc_telemetry_5_to_8_send(chan, temperature, voltage, current, current_tot, rpm, count);
    }
}
