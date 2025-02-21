#include "Copter.h"

void UMav::read_bsq_message() 
{
    if (!FD_uart_bsq.initialized()) {return;}

    // static uint32_t last_update_ms = millis();
    // uint32_t tnow = millis();
    // static uint32_t pk0_count = 0;
    // static uint32_t pk1_count = 0;
    // static uint32_t pk2_count = 0;
    while (FD_uart_bsq.get_port()->available()>0) {
        uint8_t temp = FD_uart_bsq.get_port()->read();
        // if (temp == 0xFE) pk0_count++;
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t ret = mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status, temp, &msg, &status);

        if (ret == MAVLINK_FRAMING_OK) {
            gcs().send_text(MAV_SEVERITY_INFO, "MSG");
            handle_bsq_msg(msg);
        }
    }
}

void UMav::handle_bsq_msg(const mavlink_message_t &msg)
{
    //self check cmd 400;
    trans_selfcheck.handle_bsq_msg(msg);
    trans_target.handle_bsq_msg(msg);
    trans_mission.handle_bsq_msg(msg);
    trans_relay_positon.handle_bsq_msg(msg);
}

void UMav::send_bsq_message(const mavlink_message_t &msg, uint16_t len)
{
    if (!FD_uart_bsq.initialized()) {return;}
    mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    uint8_t saved_seq = chan0_status->current_tx_seq;
    uint8_t saved_flags = chan0_status->flags;
    chan0_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    //chan0_status->current_tx_seq = FD1_mav.mavlink.seq;

    FD_uart_bsq.get_port()->write(&msg.magic, 2);
    FD_uart_bsq.get_port()->write(&msg.magic+4, 4);
    FD_uart_bsq.get_port()->write(&msg.magic+10, len-6);

    chan0_status->current_tx_seq = saved_seq;
    chan0_status->flags = saved_flags;
}
