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
        // gcs().send_text(MAV_SEVERITY_INFO, "MSG XX");
        // if (temp == 0xFE) pk0_count++;
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t ret = mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status, temp, &msg, &status);

        if (ret >= MAVLINK_FRAMING_OK) {
            if (ret > MAVLINK_FRAMING_OK) {
                gcs().send_text(MAV_SEVERITY_INFO, "BAD MSG");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "MSG");
            }
            handle_bsq_msg(msg);
        }
    }
}

void UMav::handle_bsq_msg(const mavlink_message_t &msg)
{
    trans_selfcheck.handle_bsq_msg(msg);
    trans_target.handle_bsq_msg(msg);
    trans_mission.handle_bsq_msg(msg);
    trans_relay_positon.handle_bsq_msg(msg);
}

// void UMav::send_bsq_message(const mavlink_message_t &msg, uint16_t len)
// {
//     if (!FD_uart_bsq.initialized()) {return;}
//     mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
//     uint8_t saved_seq = chan0_status->current_tx_seq;
//     uint8_t saved_flags = chan0_status->flags;
//     chan0_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
//     //chan0_status->current_tx_seq = FD1_mav.mavlink.seq;

//     FD_uart_bsq.get_port()->write(&msg.magic, 2);
//     FD_uart_bsq.get_port()->write(&msg.magic+4, 4);
//     FD_uart_bsq.get_port()->write(&msg.magic+10, len-6);

//     chan0_status->current_tx_seq = saved_seq;
//     chan0_status->flags = saved_flags;
// }
void UMav::send_bsq_message(mavlink_message_t *msg)
{
    if (!FD_uart_bsq.initialized()) {return;}

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
        FD_uart_bsq.get_port()->write(buf, header_len);
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
        FD_uart_bsq.get_port()->write(buf, header_len);
    }

    FD_uart_bsq.get_port()->write((uint8_t *)_MAV_PAYLOAD(msg), msg->len);
    FD_uart_bsq.get_port()->write((uint8_t *)ck, 2);
}
