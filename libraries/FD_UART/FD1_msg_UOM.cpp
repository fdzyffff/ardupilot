#include "FD1_msg_UOM.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <cstdio>

extern const AP_HAL::HAL &hal;

FD1_msg_UOM::FD1_msg_UOM(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_UOM::parse(uint8_t temp)
{
    ;
}

void FD1_msg_UOM::process_message(void)
{
    ;
}

void FD1_msg_UOM::make_sum()
{
    ;
}

void FD1_msg_UOM::make_init()
{
    _msg_1.content.msg.type = 255;
    _msg_1.content.msg.version = 1;
    _msg_1.content.msg.length = 0;
    _msg_1.content.msg.all_msg_data[0] = 0;
    _msg_1.length = 4;
}

void FD1_msg_UOM::swap_message(void)
{
    ;
}

bool FD1_msg_UOM::have_msg_id(uint8_t msg_id, uint8_t (&msg_mask)[4])
{
    uint8_t current_mask[4] = {0};
    get_msg_mask(msg_id, current_mask);
    return have_msg_mask(msg_mask, current_mask);
}

bool FD1_msg_UOM::have_msg_mask(uint8_t (&msg_mask)[4], uint8_t (&current_mask)[4])
{
    for (uint8_t i_mask = 0; i_mask < 4; i_mask ++) {
        if (msg_mask[i_mask] & current_mask[i_mask]) {
            return true;
        }
    }
    return false;
}

void FD1_msg_UOM::get_msg_mask(uint8_t msg_id, uint8_t (&msg_mask)[4])
{
    uint8_t shift_byte = (msg_id - 1) / 7;
    uint8_t shift_bit = 7 - ((msg_id - 1) % 7);
    memset(msg_mask, 0, 4);
    msg_mask[shift_byte] = 1 << (shift_bit);
}

uint8_t FD1_msg_UOM::get_msg_length(uint8_t msg_id)
{
    switch (msg_id) {
        case 1:
            return 20;
        case 2:
            return 8;
        case 3:
            return 1;
        case 4:
            return 1;
        case 5:
            return 1;
        case 6:
            return 8;
        case 7:
            return 2;
        case 8:
            return 8;
        case 9:
            return 2;
        case 10:
            return 2;
        case 11:
            return 2;
        case 12:
            return 1;
        case 13:
            return 2;
        case 14:
            return 2;
        case 15:
            return 1;
        case 16:
            return 1;
        case 17:
            return 1;
        case 18:
            return 1;
        case 19:
            return 1;
        case 20:
            return 6;
        case 21:
            return 1;
        default:
            return 0;
    }
}

void FD1_msg_UOM::insert_msg(uint8_t msg_id, uint8_t (&msg_data)[20])
{
    uint8_t msg_mask_length = 1;

    //check mask 1
    if (_msg_1.content.msg.all_msg_data[0] & 0x01) {
        msg_mask_length = 2;
    }

    //check mask 2
    if ((msg_mask_length == 2) && (_msg_1.content.msg.all_msg_data[1] & 0x01)) {
        msg_mask_length = 3;
    }

    //check mask 3
    if ((msg_mask_length == 3) && (_msg_1.content.msg.all_msg_data[2] & 0x01)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Bad mask");
        return;
    }

    uint8_t msg_mask[4] = {0};

    memcpy((uint8_t *)&msg_mask, (uint8_t *)&_msg_1.content.msg.all_msg_data[0], msg_mask_length);  

    uint8_t msg_length = msg_mask_length;

    for (uint8_t i_msg = 1; i_msg < 22; i_msg++) {
        if (i_msg == msg_id) {
            if (have_msg_id(i_msg, msg_mask)) {
                // get legnth of insert byte
                uint8_t insert_byte = get_msg_length(i_msg);
                for (uint8_t i_i = 0; i_i < insert_byte; i_i++) {
                    _msg_1.content.data[3 + msg_length + i_i] = msg_data[i_i];
                }
            } else {
                // get legnth of insert byte
                uint8_t insert_byte = get_msg_length(i_msg);
                for (uint8_t i_i = _msg_1.length; i_i >= msg_length + 3; i_i--) {
                    _msg_1.content.data[i_i + insert_byte] = _msg_1.content.data[i_i];
                }

                // shift back current buffer for next step
                for (uint8_t i_j = 0; i_j < insert_byte; i_j++) {
                    _msg_1.content.data[msg_length + 3 + i_j] = msg_data[i_j];
                }

                _msg_1.length += insert_byte;
                _msg_1.content.msg.length += insert_byte;

                // update the mask
                uint8_t shift_byte = (msg_id - 1) / 7;
                uint8_t shift_bit = 7 - ((msg_id - 1) % 7);
                uint8_t current_mask[4] = {0};
                current_mask[shift_byte] = 1 << (shift_bit);

                uint8_t insert_mask_byte = shift_byte - (msg_mask_length - 1);

                if (insert_mask_byte > 0) {
                    // consider add byte for new mask
                    for (uint8_t i_k = _msg_1.length; i_k >= (3 + msg_length); i_k--) {
                        _msg_1.content.data[i_k + insert_mask_byte] = _msg_1.content.data[i_k];
                    }
                    for (uint8_t i_l = 0; i_l < insert_mask_byte; i_l++) {
                        _msg_1.content.data[3 + msg_mask_length + i_l] = current_mask[msg_mask_length + i_l];
                        _msg_1.content.data[3 + msg_mask_length + i_l - 1] |= 0x01;
                    }
                    _msg_1.length += insert_mask_byte;
                } else {
                    _msg_1.content.data[3 + shift_byte] |= current_mask[shift_byte];
                }
            }
            break;
        }

        if (have_msg_id(i_msg, msg_mask)) {
            msg_length += get_msg_length(i_msg);
        }
    }
    // printf("_msg_1.content.msg.length %d\n", _msg_1.content.msg.length);
}
