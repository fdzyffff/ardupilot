#include "FD1_msg_SIYIA8mini.h"
#include <GCS_MAVLink/GCS.h>

FD1_msg_SIYIA8mini::FD1_msg_SIYIA8mini(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
}

void FD1_msg_SIYIA8mini::parse(uint8_t temp)
{
    // gcs().send_text(MAV_SEVERITY_INFO, "State: %d, Byte: %d",_msg.msg_state, temp);
    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.sum_check = 0;
            _msg.data[0] = temp;
            if (temp == PREAMBLE1) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
            }
            break;
        case FD1UART_msg_parser::FD1UART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.length = 0;
                _msg.read = 2;
                _msg.msg_state = FD1UART_msg_parser::FD1UART_DATA;
                _msg.data[1] = temp;
            }
            else
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            break;
        case FD1UART_msg_parser::FD1UART_DATA:
            if (_msg.read >= sizeof(_msg.data)-1) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
                break;
            }
            _msg.data[_msg.read] = temp;

            if (_msg.read == 4) {
                _msg.length = ((uint16_t)_msg.data[4] << 8) + ((uint16_t)_msg.data[3]);
            }

            if (_msg.read == _msg.length + 10 -1) {
                _msg.sum_check = ((uint16_t)_msg.data[_msg.length + 10 -1] << 8) + ((uint16_t)_msg.data[_msg.length + 10 - 2]);
            }
            _msg.read++;

            if (_msg.read >= (_msg.length + 10))
            {
                uint16_t data_length = _msg.length + 10 - 2;
                uint16_t crc16 = CRC16_cal(_msg.data, data_length, 0);
                if (crc16 == _msg.sum_check)
                {
                    process_message();
                }
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            break;
    }
}

void FD1_msg_SIYIA8mini::process_message(void)
{
    int16_t i = 0;

    for (i = 0; i < _msg.length + 10; i ++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD1_msg_SIYIA8mini::swap_message(void)
{
    // swap_message_sub(_msg_1.content.data[7-1] , _msg_1.content.data[8-1] );
    // swap_message_sub(_msg_1.content.data[9-1] , _msg_1.content.data[10-1] );
    // swap_message_sub(_msg_1.content.data[4-1] , _msg_1.content.data[5-1] , _msg_1.content.data[6-1] , _msg_1.content.data[7-1]);
    // swap_message_sub(_msg_1.content.data[8-1] , _msg_1.content.data[9-1] , _msg_1.content.data[10-1], _msg_1.content.data[11-1]);
    // swap_message_sub(_msg_1.content.data[12-1], _msg_1.content.data[13-1], _msg_1.content.data[14-1], _msg_1.content.data[15-1]);
    // swap_message_sub(_msg_1.content.data[16-1], _msg_1.content.data[17-1], _msg_1.content.data[18-1], _msg_1.content.data[19-1]);
    // swap_message_sub(_msg_1.content.data[20-1], _msg_1.content.data[21-1], _msg_1.content.data[22-1], _msg_1.content.data[23-1]);
    // swap_message_sub(_msg_1.content.data[24-1], _msg_1.content.data[25-1], _msg_1.content.data[26-1], _msg_1.content.data[27-1]);
}

void FD1_msg_SIYIA8mini::sum_check(void)
{
    uint16_t data_length = _msg_1.content.msg.data_length + 10 - 2;
    // uint16_t crc16 = CRC16_cal(_msg_1.content.data, data_length, 0);
    uint16_t crc16 = crc16_xmodem(_msg_1.content.data, data_length);

    _msg_1.content.data[data_length] = (uint8_t)(crc16&0xFF);
    _msg_1.content.data[data_length+1] = (uint8_t)(crc16>>8);
}

void FD1_msg_SIYIA8mini::pack_center()
{
    _msg_1.content.msg.header.head_1 = PREAMBLE1;
    _msg_1.content.msg.header.head_2 = PREAMBLE2;
    _msg_1.content.msg.ctrl = 0x01;
    _msg_1.content.msg.data_length = 1;
    _msg_1.content.msg.seq += 1;
    _msg_1.content.msg.cmd_id = 0x08;
    _msg_1.content.msg.data_region[0] = 1;

    sum_check();
}

void FD1_msg_SIYIA8mini::pack_angle(float yaw, float pitch)
{
    _msg_1.content.msg.header.head_1 = PREAMBLE1;
    _msg_1.content.msg.header.head_2 = PREAMBLE2;
    _msg_1.content.msg.ctrl = 0x01;
    _msg_1.content.msg.data_length = 4;
    _msg_1.content.msg.seq += 1;
    _msg_1.content.msg.cmd_id = 0x0E;
    int16_t yaw_cmd = (int16_t)yaw;
    int16_t pitch_cmd = (int16_t)(pitch*10.0f);
    fill_int16_t(&_msg_1.content.msg.data_region[0], yaw_cmd);
    fill_int16_t(&_msg_1.content.msg.data_region[2], pitch_cmd);

    sum_check();
}

void FD1_msg_SIYIA8mini::pack_zoom(float zoom)
{
    _msg_1.content.msg.header.head_1 = PREAMBLE1;
    _msg_1.content.msg.header.head_2 = PREAMBLE2;
    _msg_1.content.msg.ctrl = 0x01;
    _msg_1.content.msg.data_length = 2;
    _msg_1.content.msg.seq += 1;
    _msg_1.content.msg.cmd_id = 0x0F;
    if (is_negative(zoom)) {
        zoom = 1.0f;
    }
    uint8_t z1 = uint8_t(zoom/1.0f);
    uint8_t z2 = uint8_t(fmodf(zoom, 1.0f)*10.f);

    _msg_1.content.msg.data_region[0] = z1;
    _msg_1.content.msg.data_region[1] = z2;

    sum_check();
}

void FD1_msg_SIYIA8mini::pack_stabilize_mode()
{
    _msg_1.content.msg.header.head_1 = PREAMBLE1;
    _msg_1.content.msg.header.head_2 = PREAMBLE2;
    _msg_1.content.msg.ctrl = 0x01;
    _msg_1.content.msg.data_length = 1;
    _msg_1.content.msg.seq += 1;
    _msg_1.content.msg.cmd_id = 0x0C;
    _msg_1.content.msg.data_region[0] = 4;

    sum_check();
}

void FD1_msg_SIYIA8mini::pack_attitude_hz()
{
    _msg_1.content.msg.header.head_1 = PREAMBLE1;
    _msg_1.content.msg.header.head_2 = PREAMBLE2;
    _msg_1.content.msg.ctrl = 0x25;
    _msg_1.content.msg.data_length = 2;
    _msg_1.content.msg.seq += 1;
    _msg_1.content.msg.cmd_id = 0x0C;
    _msg_1.content.msg.data_region[0] = 1;
    _msg_1.content.msg.data_region[01] = 5;

    sum_check();
}



/***********************************************************
CRC16 Coding & Decoding G(X) = X^16+X^12+X^5+1
***********************************************************/

uint16_t FD1_msg_SIYIA8mini::crc16_xmodem(const uint8_t* data, size_t length)
{
    uint16_t crc = 0x0000;

    for (size_t i = 0; i < length; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;

        for (int j = 0; j < 8; ++j)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}


uint16_t FD1_msg_SIYIA8mini::CRC16_cal(uint8_t *ptr, uint32_t len, uint16_t crc_init)
{
    uint16_t crc, oldcrc16;
    uint8_t temp;
    crc = crc_init;
    while (len--!=0)
    {
        temp=(crc>>8)&0xff;
        oldcrc16=crc16_tab[*ptr^temp];
        crc=(crc<<8)^oldcrc16;
        ptr++;
    }
    //crc=~crc; //??
    return(crc);
}

uint8_t FD1_msg_SIYIA8mini::crc_check_16bites(uint8_t* pbuf, uint32_t len,uint32_t* p_result)
{
    uint16_t crc_result = 0;
    crc_result= CRC16_cal(pbuf,len, 0);
    *p_result = crc_result;
    return 2;
}
