#include "FD1_msg_lutan_in.h"
#include <GCS_MAVLink/GCS.h>

FD1_msg_lutan_in::FD1_msg_lutan_in(void)
{
    _enable = false;
    _msg_1.need_send = false;
    _msg_1.updated = false;
    init_crc_table();
}

void FD1_msg_lutan_in::parse(uint8_t temp)
{
    switch (_msg.msg_state)
    {
        default:
        case FD1UART_msg_parser::FD1UART_PREAMBLE1:
            _msg.read = 0;
            _msg.data[0] = temp;
            if (temp == PREAMBLE1) {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE2;
            }
            break;
        case FD1UART_msg_parser::FD1UART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.length = _msg_1.length;
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
            _msg.read++;

            if (_msg.read >= (_msg.length - 4))
            {
                _msg.msg_state = FD1UART_msg_parser::FD1UART_SUM;
                _msg.sum_crc32 = 0;
            }
            break;
        case FD1UART_msg_parser::FD1UART_SUM:
            _msg.data[_msg.read] = temp;
            _msg.read++;
            if (_msg.read >= _msg.length) 
            {
                _msg.sum_crc32 = ( ((uint32_t)_msg.data[_msg.read-4] << 24) | ((uint32_t)_msg.data[_msg.read-3] << 16) | ((uint32_t)_msg.data[_msg.read-2] << 8) | ((uint32_t)_msg.data[_msg.read-1] << 0) );
                uint32_t crc32_in = xcrc32(_msg.data + 2, _msg.length - 6);
                // gcs().send_text(MAV_SEVERITY_INFO, "%x %x %x %x | %d - %d", _msg.data[_msg.read-4], _msg.data[_msg.read-3], _msg.data[_msg.read-2], _msg.data[_msg.read-1], _msg.sum_crc32, crc32_in);
                // gcs().send_text(MAV_SEVERITY_INFO, "%x %x %x %x | %d - %d", _msg.data[_msg.read-4], _msg.data[_msg.read-3], _msg.data[_msg.read-2], _msg.data[_msg.read-1], swap_message_uint32_t(_msg.sum_crc32), crc32_in);
                if (_msg.sum_crc32 == crc32_in) {
                    process_message();
                }
                _msg.msg_state = FD1UART_msg_parser::FD1UART_PREAMBLE1;
            }
            break;
    }
}

void FD1_msg_lutan_in::process_message(void)
{
    int16_t i = 0;

    for (i = 0; i < _msg_1.length; i ++) {
        _msg_1.content.data[i] = _msg.data[i];
    }
    swap_message();
    // gcs().send_text(MAV_SEVERITY_INFO, "cool %d", _msg_1.content.msg.coolant);
    _msg_1.updated = true;
    _msg_1.need_send = false;
    _msg_1.print = true;
}

void FD1_msg_lutan_in::swap_message(void)
{
    _msg_1.content.msg.coolant = swap_message_int16_t(_msg_1.content.msg.coolant);
}

void FD1_msg_lutan_in::init_crc_table(void)
{
    uint32_t c;
    uint32_t i, j;
    
    for (i = 0; i < 256; i++) {
        c = (uint32_t)i;
        for (j = 0; j < 8; j++) {
            if (c & 1) {
                c = 0xedb88320L ^ (c >> 1);
            }
            else {
                c = c >> 1;
            }
        }
        crc_table[i] = c;
    }
}

uint32_t FD1_msg_lutan_in::xcrc32(const uint8_t* ptr, uint32_t size)
{
    // gcs().send_text(MAV_SEVERITY_INFO, "len %d %x,%x,%x,%x", size, ptr[0], ptr[1], ptr[2], ptr[size]);
    // uint32_t crcTable[256], crcTmp1;
 
    // // 动态生成CRC-32表
    // for (int i = 0; i<256; i++)
    // {
    //     crcTmp1 = i;
    //     for (int j = 8; j>0; j--)
    //     {
    //         if (crcTmp1 & 1) crcTmp1 = (crcTmp1 >> 1) ^ 0xEDB88320L;
    //         else crcTmp1 >>= 1;
    //     }
    //     crcTable[i] = crcTmp1;
    // }
 
    // 计算CRC32值
    uint32_t crcTmp2 = 0xFFFFFFFF;
    while (size--)
    {
        crcTmp2 = ((crcTmp2 >> 8) & 0x00FFFFFF) ^ crc_table[(crcTmp2 ^ (*ptr)) & 0xFF];
        ptr++;
    }
    return (crcTmp2 ^ 0xFFFFFFFF);
}
