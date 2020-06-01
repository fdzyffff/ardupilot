
#define AP_YANGUART_BAUD        115200
#define AP_YANGUART_BUFSIZE_RX  256
#define AP_YANGUART_BUFSIZE_TX  256

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "YANG_UART.h"

extern const AP_HAL::HAL& hal;


YANG_UART::YANG_UART(void)
{
    _protocol = AP_SerialManager::SerialProtocol_None;
    _port = NULL;
    _initialized = false;
}

/*
 * init - perform required initialisation
 */
bool YANG_UART::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have FrSky on multiple serial ports)
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_YANGUart, 0))) {
        _protocol = AP_SerialManager::SerialProtocol_YANGUart; 
        // we don't want flow control for either protocol
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        //Initialize the uart
        _port->begin(AP_YANGUART_BAUD,AP_YANGUART_BUFSIZE_RX, AP_YANGUART_BUFSIZE_TX);
        _initialized = true;
    }
    return _initialized;
}


bool YANG_UART::update(void)
{
    if(!initialized()) {
        return false;
    }
    read();
    write();
    return true;
}

void YANG_UART::read(void)
{
    while (_port->available() > 0) {
        uint8_t temp = _port->read();
        parse(temp);
    }
}

void YANG_UART::parse(uint8_t temp)
{
    switch (_msg.msg_state)
    {
        default:
        case YANGUART_msg_parser::YANGUART_PREAMBLE1:
            _msg.read = 0;
            _msg.sum_check = 0;
            if (temp == PREAMBLE1) {
                _msg.msg_state = YANGUART_msg_parser::YANGUART_PREAMBLE2;
            }
            break;
        case YANGUART_msg_parser::YANGUART_PREAMBLE2:
            if (temp == PREAMBLE2)
            {
                _msg.msg_state = YANGUART_msg_parser::YANGUART_INDEX;
            }
            else
            {
                _msg.msg_state = YANGUART_msg_parser::YANGUART_PREAMBLE1;
            }
            break;
        case YANGUART_msg_parser::YANGUART_INDEX:
            _msg.header.head_1 = PREAMBLE1;
            _msg.header.head_2 = PREAMBLE2;
            _msg.sum_check += temp;
            _msg.header.index = temp;
            switch (_msg.header.index) {
                case YANG_UART_INDEX1:
                    _msg.length = _msg_1.length;
                    _msg.read = 3;
                    _msg.msg_state = YANGUART_msg_parser::YANGUART_DATA;
                    break;
                case YANG_UART_INDEX2:
                    _msg.length = _msg_2.length;
                    _msg.read = 3;
                    _msg.msg_state = YANGUART_msg_parser::YANGUART_DATA;
                    break;
                default:
                    _msg.msg_state = YANGUART_msg_parser::YANGUART_PREAMBLE1;
                    break;
            }
            break;

        case YANGUART_msg_parser::YANGUART_DATA:
            if (_msg.read >= sizeof(_msg.data)) {
                _msg.msg_state = YANGUART_msg_parser::YANGUART_PREAMBLE1;
                break;
            }
            _msg.data[_msg.read-3] = temp;

            _msg.sum_check += temp;

            _msg.read++;
            if (_msg.read >= (_msg.length - 1))
            {
                _msg.msg_state = YANGUART_msg_parser::YANGUART_CRC1;
            }
            break;
        case YANGUART_msg_parser::YANGUART_CRC1:
            _msg.data[_msg.read-3] = temp;
            _msg.msg_state = YANGUART_msg_parser::YANGUART_PREAMBLE1;

            if (_msg.sum_check == temp)
            {
                process_message();
            }
            break;
    }
}

void YANG_UART::process_message(void)
{
    int16_t i = 0;
    switch (_msg.header.index) {
        case INDEX1:
            _msg_1.content.data[0] = _msg.header.head_1;
            _msg_1.content.data[1] = _msg.header.head_2;
            _msg_1.content.data[2] = _msg.header.index;
            for (i = 0; i < _msg_1.length - 3; i ++) {
                _msg_1.content.data[i+3] = _msg.data[i];
            }
            _msg_1.updated = true;
            break;
        case INDEX2:
            _msg_2.content.data[0] = _msg.header.head_1;
            _msg_2.content.data[1] = _msg.header.head_2;
            _msg_2.content.data[2] = _msg.header.index;
            for (i = 0; i < _msg_2.length - 3; i ++) {
                _msg_2.content.data[i+3] = _msg.data[i];
            }
            _msg_2.updated = true;
            break;
        default:
            break;
    }

}

void YANG_UART::write(void)
{
    if(!initialized()) {
        return ;
    }
    int16_t i = 0;
    if (_msg_1.updated)
    {
        for(i = 0;i < _msg_1.length ; i ++)
        {
            _port->write(_msg_1.content.data[i]);
        }
        _msg_1.updated = false;
    }
    if (_msg_2.updated)
    {
        for(i = 0;i < _msg_2.length ; i ++)
        {
            _port->write(_msg_2.content.data[i]);
        }
        _msg_2.updated = false;
    }
    if (_msg_3.updated)
    {
        for(i = 0;i < _msg_3.length ; i ++)
        {
            _port->write(_msg_3.content.data[i]);
        }
        _msg_3.updated = false;
    }
}

