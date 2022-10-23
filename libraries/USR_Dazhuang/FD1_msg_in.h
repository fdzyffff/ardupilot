#include "FD1_message.h"

#define FD1_MSG_IN_LEN 54
class FD1_msg_in : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;
        uint8_t  ecu_status;
        uint16_t damper;          //unit: 0.1%, value = real * 10
        uint16_t rpm;             //0~12000rpm
        int16_t  cylinder_temp1;  //0.1 degree
        int16_t  cylinder_temp2;  //0.1 degree
        int16_t  venting_temp1;   //1 degree
        int16_t  venting_temp2;   //1 degree
        int16_t  fuel_pressure;   //1 mbar
        uint16_t fuel_time;       //micro second
        uint16_t inite_angle;     //0.1 degree
        int8_t   baro_temp;       //1 degree
        uint16_t baro_pressure;   //1 mbar
        uint16_t ecu_power_volt;  //0.1 V
        uint8_t  empty1;          //empty byte
        uint8_t  ecu_inner_volt;  //0.1 V
        int16_t  ecu_temp;        //0.1 degree
        char ecu_type[8];         //fixed value
        char ecu_version[8];      //fixed value
        uint32_t ecu_serial;      //fixed value
        uint16_t warning_code;    //warning code
        uint8_t  count;           //0~255
        uint8_t  sum_check;       //byte 3 to 53
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_IN_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_IN_LEN;
        Content_1 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED FD1UART_msg_parser
    {
        enum
        {
            FD1UART_PREAMBLE1 = 0,
            FD1UART_PREAMBLE2,
            FD1UART_DATA,
            FD1UART_SUM,
            FD1UART_END,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint8_t sum_check;
        uint8_t data[FD1_MSG_IN_LEN];
    } _msg;

    FD1_msg_in();
    
    /* Do not allow copies */
    // FD1_msg_in(const FD1_msg_in &other) = delete;
    // FD1_msg_in &operator=(const FD1_msg_in&) = delete;

    static const uint8_t PREAMBLE1 = 0x55;
    static const uint8_t PREAMBLE2 = 0xAA;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    FD1UART_MSG_1 _msg_1;
};
