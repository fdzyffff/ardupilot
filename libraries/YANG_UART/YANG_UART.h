#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define YANG_UART_INDEX1 0xAA
#define YANG_UART_INDEX2 0xAB
#define YANG_UART_INDEX3 0xAC

class YANG_UART {
public:
    struct PACKED YANGUART_msg_header {
        uint8_t head_1;
        uint8_t head_2;
        uint8_t index;
    };
    // message structure
    struct PACKED Remote_CMD_1 {
        uint8_t p1;
        uint8_t p2;
        uint32_t longitude;
        uint32_t latitude;
        uint16_t alt;
    };

    struct PACKED Remote_CMD_2 {
        uint8_t line_index;
        uint8_t point_index;
        uint32_t longitude;
        uint32_t latitude;
        uint16_t alt;
    };

    struct PACKED Remote_CMD_3 {
        uint8_t temp_data[12];
    };

    struct PACKED Remote_CMD_4 {
        uint8_t temp_data[12];
    };

    struct PACKED Remote_CMD_5 {
        uint8_t group_index;
        uint8_t attack_index;
    };

    struct PACKED Remote_CMD_6 {
        uint8_t address;
    };

    union PACKED Remote_CMD {
        Remote_CMD_1 cmd_1;
        Remote_CMD_2 cmd_2;
        Remote_CMD_3 cmd_3;
        Remote_CMD_4 cmd_4;
        Remote_CMD_5 cmd_5;
        Remote_CMD_6 cmd_6;
        uint8_t cmd_data[12];
    };

    // message structure
    struct PACKED MSG_Command_1 {
        YANGUART_msg_header header;
        uint8_t console_type;
        uint8_t remote_index;
        Remote_CMD remote_cmd;
        uint8_t unused[4];
        uint8_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[22];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // message structure
    struct PACKED MSG_Command_2 {

        uint8_t console_type;
        uint8_t remote_index;
        Remote_CMD remote_cmd;
        int16_t chuangheng_gnd_spd;
        int16_t chuangheng_offset;
        int16_t chuangheng_alt;
        int16_t apm_x_offset;
        int16_t apm_y_offset;
        int16_t apm_z_offset;
        uint32_t leader_longitude;
        uint32_t leader_latitude;
        uint16_t leader_alt;
        uint8_t unused[5];
        uint8_t sum_check;
    };

    union PACKED Content_2 {
        MSG_Command_2 msg;
        uint8_t data[45];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // message structure
    struct PACKED MSG_Command_3 {
        YANGUART_msg_header header;
        uint32_t longitude;
        uint32_t latitude;
        uint16_t alt;
        int16_t pitch;
        int16_t roll;
        int16_t yaw;
        int16_t gnd_spd;
        uint16_t err_status;
        uint8_t remote_status;
        uint8_t wp_status;
        uint8_t remote_index;
        uint8_t unused[3];
        uint8_t sum_check;
    };

    union PACKED Content_3 {
        MSG_Command_3 msg;
        uint8_t data[30];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED YANGUART_MSG_1 {
        bool updated;
        const uint16_t length = 22;
        Content_1 content;
    };
    // message structure
    struct PACKED YANGUART_MSG_2 {
        bool updated;
        const uint16_t length = 45;
        Content_2 content;
    };
    // message structure
    struct PACKED YANGUART_MSG_3 {
        bool updated;
        const uint16_t length = 30;
        Content_3 content;
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    struct PACKED YANGUART_msg_parser
    {
        enum
        {
            YANGUART_PREAMBLE1 = 0,
            YANGUART_PREAMBLE2,
            YANGUART_INDEX,
            YANGUART_DATA,
            YANGUART_CRC1,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint8_t sum_check;
        YANGUART_msg_header header;
        uint8_t data[60];;
    } _msg;

    YANG_UART();

    /* Do not allow copies */
    YANG_UART(const YANG_UART &other) = delete;
    YANG_UART &operator=(const YANG_UART&) = delete;

    // init - perform required initialisation
    bool init();
    bool update();
    bool initialized() {return _initialized;}

    YANGUART_MSG_1& get_msg_1() { return _msg_1; }
    YANGUART_MSG_2& get_msg_2() { return _msg_2; }
    YANGUART_MSG_3& get_msg_3() { return _msg_3; }

    static const uint8_t PREAMBLE1 = 0xEB;
    static const uint8_t PREAMBLE2 = 0x90;

    static const uint8_t INDEX1 = YANG_UART_INDEX1;
    static const uint8_t INDEX2 = YANG_UART_INDEX2;
    static const uint8_t INDEX3 = YANG_UART_INDEX3;
private:
    void read();
    void write();
    void process_message(void);
    void parse(uint8_t temp);

    AP_HAL::UARTDriver *_port;                  // UART used to send data to FrSky receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter
    bool _initialized;

    YANGUART_MSG_1 _msg_1;
    YANGUART_MSG_2 _msg_2;
    YANGUART_MSG_3 _msg_3;
};
