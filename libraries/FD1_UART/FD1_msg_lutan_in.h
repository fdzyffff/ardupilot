#include "FD1_message.h"

#define FD1_MSG_LUTAN_IN_LEN 88
class FD1_msg_lutan_in : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header;
        uint8_t flag;
                                // Offset| Size| Sign?| Name           | Function                     | Units   | Mult|Divide|
        uint16_t seconds;       // 0     | 2   | N    | seconds        | Seconds ECU has been on      | s       | 1   | 1    |
        uint16_t pulseWidth1;   // 2     | 2   | N    | pulseWidth1    | Main pulsewidth bank 1       | ms      | 1   | 1000 |
        uint16_t pulseWidth2;   // 4     | 2   | N    | pulseWidth2    | Main pulsewidth bank 2       | ms      | 1   | 1000 |
        uint16_t rpm;           // 6     | 2   | N    | rpm            | Engine RPM                   | RPM     | 1   | 1    |
        int16_t advance[4];     // 8     | 2   | Y    | advance        | Final ignition spark advance | deg BTDC| 1   | 10   |
        int16_t barometer;      // 16    | 2   | Y    | barometer      | Barometric pressure          | kPa     | 1   | 10   |
        int16_t map;            // 18    | 2   | Y    | map            | Manifold air pressure        | kPa     | 1   | 10   |
        int16_t mat;            // 20    | 2   | Y    | mat            | Manifold air temperature     | deg F   | 1   | 10   |
        int16_t coolant;        // 22    | 2   | Y    | coolant        | Coolant temperature          | deg F   | 1   | 10   |
        int16_t tps;            // 24    | 2   | Y    | tps            | Throttle position            | %       | 1   | 10   |
        int16_t batteryVoltage; // 26    | 2   | Y    | batteryVoltage | Battery voltage              | V       | 1   | 10   |
        int16_t afr1;           // 28    | 2   | Y    | afr1           | AFR1                         | AFR     | 1   | 10   |
        int16_t afr2;           // 30    | 2   | Y    | afr2           | AFR2                         | AFR     | 1   | 10   |
        uint8_t empty_1[14];
        int16_t baroCorrection; // 46    | 2   | Y    | baroCorrection | Barometric fuel correction   | %       | 1   | 10   |
        int16_t gammaEnrich;    // 48    | 2   | Y    | gammaEnrich    | Total fuel correction        | %       | 1   | 10   |
        int16_t ve1[3];         // 50    | 2   | Y    | ve1            | VE value table/bank 1        | %       | 1   | 10   |
        int16_t cold_adv_deg;   // 56    | 2   | Y    | cold_adv_deg   | Cold advance                 | deg     | 1   | 10   |
        int16_t TPSdot;         // 58    | 2   | Y    | TPSdot         | Rate of change of TPS        | %/s     | 1   | 10   |
        int16_t MAPdot;         // 60    | 2   | Y    | MAPdot         | Rate of change of MA P       | kPa/s   | 1   | 10   |
        uint8_t empty_2[12];
        int16_t egoV1;          // 74    | 2   | Y    | egoV1          | Voltage from O2#1            | V       | 1   | 100  |
        int16_t egoV2;          // 76    | 2   | Y    | egoV2          | Voltage from O2#2            | V       | 1   | 100  |

        uint8_t empty_3[3];
        uint32_t crc32;

    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_LUTAN_IN_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_LUTAN_IN_LEN;
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
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint32_t sum_crc32;
        uint8_t data[FD1_MSG_LUTAN_IN_LEN];
    } _msg;

    FD1_msg_lutan_in();
    
    /* Do not allow copies */
    FD1_msg_lutan_in(const FD1_msg_lutan_in &other) = delete;
    FD1_msg_lutan_in &operator=(const FD1_msg_lutan_in&) = delete;

    static const uint8_t PREAMBLE1 = 0x00;
    static const uint8_t PREAMBLE2 = 0x52;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;

    FD1UART_MSG_1 _msg_1;

    void init_crc_table();
    uint32_t xcrc32(const uint8_t* ptr, uint32_t size);
    uint32_t crc_table[256];
};
