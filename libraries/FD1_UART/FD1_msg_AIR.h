#include "FD1_message.h"

#define FD1_MSG_AIR_LEN 77
class FD1_msg_AIR : public FD1_message{
public:
    struct PACKED FD1_msg_header {
        uint8_t head_1;
        uint8_t head_2;
    };

    // message structure
    struct PACKED MSG_Command_1 {
        FD1_msg_header header; //同步头
        uint8_t count;                      //帧计数
        uint32_t psi;                       //指示静压             1/1024, [14.0kPa, 132.0kPa]
        uint32_t ps;                        //真实静压             1/1024, [14.0kPa, 132.0kPa] //11
        int32_t qci;                        //指示动压             1/1024, [-0.12kPa, 8.16Pa]
        int32_t qc;                         //真实动压             1/1024, [-0.12kPa, 8.16Pa]
        int32_t hp;                         //气压高度             1/16, [-720.0m, 13200.0m]
        int16_t hpr;                        //升降速度             1/16, [-360.0m/s, 360.0m/s]
        int16_t ts;                         //静温                 1/16, [-72.0℃, 96.0℃] //27
        int16_t tt;                         //总温                 1/16, [-72.0℃, 132.0℃]
        uint16_t mi;                        //马赫数               1/1024, [0.0, 0.7404]
        uint16_t vi;                        //指示空速              1/64, [0.0km/h, 444.0km/h]
        uint16_t vt;                        //真空速                1/64, [0.0km/h, 996.0km/h]
        uint16_t adr;                       //大气密度比            1/256, [0.1274, 1.7412]
        int16_t aoai1;                      //局部攻角1             1/128, [-20.5°, 30.5°]
        int16_t aoai2;                      //局部攻角2             1/128, [-20.5°, 30.5°]
        int16_t aoat1;                      //真攻角1               1/128, [-20.5°, 30.5°]
        int16_t aoat2;                      //真攻角2               1/128, [-20.5°, 30.5°]
        int16_t aosi1;                      //局部侧滑角1           1/128, [-20.5°, 20.5°]
        int16_t aosi2;                      //局部侧滑角2           1/128, [-20.5°, 20.5°]
        int16_t aost1;                      //真侧滑角1             1/128, [-20.5°, 20.5°]
        int16_t aost2;                      //真侧滑角2             1/128, [-20.5°, 20.5°]
        uint16_t faultword;
        uint32_t datavalid;
        uint16_t coffpress_k0;              //压力校准系数K0        1/256, [-128.0, 127.0]
        uint16_t coffpress_k1;              //压力校准系数K1        1/256, [-128.0, 127.0]
        uint16_t coffpress_k2;              //压力校准系数K2        1/256, [-128.0, 127.0]
        uint16_t coffpress_k3;              //压力校准系数K3        1/256, [-128.0, 127.0]
        int16_t coffangle_k0;               //攻角侧滑角校准系数K0   1/1024, [-32.0, 31.0]
        int16_t coffangle_k1;               //攻角侧滑角校准系数K1   1/1024, [-32.0, 31.0]
        int16_t coffangle_k2;               //攻角侧滑角校准系数K2   1/1024, [-32.0, 31.0]
        int16_t coffangle_k3;               //攻角侧滑角校准系数K3   1/1024, [-32.0, 31.0]
        uint16_t sum_check;
    };

    union PACKED Content_1 {
        MSG_Command_1 msg;
        uint8_t data[FD1_MSG_AIR_LEN];
    };

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // message structure
    struct PACKED FD1UART_MSG_1 {
        bool print;
        bool updated;
        bool need_send;
        const uint16_t length = FD1_MSG_AIR_LEN;
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
            FD1UART_SUM1,
            FD1UART_SUM2,
        } msg_state;

        uint16_t length;
        uint16_t read;
        uint16_t sum_check;
        uint8_t data[FD1_MSG_AIR_LEN];
    } _msg;

    FD1_msg_AIR();
    
    /* Do not allow copies */
    FD1_msg_AIR(const FD1_msg_AIR &other) = delete;
    FD1_msg_AIR &operator=(const FD1_msg_AIR&) = delete;

    static const uint8_t PREAMBLE1 = 0x55;
    static const uint8_t PREAMBLE2 = 0xAA;

    void process_message(void) override;
    void parse(uint8_t temp) override;
    void swap_message() override;
    void sum_check() override {;} 

    FD1UART_MSG_1 _msg_1;
};
