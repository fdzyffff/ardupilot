#pragma once

#include <AP_HAL/AP_HAL.h>
#include <FD1_UART/FD1_UART.h>
#include "Uattack.h"

class UAttack;
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class UTarget_Base {
public:
    friend class UAttack;

    UTarget_Base(UAttack &frotend_in): _frotend(frotend_in) {};
    virtual ~UTarget_Base() {};
    virtual bool init() = 0;
    virtual bool is_valid() = 0;
    virtual void update() = 0;
    virtual void do_cmd() = 0;
    virtual void handle_info(float p1, float p2) = 0;
    virtual void handle_msg(const mavlink_message_t &msg) = 0;
    virtual void handle_info_test(float p1, float p2) = 0;
    UAttack &_frotend;
    bool _valid;
};

class UTarget_Cam: public UTarget_Base {
public:
    friend class UAttack;

    UTarget_Cam(UAttack &frotend_in);
    ~UTarget_Cam() {};
    bool init() override;
    bool is_valid() override;
    void update() override;
    void do_cmd() override;
    void handle_info(float p1, float p2) override;
    void handle_msg(const mavlink_message_t &msg) override;
    void handle_info_test(float p1, float p2) override;

    AP_HAL::UARTDriver* get_port(void) {return _port;}
    FD1_msg_attack& get_msg_attack() { return uart_msg_attack; }

private:
    AP_HAL::UARTDriver* _port;
    // message structure
    FD1_msg_attack uart_msg_attack; //通用应答格式  飞控→任务

    uint32_t _last_ms;

    DerivativeFilterFloat_Size7 _pitch_filter;
    DerivativeFilterFloat_Size7 _yaw_filter;
    LowPassFilterFloat _yaw_sample_filter;
    LowPassFilterFloat _pitch_sample_filter;
    float _last_yaw;
    float _last_yaw_sample;
};

class UTarget_Loc: public UTarget_Base {
public:
    friend class UAttack;

    UTarget_Loc(UAttack &frotend_in);
    ~UTarget_Loc() {};
    bool init() override;
    bool is_valid() override;
    void update() override;
    void do_cmd() override;
    void handle_info(float p1, float p2) override;
    void handle_msg(const mavlink_message_t &msg) override;
    void handle_info_test(float p1, float p2) override;

private:
    bool _have_target;
    uint32_t _last_ms;

    DerivativeFilterFloat_Size7 _pitch_filter;
    DerivativeFilterFloat_Size7 _yaw_filter;
    LowPassFilterFloat _yaw_sample_filter;
    LowPassFilterFloat _pitch_sample_filter;
    float _last_yaw;
    float _last_yaw_sample;
};

class UTarget_Mav: public UTarget_Base {
public:
    friend class UAttack;

    UTarget_Mav(UAttack &frotend_in);
    ~UTarget_Mav() {};
    bool init() override;
    bool is_valid() override;
    void update() override;
    void do_cmd() override;
    void handle_info(float p1, float p2) override;
    void handle_msg(const mavlink_message_t &msg) override;
    void handle_info_test(float p1, float p2) override;

private:
    uint32_t _last_ms;

    DerivativeFilterFloat_Size7 _pitch_filter;
    DerivativeFilterFloat_Size7 _yaw_filter;
    LowPassFilterFloat _yaw_sample_filter;
    LowPassFilterFloat _pitch_sample_filter;
    float _last_yaw;
    float _last_yaw_sample;
};
