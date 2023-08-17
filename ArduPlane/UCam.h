#pragma once

class UCam {
public:
    friend class UAttack;

    UAttack_Port(UAttack &frotend_in): _frotend(frotend_in) {};
    virtual bool is_valid() = 0;
    virtual void update() = 0;
    virtual void do_cmd() = 0;
    virtual void handle_info() = 0;
    UAttack &_frotend;

};

class UCam_DYT: public UCam {
public:
    friend class UAttack;

    UAttack_Port_DYT(UAttack &frotend_in AP_HAL::UARTDriver* port_in);
    bool is_valid() override;
    void update() override;
    void do_cmd() override;
    void handle_info() override;

private:
    FD1_UART* FD1_uart_ptr;
    uint32_t _last_ms;

    DerivativeFilterFloat_Size7 _pitch_filter;
    DerivativeFilterFloat_Size7 _yaw_filter;
    LowPassFilterFloat _yaw_rate_filter;
};
