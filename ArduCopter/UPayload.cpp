#include "Copter.h"

UPayload::UPayload()
{

    _last_state_ms = AP_HAL::millis();
    _desire_state = payload_none; // the state want to be
    _current_state = payload_none; // the state confirmed from payload
    _new_msg = false;
}

// initialise
void UPayload::init()
{
    FD_uart_payload.init();
    FD_uart_payload.get_msg_payload().set_enable();
}

// clear return path and set home location.  This should be called as part of the arming procedure
void UPayload::msg_payload2apm_handle()
{
    FD_uart_payload.get_msg_payload()._msg_1.updated = false;
    uint8_t _type = FD_uart_payload.get_msg_payload()._msg_1.content.msg.type;
    uint8_t _cmd = FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd;

    switch (_type) {
        case 0xA5:
            if (_cmd == 0xAA) {
                _current_state = payload_parse;
                _new_msg = true;
            }
            if (_cmd == 0xFF) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "Fail: Parse : %x, %x",_type,_ret);
            }
            break;
        case 0x88:
            if (_cmd == 0xAA) {
                _current_state = payload_arm1;
                _new_msg = true;
            }
            if (_cmd == 0xFF) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "Fail: Arm 1 : %x, %x",_type,_ret);
            }
            break;
        case 0x99:
            if (_cmd == 0xAA) {
                _current_state = payload_arm2;
                _new_msg = true;
            }
            if (_cmd == 0xFF) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "Fail: Arm 2 : %x, %x",_type,_ret);
            }
            break;
        case 0xC5:
            if (_cmd == 0xAA) {
                _current_state = payload_armfinal;
                _new_msg = true;
            }
            if (_cmd == 0xFF) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "Fail: Arm Final : %x, %x",_type,_ret);
            }
            break;
        case 0x58:
            if (_cmd == 0xAA) {
                _current_state = payload_destroy;
                _new_msg = true;
            }
            if (_cmd == 0xFF) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "Fail: Destroy : %x, %x",_type,_ret);
            }
            break;
        case 0xDE:
            if (_cmd == 0xAA) {
                _current_state = payload_disarm;
                _new_msg = true;
            }
            if (_cmd == 0xFF) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "Fail: Disarm : %x, %x",_type,_ret);
            }
            break;
        default:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Err: T[%d] C[%d]",_type,_cmd,_ret);
            break;
    }
    send_current_state_text();
}

void UPayload::send_current_state_text() {
    switch (_current_state) {
        case payload_none:
            gcs().send_text(MAV_SEVERITY_WARNING, "In Payload None");
            break;
        case payload_parse:
            gcs().send_text(MAV_SEVERITY_WARNING, "In Payload Parse");
            break;
        case payload_arm1:
            gcs().send_text(MAV_SEVERITY_WARNING, "In Payload Arm 1");
            break;
        case payload_arm2:
            gcs().send_text(MAV_SEVERITY_WARNING, "In Payload Arm 2");
            break;
        case payload_armfinal:
            gcs().send_text(MAV_SEVERITY_WARNING, "In Payload Arm Final");
            break;
        case payload_fire:
            gcs().send_text(MAV_SEVERITY_WARNING, "In Payload Fire");
            break;
        case payload_destroy:
            gcs().send_text(MAV_SEVERITY_WARNING, "In Payload Destroy");
            break;
        case payload_disarm:
            gcs().send_text(MAV_SEVERITY_WARNING, "In Payload Disarm");
            break;
        default:
            break;
    }
}

void UPayload::set_state(state_t state) {
    if ((_desire_state <= payload_fire) && (_desire_state > state)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Can not set back");
        send_current_state_text();
        return;
    }

    switch (state) {
        case payload_none:
            gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload None");
            break;
        case payload_parse:
            gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload Parse");
            break;
        case payload_arm1:
            gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload Arm 1");
            break;
        case payload_arm2:
            gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload Arm 2");
            break;
        case payload_armfinal:
            gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload Arm Final");
            break;
        case payload_fire:
            gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload Fire");
            break;
        case payload_destroy:
            gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload Destroy");
            break;
        case payload_disarm:
            gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload Disarm");
            break;
        default:
            break;
    }
    // send_current_state_text();
}

void UPayload::do_next_state() {
    state_t tmp_next_state = payload_none;
    switch (_current_state) {
        case payload_none:
            tmp_next_state = payload_parse;
            break;
        case payload_parse:
            tmp_next_state = payload_arm1;
            break;
        case payload_arm1:
            tmp_next_state = payload_arm2;
            break;
        case payload_arm2:
            tmp_next_state = payload_armfinal;
            break;
        case payload_armfinal:
            tmp_next_state = payload_fire;
            break;
        default:
            break;
    }

    if (tmp_next_state != payload_none) {
        send_state_msg(tmp_next_state);
    }
}

void UPayload::send_state_msg(state_t state) {
    FD_uart_payload.get_msg_payload()._msg_1.content.msg.type = 0x11;
    FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0x00;
    switch (state) {
        case payload_parse:
            gcs().send_text(MAV_SEVERITY_WARNING, "Send Payload Parse");
            FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0x12;
            break;
        case payload_arm1:
            gcs().send_text(MAV_SEVERITY_WARNING, "Send Payload Arm 1");
            FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0x21;
            break;
        case payload_arm2:
            gcs().send_text(MAV_SEVERITY_WARNING, "Send Payload Arm 2");
            FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0x22;
            break;
        case payload_armfinal:
            gcs().send_text(MAV_SEVERITY_WARNING, "Send Payload Arm Final");
            FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0x44;
            break;
        case payload_fire:
            gcs().send_text(MAV_SEVERITY_WARNING, "Send Payload Fire");
            FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0xE2;
            break;
        case payload_destroy:
            gcs().send_text(MAV_SEVERITY_WARNING, "Send Payload Destroy");
            FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0xF5;
            break;
        case payload_disarm:
            gcs().send_text(MAV_SEVERITY_WARNING, "Send Payload Disarm");
            FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0xB6;
            break;
        default:
            break;
    }

    if (FD_uart_payload.initialized()) {
        FD_uart_payload.get_msg_payload().sum();
        FD_uart_payload.get_port()->write(FD_uart_payload.get_msg_payload()._msg_1.data, sizeof(FD_uart_payload.get_msg_payload()._msg_1.data));
    }
}

void UPayload::push_state() {
    if (_new_msg) {
        _new_msg = false;
        if (_desire_state <= payload_fire && _desire_state > _current_state) {
            do_next_state();
            _last_state_ms = AP_HAL::millis();
        }
    } else {
        if ((_desire_state != payload_none) && (_desire_state != _current_state) && (AP_HAL::millis() - _last_state_ms > 3000)) {
            send_state_msg(_current_state+1);
            _last_state_ms = AP_HAL::millis();
        }
    }


    if (_desire_state <= payload_fire && _desire_state > _current_state) {
        if (_new_msg) {
            _new_msg = false;
            do_next_state();
            _last_state_ms = AP_HAL::millis();
        }
        if (AP_HAL::millis() - _last_state_ms > 3000) {
            do_next_state();
            _last_state_ms = AP_HAL::millis();
        }
    } else if ((_desire_state != payload_none) && (_desire_state != _current_state) && (AP_HAL::millis() - _last_state_ms > 3000)) {
        send_state_msg(_desire_state);
        _last_state_ms = AP_HAL::millis();
    }
}

void UPayload::flying_check() {
    static uint32_t _last_land_ms = millis();
    static bool need_update = false;
    if (copter.motors->armed() && !copter.ap.land_complete) {
        if (millis() - _last_land_ms > 5000) {
            if (need_update && _current_state == payload_selfcheck) {
                set_state(payload_voltup);
                need_update = false;
            }
        }

    } else {
        _last_land_ms = millis();
        need_update = true;
    }
}

void UPayload::update()
{

    if (!FD_uart_payload.initialized()) {return;}

    // static uint32_t last_update_ms = millis();
    // uint32_t tnow = millis();
    // static uint32_t pk0_count = 0;
    // static uint32_t pk1_count = 0;
    // static uint32_t pk2_count = 0;

    while (FD_uart_payload.get_port()->available()>0) {
        uint8_t temp = FD_uart_payload.get_port()->read();
        FD_uart_payload.get_msg_payload().parse(temp);
        if (FD_uart_payload.get_msg_payload()._msg_1.updated) {
            msg_payload2apm_handle();
        }

    // update to volt UP after flying 5s
    flying_check();

    // send cmd to payload to push state to desire_state
    push_state();
}

void UPayload::cmd_handle(int16_t cmd_in)
{
    if (!FD_uart_payload.initialized()) {return;}
    if (cmd_in == 1) {
        switch (_desire_state) {
            case payload_none:
                set_state(payload_parse);
                break;
            case payload_parse:
                set_state(payload_selfcheck);
                break;
            case payload_selfcheck:
                set_state(payload_voltup);
                break;
            case payload_voltup:
                set_state(payload_arm);
                break;
            case payload_arm:
                set_state(payload_fire);
                break;
            case payload_fire:
                set_state(payload_none);
                break;
            default:
                break;
        }
    }

    if (cmd_in == 2) {
        set_state(payload_fire);
    }
}
