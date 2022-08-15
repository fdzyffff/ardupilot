#include "Copter.h"

UPayload::UPayload()
{

    _last_state_ms = AP_HAL::millis();
    _desire_state = payload_none; // the state want to be
    _current_state = payload_none; // the state confirmed from payload
    _next_state = payload_none; // the state send to payload
}

// initialise
void UPayload::init()
{
    _uart.init();
    _uart.get_msg_payload2apm().set_enable();
}

// clear return path and set home location.  This should be called as part of the arming procedure
void UPayload::msg_payload2apm_handle()
{
    _uart.get_msg_payload2apm()._msg_1.updated = false;
    uint8_t _target = _uart.get_msg_payload2apm()._msg_1.content.msg.target;
    uint8_t _cmd = _uart.get_msg_payload2apm()._msg_1.content.msg.cmd;
    uint8_t _ret = _uart.get_msg_payload2apm()._msg_1.content.msg.ret;

    if (_target != 0x10) {return;}

    bool success = false;
    switch (_next_state) {
        case payload_parse:
            if (_cmd == 0x01) {
                // copter.gcs().send_text(MAV_SEVERITY_WARNING, "step[%d]:%d",_cmd,_ret);
                success = _ret;
            }
            break;
        case payload_selfcheck:
            if (_cmd == 0x02) {
                // copter.gcs().send_text(MAV_SEVERITY_WARNING, "step[%d]:%d",_cmd,_ret);
                success = _ret;
            }
            break;
        case payload_voltup:
            if (_cmd == 0x03) {
                // copter.gcs().send_text(MAV_SEVERITY_WARNING, "step[%d]:%d",_cmd,_ret);
                success = _ret;
            }
            break;
        case payload_arm:
            if (_cmd == 0x04) {
                // copter.gcs().send_text(MAV_SEVERITY_WARNING, "step[%d]:%d",_cmd,_ret);
                success = _ret;

            }
            break;
        case payload_fire:
            if (_cmd == 0x05) {
                // copter.gcs().send_text(MAV_SEVERITY_WARNING, "step[%d]:%d",_cmd,_ret);
                success = _ret;
            }
            break;
        case payload_none:
        default:
            break;
    }
    if (success && _next_state > _current_state) {
        _current_state = _next_state;
        send_current_state_text();
    }
}

void UPayload::send_current_state_text() {
    switch (_current_state) {
        case payload_parse:
            gcs().send_text(MAV_SEVERITY_WARNING, "In Payload Parse");
            break;
        case payload_selfcheck:
            gcs().send_text(MAV_SEVERITY_WARNING, "In Payload Selfcheck");
            break;
        case payload_voltup:
            gcs().send_text(MAV_SEVERITY_WARNING, "In Payload VoltUP");
            break;
        case payload_arm:
            gcs().send_text(MAV_SEVERITY_WARNING, "In Payload Arm");
            break;
        case payload_fire:
            gcs().send_text(MAV_SEVERITY_WARNING, "In Payload Fire");
            break;
        case payload_none:
            gcs().send_text(MAV_SEVERITY_WARNING, "In Payload None");
        default:
            break;
    }
}

void UPayload::set_state(state_t state) {
    if (_desire_state > state) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Can not set back");
        send_current_state_text();
        return;
    }

    switch (state) {
        case payload_parse:
            _desire_state = state;
            gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload Parse");
            break;
        case payload_selfcheck:
            _desire_state = state;
            gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload Selfcheck");
            break;
        case payload_voltup:
            _desire_state = state;
            gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload VoltUP");
            break;
        case payload_arm:
            _desire_state = state;
            gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload Arm");
            break;
        case payload_fire:
            if (_current_state == payload_arm) {
                _desire_state = state;
                gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload Fire");
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "Fail Set Fire");
            }
            break;
        case payload_none:
            _desire_state = state;
            gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload None");
        default:
            break;
    }
    send_current_state_text();
}

void UPayload::do_next_state() {
    uint8_t _cmd = 0x00;
    switch (_next_state) {
        case payload_parse:
            _cmd = 0x01;
            break;
        case payload_selfcheck:
            _cmd = 0x02;
            break;
        case payload_voltup:
            _cmd = 0x03;
            break;
        case payload_arm:
            _cmd = 0x04;
            break;
        case payload_fire:
            _cmd = 0x05;
            break;
        case payload_none:
        default:
            break;
    }

    _uart.get_msg_apm2payload()._msg_1.content.msg.target = 0x01;
    _uart.get_msg_apm2payload()._msg_1.content.msg.cmd = _cmd;
    _uart.get_msg_apm2payload()._msg_1.content.msg.ret = 0x00;
    _uart.get_msg_apm2payload()._msg_1.need_send = true;
}

void UPayload::push_state() {
    switch (_current_state) {
        case payload_none:
            _next_state = payload_parse;
            break;
        case payload_parse:
            _next_state = payload_selfcheck;
            break;
        case payload_selfcheck:
            _next_state = payload_voltup;
            break;
        case payload_voltup:
            _next_state = payload_arm;
            break;
        case payload_arm:
            _next_state = payload_fire;
            break;
        case payload_fire:
        default:
            break;
    }
}

void UPayload::update_state() {
    if (_next_state > _current_state) {
        if (AP_HAL::millis() - _last_state_ms > 1000) {
            do_next_state();
            _last_state_ms = AP_HAL::millis();
        }
    } else if (_next_state == _current_state) {
        if (_desire_state > _next_state) {
            push_state();
            if (_last_state_ms > 1000) {_last_state_ms -= 1000;}
        }
    }
}
void UPayload::update()
{
    if (_uart.initialized()) {
        _uart.read();
    }

    if (_uart.get_msg_payload2apm()._msg_1.updated) {
        msg_payload2apm_handle();
    }

    update_state();

    if (_uart.initialized()) {
        _uart.write();
    }
}

void UPayload::cmd_handle(int16_t cmd_in)
{
    if (!_uart.initialized()) {return;}
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
