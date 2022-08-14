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

    if (_target != 0x10) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "Err: T[%d] C[%d] R[%d]",_target,_cmd,_ret);
        return;
    }
    
    if (!_ret) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "Fail: Cmd[%d]:%d",_cmd,_ret);
        return;
    }

    switch (_cmd) {
        case 0x01:
            _current_state = payload_parse;
            _new_msg = true;
            break;
        case 0x02:
            _current_state = payload_selfcheck;
            _new_msg = true;
            break;
        case 0x03:
            _current_state = payload_voltup;
            _new_msg = true;
            break;
        case 0x04:
            _current_state = payload_arm;
            _new_msg = true;
            break;
        case 0x05:
            _current_state = payload_fire;
            _new_msg = true;
            break;
        default:
            break;
    }
    send_current_state_text();
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
            _desire_state = state;
            gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload Fire");
            break;
        case payload_none:
            _desire_state = state;
            gcs().send_text(MAV_SEVERITY_WARNING, "Set Payload None");
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
            tmp_next_state = payload_selfcheck;
            break;
        case payload_selfcheck:
            tmp_next_state = payload_voltup;
            break;
        case payload_voltup:
            tmp_next_state = payload_arm;
            break;
        case payload_arm:
            tmp_next_state = payload_fire;
            break;
        case payload_fire:
            tmp_next_state = payload_fire;
            break;
        default:
            break;
    }

    static state_t last_report_state = tmp_next_state;
    static uint32_t last_report_ms = 0;
    bool tmp_need_report = false;
    if (last_report_state != tmp_next_state || millis()-last_report_ms>3000) {
        tmp_need_report = true;
        last_report_ms = millis();
        last_report_state = tmp_next_state;
    }

    _uart.get_msg_apm2payload()._msg_1.content.msg.target = 0x01;
    _uart.get_msg_apm2payload()._msg_1.content.msg.ret = 0x00;
    switch (tmp_next_state) {
        case payload_parse:
            if (tmp_need_report) {gcs().send_text(MAV_SEVERITY_WARNING, "Sending Parse");}
            _uart.get_msg_apm2payload()._msg_1.content.msg.cmd = 0x01;
            _uart.get_msg_apm2payload()._msg_1.need_send = true;
            break;
        case payload_selfcheck:
            if (tmp_need_report) {gcs().send_text(MAV_SEVERITY_WARNING, "Sending Selfcheck");}
            _uart.get_msg_apm2payload()._msg_1.content.msg.cmd = 0x01;
            _uart.get_msg_apm2payload()._msg_1.need_send = true;
            break;
        case payload_voltup:
            if (tmp_need_report) {gcs().send_text(MAV_SEVERITY_WARNING, "Sending VoltUP");}
            _uart.get_msg_apm2payload()._msg_1.content.msg.cmd = 0x03;
            _uart.get_msg_apm2payload()._msg_1.need_send = true;
            break;
        case payload_arm:
            if (tmp_need_report) {gcs().send_text(MAV_SEVERITY_WARNING, "Sending Arm");}
            _uart.get_msg_apm2payload()._msg_1.content.msg.cmd = 0x04;
            _uart.get_msg_apm2payload()._msg_1.need_send = true;
            break;
        case payload_fire:
            if (tmp_need_report) {gcs().send_text(MAV_SEVERITY_WARNING, "Sending Fire");}
            _uart.get_msg_apm2payload()._msg_1.content.msg.cmd = 0x05;
            _uart.get_msg_apm2payload()._msg_1.need_send = true;
            break;
        case payload_none:
        default:
            break;
    }

    if (_uart.initialized()) {
        _uart.write();
    }
}

void UPayload::push_state() {
    if (_new_msg) {
        _new_msg = false;
        if (_desire_state > _current_state) {
            do_next_state();
            _last_state_ms = AP_HAL::millis();
        }
    } else {
        if (_desire_state > _current_state && AP_HAL::millis() - _last_state_ms > 333) {
            do_next_state();
            _last_state_ms = AP_HAL::millis();
        }
    }
}

void UPayload::flying_check() {
    static uint32_t _last_land_ms = millis();
    static bool need_update = false;
    if (copter.motors->armed() && !copter.ap.land_complete) {
        if (millis() - _last_land_ms > 5000) {
            if (need_update && _current_state==payload_selfcheck) {
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
    if (_uart.initialized()) {
        _uart.read();
    }

    if (_uart.get_msg_payload2apm()._msg_1.updated) {
        msg_payload2apm_handle();
    }

    // update to volt UP after flying 5s
    flying_check();

    // send cmd to payload to push state to desire_state
    push_state();
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
