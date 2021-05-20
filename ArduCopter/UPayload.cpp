#include "Copter.h"

UPayload::UPayload()
{
    _state = payload_none;
    _last_state_ms = AP_HAL::millis();
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

        // // log lead's estimated vs reported position
        // AP::logger().Write("FOLL",
        //                                        "TimeUS,Lat,Lon,Alt,VelN,VelE,VelD,LatE,LonE,AltE",  // labels
        //                                        "sDUmnnnDUm",    // units
        //                                        "F--B000--B",    // mults
        //                                        "QLLifffLLi",    // fmt
        //                                        AP_HAL::micros64(),
        //                                        _target_location.lat,
        //                                        _target_location.lng,
        //                                        _target_location.alt,
        //                                        (double)_target_velocity_ned.x,
        //                                        (double)_target_velocity_ned.y,
        //                                        (double)_target_velocity_ned.z,
        //                                        loc_estimate.lat,
        //                                        loc_estimate.lng,
        //                                        packet.alt/10
        //                                        );

    if (_target != 0x10) {return;}

    switch (_state) {
        case payload_parse:
            if (_cmd == 0x01) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "step[%d]:%d",_cmd,_ret);
                if (_ret == 0xF0) {
                    set_state(payload_selfcheck);
                }
            }
            break;
        case payload_selfcheck:
            if (_cmd == 0x02) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "step[%d]:%d",_cmd,_ret);
                if (_ret == 0xF0) {
                    set_state(payload_voltup);
                }
            }
            break;
        case payload_voltup:
            if (_cmd == 0x03) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "step[%d]:%d",_cmd,_ret);
                if (_ret == 0xF0) {
                    set_state(payload_arm);
                }
            }
            break;
        case payload_arm:
            if (_cmd == 0x04) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "step[%d]:%d",_cmd,_ret);
                if (_ret == 0xF0) {
                    set_state(payload_fire);
                }
            }
            break;
        case payload_fire:
            if (_cmd == 0x05) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "step[%d]:%d",_cmd,_ret);
                if (_ret == 0xF0) {
                    set_state(payload_none);
                }
            }
            break;
        case payload_none:
        default:
            break;
    }
}

// update
void UPayload::set_state(state_t state) {
    if (_state == state) {return;}
    _state = state;
    _last_state_ms = AP_HAL::millis();
    uint8_t _cmd = 0x00;
    switch (_state) {
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

// update
void UPayload::update()
{
    if (_uart.initialized()) {
        _uart.read();
    }

    if (_uart.get_msg_payload2apm()._msg_1.updated) {
        msg_payload2apm_handle();
    }

    if (AP_HAL::millis() - _last_state_ms > 5000) {
        set_state(payload_none);
    }

    if (_uart.initialized()) {
        _uart.write();
    }
}
