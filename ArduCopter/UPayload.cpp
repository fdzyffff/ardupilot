#include "Copter.h"

UPayload::UPayload()
{
    ;
}

// initialise
void UPayload::init()
{
    _last_state_ms = millis();
    _desire_state = payload_none; // the state want to be
    _current_state = payload_none; // the state confirmed from payload
    FD_uart_payload.init();
    FD_uart_payload.get_msg_payload().set_enable();
    _fire_ms = 0;
    _msg_count = 0;
    _err_code = 0;
    copter.gcs().send_text(MAV_SEVERITY_WARNING, "Payload Init");
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
            }
            if (_cmd == 0xFF) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "Fail: Parse : %x, %x",_type,_cmd);
                _err_code = _cmd;
            }
            break;
        case 0x88:
            if (_cmd == 0xAA) {
                _current_state = payload_arm1;
            }
            if (_cmd == 0xFF) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "Fail: Arm 1 : %x, %x",_type,_cmd);
                _err_code = _cmd;
            }
            break;
        case 0x99:
            if (_cmd == 0xAA) {
                _current_state = payload_arm2;
            }
            if (_cmd == 0xFF) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "Fail: Arm 2 : %x, %x",_type,_cmd);
                _err_code = _cmd;
            }
            break;
        case 0xC5:
            if (_cmd == 0xAA) {
                _current_state = payload_armfinal;
            }
            if (_cmd == 0xFF) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "Fail: Arm Final : %x, %x",_type,_cmd);
                _err_code = _cmd;
            }
            break;
        case 0x58:
            if (_cmd == 0xAA) {
                _current_state = payload_destroy;
            }
            if (_cmd == 0xFF) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "Fail: Destroy : %x, %x",_type,_cmd);
                _err_code = _cmd;
            }
            break;
        case 0xDE:
            if (_cmd == 0xAA) {
                _current_state = payload_disarm;
            }
            if (_cmd == 0xFF) {
                copter.gcs().send_text(MAV_SEVERITY_WARNING, "Fail: Disarm : %x, %x",_type,_cmd);
                _err_code = _cmd;
            }
            break;
        default:
            copter.gcs().send_text(MAV_SEVERITY_WARNING, "Err: T[%d] C[%d]",_type,_cmd);
            _err_code = _cmd;
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
    if (_desire_state == state) {
        return;
    }

    // if ((_desire_state <= payload_fire) && (_desire_state > state)) {
    //     gcs().send_text(MAV_SEVERITY_WARNING, "Can not set back");
    //     send_current_state_text();
    //     return;
    // }

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
    _desire_state = state;
    if (_desire_state != _current_state) {
        send_state_msg(_desire_state);
    }
}


void UPayload::send_state_msg(state_t state) {
    FD_uart_payload.get_msg_payload()._msg_1.content.msg.header.head_1 = FD_msg_Payload::PREAMBLE1;
    FD_uart_payload.get_msg_payload()._msg_1.content.msg.header.head_2 = FD_msg_Payload::PREAMBLE2;
    FD_uart_payload.get_msg_payload()._msg_1.content.msg.post1 = FD_msg_Payload::POSTAMBLE1;
    FD_uart_payload.get_msg_payload()._msg_1.content.msg.post2 = FD_msg_Payload::POSTAMBLE2;
    FD_uart_payload.get_msg_payload()._msg_1.content.msg.type = 0x11;
    FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0x00;
    switch (state) {
        case payload_parse:
            // gcs().send_text(MAV_SEVERITY_WARNING, "Send Payload Parse");
            FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0x12;
            break;
        case payload_arm1:
            // gcs().send_text(MAV_SEVERITY_WARNING, "Send Payload Arm 1");
            FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0x21;
            break;
        case payload_arm2:
            // gcs().send_text(MAV_SEVERITY_WARNING, "Send Payload Arm 2");
            FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0x22;
            break;
        case payload_armfinal:
            // gcs().send_text(MAV_SEVERITY_WARNING, "Send Payload Arm Final");
            FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0x44;
            break;
        case payload_fire:
            // gcs().send_text(MAV_SEVERITY_WARNING, "Send Payload Fire");
            FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0xE2;
            break;
        case payload_destroy:
            // gcs().send_text(MAV_SEVERITY_WARNING, "Send Payload Destroy");
            FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0xF5;
            break;
        case payload_disarm:
            // gcs().send_text(MAV_SEVERITY_WARNING, "Send Payload Disarm");
            FD_uart_payload.get_msg_payload()._msg_1.content.msg.cmd = 0xB6;
            break;
        default:
            break;
    }

    if (FD_uart_payload.initialized()) {
        FD_uart_payload.get_msg_payload().sum_check();
        FD_uart_payload.get_port()->write(FD_uart_payload.get_msg_payload()._msg_1.content.data, sizeof(FD_uart_payload.get_msg_payload()._msg_1.content.data));
        _msg_count++;
    }
}

void UPayload::push_state() {
    if (AP_HAL::millis() - _last_state_ms > 2000) {
        switch (_current_state) {
            case payload_none:
                if (copter.motors->armed() && copter.motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
                    set_state(payload_parse);
                }
                break;
            case payload_parse:
                set_state(payload_arm1);
                break;
            case payload_disarm:
                set_state(payload_none);
                break;
            default:
                break;
        }

        if (_desire_state != _current_state) {
            send_state_msg(_desire_state);
            _last_state_ms = AP_HAL::millis();

            if (_msg_count > 5) {
                gcs().send_text(MAV_SEVERITY_INFO, "No Payload response");
                _msg_count = 0;
                _err_code = 2;
            }
        } else {
            _err_code = 0;
        }
    }
}

// void UPayload::flying_check() {
//     static uint32_t _last_land_ms = millis();
//     static bool need_update = false;
//     if (copter.motors->armed() && !copter.ap.land_complete) {
//         if (millis() - _last_land_ms > 5000) {
//             if (need_update && _current_state == payload_none) {
//                 set_state(payload_armfinal);
//                 need_update = false;
//             }
//         }

//     } else {
//         _last_land_ms = millis();
//         // need_update = true;
//     }
// }

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
            // gcs().send_text(MAV_SEVERITY_INFO, "updated");
            msg_payload2apm_handle();
            if (_msg_count > 0) {_msg_count--;}
        }
    }

    // update to volt UP after flying 5s
    // flying_check();

    // send cmd to payload to push state to desire_state
    push_state();

    update_destory();

    // for test purpose
    // static uint32_t last_test_ms = millis();
    // if (millis() - last_test_ms >5000 && (_desire_state == _current_state)) {
    //     last_test_ms = millis();
    //     switch (_desire_state) {
    //         case payload_none:
    //             set_state(payload_parse);
    //             // _current_state = payload_none;
    //             break;
    //         case payload_parse:
    //             set_state(payload_arm1);
    //             // _current_state = payload_parse;
    //             break;
    //         case payload_arm1:
    //             set_state(payload_arm2);
    //             // _current_state = payload_arm1;
    //             break;
    //         case payload_arm2:
    //             set_state(payload_armfinal);
    //             // _current_state = payload_arm2;
    //             break;
    //         case payload_armfinal:
    //             set_state(payload_fire);
    //             // _current_state = payload_armfinal;
    //             break;
    //         case payload_fire:
    //             set_state(payload_disarm);
    //             // _current_state = payload_fire;
    //             break;
    //         case payload_disarm:
    //             set_state(payload_parse);
    //             // _current_state = payload_disarm;
    //             break;
    //         default:
    //             break;
    //     }
    // }
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
                set_state(payload_arm1);
                break;
            case payload_arm1:
                set_state(payload_arm2);
                break;
            case payload_arm2:
                set_state(payload_armfinal);
                break;
            case payload_armfinal:
                set_state(payload_fire);
                break;
            case payload_fire:
                set_state(payload_disarm);
                break;
            case payload_disarm:
                set_state(payload_parse);
                break;
            default:
                break;
        }
    }

    if (cmd_in == 2) {
        set_state(payload_fire);
    }
}

void UPayload::handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        // decode packet
        // gcs().send_text(MAV_SEVERITY_WARNING, "Target mavpkg");
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(&msg, &packet);
        switch(packet.command) {
            case MAV_CMD_USER_5:
                handle_destory(is_equal(packet.param1, 1.0f), packet.param2);
                break;
            default:
                break;
        }
    }
}

void UPayload::handle_destory(bool do_destory, float fire_s_in) {
    if (do_destory) {       
        _fire_ms = millis();
        _fire_count_s = fire_s_in;
        gcs().send_text(MAV_SEVERITY_INFO, "destory after %0.1fs", _fire_count_s);
    } else {
        _fire_ms = 0;
        _fire_count_s = 0.0f;
        gcs().send_text(MAV_SEVERITY_INFO, "destory cancel");
        set_state(payload_disarm);
    }

}

void UPayload::update_destory()
{
    if (_fire_ms == 0) {return;}
    float dt = ((float)(millis() - _fire_ms)) * 0.001f;
    float last_s = _fire_count_s - dt;
    if (last_s < 0) {
        if (copter.motors->armed()) {
            set_state(payload_destroy);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "destory cancel due to disarm");
        }
        _fire_ms = 0;
        _fire_count_s = 0.0f;
    }


    static uint32_t _last_info_ms = millis();
    if (((millis() - _last_info_ms > 10000) && (last_s > 10.0f)) || ((millis() - _last_info_ms > 1000) && (last_s < 10.0f)) ) {
        _last_info_ms = millis();
        gcs().send_text(MAV_SEVERITY_INFO, "destory after %0.1fs", _fire_count_s - dt);
    }
}

uint8_t UPayload::get_status()
{
    uint8_t ret = 0;
    if (_err_code) {
        ret = 2;
    } else {
        switch (_current_state) {
            case payload_none:
            case payload_parse:
            case payload_arm1:
            case payload_arm2:
            case payload_disarm:
                ret = 0;
                break;
            case payload_armfinal:
            case payload_fire:
                ret = 1;
                break;
            default:
                break;
        }
    }

    return ret;
}
