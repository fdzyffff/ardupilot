#include "Copter.h"

void UserEngines::Init()
{
    _engine[0].Init(SRV_Channel::k_engine_1, 1);
    _engine[1].Init(SRV_Channel::k_engine_2, 2);
    _engine[2].Init(SRV_Channel::k_engine_3, 3);
    _engine[3].Init(SRV_Channel::k_engine_4, 4);
    _engine[4].Init(SRV_Channel::k_engine_5, 5);
    _engine[5].Init(SRV_Channel::k_engine_6, 6);
    _engine[6].Init(SRV_Channel::k_engine_7, 7);

    set_state(UserEnginesState::None);
    // copter.ap.motor_interlock_switch = false;
    need_send = false;

    Set_port(AP_SerialManager::SerialProtocol_Engines);
}

void UserEngines::Set_port(AP_SerialManager::SerialProtocol in_protocol)
{
    if (_uart == nullptr) {
        _uart = new FD_UART(in_protocol);
        _uart->init();
        if (_uart->initialized()) {
            _engine[0].Set_msg(&(_uart->get_msg_engine(0)));
            _engine[1].Set_msg(&(_uart->get_msg_engine(1)));
            _engine[2].Set_msg(&(_uart->get_msg_engine(2)));
            _engine[3].Set_msg(&(_uart->get_msg_engine(3)));
            _engine[4].Set_msg(&(_uart->get_msg_engine(4)));
            _engine[5].Set_msg(&(_uart->get_msg_engine(5)));
            _engine[6].Set_msg(&(_uart->get_msg_engine(6)));
        }
    }
}

void UserEngines::Update()
{
    update_uart();
    update_engine();
    update_state();
    update_output();
}

void UserEngines::update_uart()
{
    _uart->read();
}

void UserEngines::set_state(UserEngines::UserEnginesState in_state)
{
    if (_state == in_state) {return;}
    _state = in_state;
    _last_state_ms = millis();
    switch (_state) {
        case UserEnginesState::Start:
            for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
                _engine[i_engine].boost();
            }
            break;
        case UserEnginesState::Stop:
            for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
                _engine[i_engine].brake();
            }
            break;
        default:
            break;
    }
}

bool UserEngines::is_state(UserEngines::UserEnginesState in_state)
{
    return (_state == in_state);
}

void UserEngines::update_engine()
{
    for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
        _engine[i_engine].Update();
    }
    // must put behind engine.Update() to set need_send flag true
    if (_uart->initialized()) {
        engine_msg_pack();
    }
}

void UserEngines::update_state()
{
    uint32_t tnow = millis();
    if (_state == UserEnginesState::Start)
    {
        if (tnow - _last_state_ms < 5000) {
            for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
                if (!_engine[i_engine].is_state(UserEngine::EngineState::Running))
                {
                    // copter.ap.motor_interlock_switch = false;
                    return;
                }
            }
            // copter.ap.motor_interlock_switch = true; //allow output to motors
        } else {
            set_state(UserEnginesState::None);
        }
    }

    if (_state == UserEnginesState::Stop)
    {
        // copter.ap.motor_interlock_switch = false;
        if (tnow - _last_state_ms > 5000) {
            set_state(UserEnginesState::None);
        }
    }
}

void UserEngines::update_output() // call at 400 Hz
{
    for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
        // if (_engine[i_engine].can_override() && copter.ap.motor_interlock_switch) {
        if (_engine[i_engine].can_override()) {
            float delta_throttle = 2000 - copter.g2.user_parameters.thr_low.get();
            _output[i_engine] = constrain_int16(copter.g2.user_parameters.thr_low + (int16_t)(copter.motors->get_throttle_out()*delta_throttle), copter.g2.user_parameters.thr_low, 2000);
        } else {
            _output[i_engine] = _engine[i_engine].get_output(); // the engine is in boost or brake procedures, output are pre-set in time order with uart state feedback
        }
        SRV_Channels::set_output_pwm(_engine[i_engine].get_srv_function(), _output[i_engine]);
    }
}

void UserEngines::engine_msg_pack()
{
    for (uint8_t i_engine = 0; i_engine < ENGINE_NUM; i_engine++) {
        if (_engine[i_engine].connected()) {
            uint8_t *temp_data = _engine[i_engine].Get_msg()->_msg_1.content.data;
            memcpy((engine_msg+i_engine*5), temp_data+4, 5);
            need_send |= true;
        } else {
            memset((engine_msg+i_engine*5), 0, 5);
        }
    }
    // if (_uart->initialized() && need_send) {
    //     _uart->get_port()->write(engine_msg, sizeof(engine_msg));
    //     need_send = false;
    // }
}

void UserEngines::forward_engine_control(float p1, float p2, float p3, float p4, float p5, float p6, float p7) {
    if (!_uart->initialized()) {
        return;
    }

    mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    uint8_t saved_seq = chan0_status->current_tx_seq;
    uint8_t saved_flags = chan0_status->flags;
    chan0_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    //chan0_status->current_tx_seq = FD1_mav.mavlink.seq;

    mavlink_message_t msg;
    uint16_t len;
    mavlink_command_long_t packet;
    packet.command = MAV_CMD_USER_3;
    packet.param1 = p1;
    packet.param2 = p2;
    packet.param3 = p3;
    packet.param4 = p4;
    packet.param5 = p5;
    packet.param6 = p6;
    packet.param7 = p7;
    packet.target_system = 0;
    packet.target_component = 0;
    packet.confirmation = 0;

    copter.gcs().send_text(MAV_SEVERITY_WARNING, "SEND [%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f]", p1, p2, p3, p4, p5, p6, p7);

    len = mavlink_msg_command_long_encode(copter.g.sysid_this_mav,
                                        0,
                                        &msg, &packet);

    _uart->get_port()->write(&msg.magic, 2);
    _uart->get_port()->write(&msg.magic+4, 4);
    _uart->get_port()->write(&msg.magic+10, len-6);

    // mavlink_heartbeat_t heartbeat = {0};
    // heartbeat.type = 1;
    // heartbeat.autopilot = 2;
    // heartbeat.base_mode = 3;
    // heartbeat.system_status = 4;
    // heartbeat.mavlink_version = 0;
    // heartbeat.custom_mode = 1;

    /*
     save and restore sequence number for chan0, as it is used by
     generated encode functions
    */
    // len = mavlink_msg_heartbeat_encode(5,
    //                                    8,
    //                                    &msg, &heartbeat);

//    gcs().send_text(MAV_SEVERITY_INFO, "ck %d  %d (%d)",(uint8_t)(msg.checksum & 0xFF), (uint8_t)(msg.checksum >> 8), len);


    // _uart->get_port()->write(&msg.magic, 2);
    // _uart->get_port()->write(&msg.magic+4, 4);
    // _uart->get_port()->write(&msg.magic+10, len-6);

    chan0_status->current_tx_seq = saved_seq;
    chan0_status->flags = saved_flags;
}
