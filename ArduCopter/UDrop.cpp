#include "Copter.h"

void UDrop::init()
{
    uart_msg_drop.init();
    uart_msg_drop.get_msg_ID1().set_enable();
    uart_msg_drop.get_msg_ID2().set_enable();
    uart_msg_drop.get_msg_ID6().set_enable();

    set_state(UDropState::Idle);
}


void UDrop::update()
{
    if (!uart_msg_drop.initialized()) {
        if (!is_state(UDropState::Error)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Err! Fail to initialize");
            set_state(UDropState::Error);
        }
        return;
    }
    read_uart();
    update_state();
    write_uart();
}

void UDrop::read_uart()
{
	if (!uart_msg_drop.initialized()) {return;}
    uart_msg_drop.read();
    if (uart_msg_drop.get_msg_ID2()._msg_1.updated) {
        Location temp_loc = Location(
            uart_msg_drop.get_msg_ID2()._msg_1.content.msg.lat,
            uart_msg_drop.get_msg_ID2()._msg_1.content.msg.lng,
            uart_msg_drop.get_msg_ID2()._msg_1.content.msg.alt*100, Location::AltFrame::ABSOLUTE
        );
        do_set_waypoint(temp_loc);
        uart_msg_drop.get_msg_ID2()._msg_1.updated = false;
    }
    if (uart_msg_drop.get_msg_ID6()._msg_1.updated) {
        if (uart_msg_drop.get_msg_ID6()._msg_1.content.msg.launch == 1) {
            do_launch();
        }
        uart_msg_drop.get_msg_ID6()._msg_1.updated = false;
    }
}

void UDrop::write_uart()
{
    if (!uart_msg_drop.initialized()) {return;}
    static uint32_t _last_send_ms = 0;
    if (millis() - _last_send_ms < 500) {
        return;
    }
    _last_send_ms = millis();
    FD1_msg_ID1 &temp_msg = uart_msg_drop.get_msg_ID1();
    switch (_state) {
        case UDropState::Idle: 
            temp_msg._msg_1.content.msg.status = 0x01;
            break;
        case UDropState::Ready: 
        case UDropState::Launching: 
            temp_msg._msg_1.content.msg.status = 0x03;
            break;
        case UDropState::Done: 
            temp_msg._msg_1.content.msg.status = 0x05;
            break;
        case UDropState::Error: 
            temp_msg._msg_1.content.msg.status = 0x09;
            break;
        default:
            temp_msg._msg_1.content.msg.status = 0x00;
            break;
    }
    temp_msg.cal_sumcheck();
    temp_msg._msg_1.need_send = true;
    uart_msg_drop.write();
}

void UDrop::set_state(UDrop::UDropState in_state)
{
    if (_state == in_state) {return;}
    _state = in_state;
    _last_state_ms = millis();
    // gcs().send_text(MAV_SEVERITY_INFO, "%d", _last_state_ms);
    switch (_state) {
        case UDropState::Idle: 
            gcs().send_text(MAV_SEVERITY_INFO, "State Idle");
            break;
        case UDropState::Ready: 
            gcs().send_text(MAV_SEVERITY_INFO, "State Ready");
            break;
        case UDropState::Launching: 
            gcs().send_text(MAV_SEVERITY_INFO, "State Launching");
            break;
        case UDropState::Done: 
            gcs().send_text(MAV_SEVERITY_INFO, "State Done");
            break;
        case UDropState::Error: 
            gcs().send_text(MAV_SEVERITY_INFO, "State Error");
            break;
        default:
            break;
    }
}

bool UDrop::is_state(UDrop::UDropState in_state)
{
    return (_state == in_state);
}

void UDrop::update_state()
{
    uint32_t tnow = millis();
    switch (_state) {
        case UDropState::Launching: 
            if (tnow - _last_state_ms > 10000) {
                gcs().send_text(MAV_SEVERITY_INFO, "Err! Fail to Drop");
                set_state(UDropState::Error);
            }
            verify_launch();  
            break;
        default:
            break;
    }
}

void UDrop::do_set_waypoint(Location& loc)
{
    AP_Mission::Mission_Command tmp_cmd_wp;
    tmp_cmd_wp.id = MAV_CMD_NAV_WAYPOINT;
    tmp_cmd_wp.content.location.lat = loc.lat;
    tmp_cmd_wp.content.location.lng = loc.lng;
    tmp_cmd_wp.content.location.set_alt_cm(loc.alt, Location::AltFrame::ABSOLUTE);
    AP_Mission::Mission_Command tmp_cmd_land;
    tmp_cmd_land.id = MAV_CMD_NAV_LAND;
    tmp_cmd_land.content.location.lat = loc.lat;
    tmp_cmd_land.content.location.lng = loc.lng;
    tmp_cmd_land.content.location.set_alt_cm(0, Location::AltFrame::ABSOLUTE);
    if (copter.mode_auto.mission.num_commands() != 0) {
        copter.mode_auto.mission.clear();
        gcs().send_text(MAV_SEVERITY_INFO, "Clear previous mission");
        // copter.mode_auto.mission.write_home_to_storage();
        // gcs().send_text(MAV_SEVERITY_INFO, "N: %d", copter.mode_auto.mission.num_commands());
    }
    if (copter.mode_auto.mission.add_cmd(tmp_cmd_wp) && 
        copter.mode_auto.mission.add_cmd(tmp_cmd_wp) && 
        copter.mode_auto.mission.add_cmd(tmp_cmd_land)) {
        set_state(UDropState::Ready);
        // gcs().send_text(MAV_SEVERITY_INFO, "N: %d", copter.mode_auto.mission.num_commands());
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Err! Fail to write WP");
    }
}

void UDrop::do_launch()
{
    if (is_state(UDropState::Ready)) {
        set_state(UDropState::Launching);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "No target position!");
        set_state(UDropState::Launching);
    }
}

void UDrop::verify_launch()
{
    static uint32_t _last_update_ms = 0;
    if (millis() - _last_update_ms < 500) {
        return;
    }
    _last_update_ms = millis();
    bool mode_ok = false;
    if (copter.flightmode->mode_number() != Mode::Number::THROW) {
        mode_ok = copter.set_mode(Mode::Number::THROW, ModeReason::GCS_COMMAND);
    } else {
        mode_ok = true;
    }
    bool arm_ok = false;
    if (!copter.motors->armed()) {
        arm_ok = copter.arming.arm(AP_Arming::Method::MAVLINK);
    } else {
        arm_ok = true;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Arm:%d,Mode:%d",arm_ok, mode_ok);
    if (mode_ok && arm_ok) {
        set_state(UDropState::Done);
    }
}

