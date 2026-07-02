/// @file   AP_Parachute_DroneCAN_YHC.cpp
/// @brief  DroneCAN backend for Firefly Manti4 parachute (YHC variant)
///
/// Message flow:
///   OUT 1664 VehicleStatus (broadcast, 10Hz) — height, vz, state, connected flag
///   IN  1663 ParachuteStatus (broadcast) — state, errors, backup voltage
///   IN  164  VehicleService (request) — KILLSWITCH → always reject
///   OUT 163  ParachuteService (request) — DEPLOY_FORCE when AP_Parachute::release()

#include "AP_Parachute_config.h"

#if HAL_PARACHUTE_ENABLED

#include "AP_Parachute_DroneCAN_YHC.h"
#include "AP_Parachute.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>

#define LOG_TAG "ParaYHC"

extern const AP_HAL::HAL& hal;

AP_Parachute_DroneCAN_YHC::AP_Parachute_DroneCAN_YHC() :
    _ap_dronecan(nullptr),
    _node_id(0),
    _last_status_ms(0),
    _last_vehicle_status_send_ms(0),
    _healthy(false),
    _parachute_state(0),
    _vehicle_service_server(nullptr),
    _parachute_service_client(nullptr),
    _vehicle_status_pub(nullptr)
{
}

// called once by AP_DroneCAN::init()
void AP_Parachute_DroneCAN_YHC::subscribe_msgs(AP_DroneCAN *ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }

    AP_Parachute *para = AP::parachute();
    if (para == nullptr || !para->yhc_enabled()) {
        return;
    }

    // create singleton and register with AP_Parachute
    static AP_Parachute_DroneCAN_YHC *instance;
    if (instance == nullptr) {
        instance = NEW_NOTHROW AP_Parachute_DroneCAN_YHC();
        if (instance == nullptr) {
            return;
        }
        para->set_yhc_backend(instance);
    }

    // 1. Subscribe to ParachuteStatus broadcast (1663) from parachute
    if (Canard::allocate_sub_arg_callback(ap_dronecan,
            &handle_parachute_status_trampoline,
            ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("parachute_yhc_status");
    }

    // 2. Create VehicleService server (164) to handle KILLSWITCH/FLIGHT_BLOCK
    if (instance->_vehicle_service_server == nullptr) {
        instance->_vehicle_service_server = NEW_NOTHROW
            Canard::Server<flyfire_VehicleServiceRequest>(
                ap_dronecan->get_canard_iface(),
                instance->_vehicle_service_cb);
        if (instance->_vehicle_service_server == nullptr) {
            AP_BoardConfig::allocation_error("parachute_yhc_svc_srv");
        }
    }

    // 3. Create VehicleStatus publisher (1664) for periodic broadcast
    if (instance->_vehicle_status_pub == nullptr) {
        instance->_vehicle_status_pub = NEW_NOTHROW
            Canard::Publisher<flyfire_VehicleStatus>(
                ap_dronecan->get_canard_iface());
    }

    // store dronecan pointer for later use
    if (instance->_ap_dronecan == nullptr) {
        instance->_ap_dronecan = ap_dronecan;
    }
}

// find or bind the backend
AP_Parachute_DroneCAN_YHC* AP_Parachute_DroneCAN_YHC::get_backend(
    AP_DroneCAN *ap_dronecan, uint8_t node_id)
{
    AP_Parachute *para = AP::parachute();
    if (para == nullptr) {
        return nullptr;
    }
    return para->get_yhc_backend();
}

// ---- ParachuteStatus (1663, broadcast from parachute) ----

void AP_Parachute_DroneCAN_YHC::handle_parachute_status_trampoline(
    AP_DroneCAN *ap_dronecan,
    const CanardRxTransfer& transfer,
    const flyfire_ParachuteStatus &msg)
{
    AP_Parachute *para = AP::parachute();
    if (para == nullptr) {
        return;
    }
    AP_Parachute_DroneCAN_YHC *backend = para->get_yhc_backend();
    if (backend == nullptr) {
        return;
    }
    backend->handle_parachute_status(msg, transfer.source_node_id);
}

void AP_Parachute_DroneCAN_YHC::handle_parachute_status(
    const flyfire_ParachuteStatus &msg, uint8_t node_id)
{
    _parachute_state = msg.state;
    _is_vehicle_connected = msg.is_vehicle_connected;
    _is_enable_auto_launch = msg.is_enable_auto_launch;
    _height_of_home = msg.height_of_home;
    _backup_power_voltage = msg.backup_power_voltage;

    _last_status_ms = AP_HAL::millis();
    _healthy = true;

    if (_node_id == 0) {
        _node_id = node_id;
    }
}

// ---- VehicleService (164, service from parachute) ----
// Strategy: ALWAYS REJECT KILLSWITCH

void AP_Parachute_DroneCAN_YHC::handle_vehicle_service_request(
    const CanardRxTransfer& transfer,
    const flyfire_VehicleServiceRequest &req)
{
    if (req.magic_number != FLYFIRE_VEHICLESERVICE_REQUEST_MAGIC_NUMBER) {
        return;
    }

    flyfire_VehicleServiceResponse rsp;
    rsp.result = FLYFIRE_VEHICLESERVICE_RESPONSE_RESULT_REJECTED;

    if (_node_id == 0) {
        _node_id = transfer.source_node_id;
    }

    switch (req.type) {
        case FLYFIRE_VEHICLESERVICE_REQUEST_TYPE_KILLSWITCH:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ParaYHC: KILLSWITCH rejected");
            break;

        case FLYFIRE_VEHICLESERVICE_REQUEST_TYPE_FLIGHT_BLOCK:
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "ParaYHC: FLIGHT_BLOCK (parachute abnormal)");
            rsp.result = FLYFIRE_VEHICLESERVICE_RESPONSE_RESULT_SUCCESS;
            break;

        case FLYFIRE_VEHICLESERVICE_REQUEST_TYPE_FLIGHT_BLOCK_RELEASE:
            rsp.result = FLYFIRE_VEHICLESERVICE_RESPONSE_RESULT_SUCCESS;
            break;

        default:
            break;
    }

    if (_vehicle_service_server != nullptr) {
        _vehicle_service_server->respond(transfer, rsp);
    }
}

// ---- VehicleStatus (1664, broadcast TO parachute) ----

void AP_Parachute_DroneCAN_YHC::send_vehicle_status(void)
{
    if (_vehicle_status_pub == nullptr || _node_id == 0) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (now - _last_vehicle_status_send_ms < 100) {
        return;
    }
    _last_vehicle_status_send_ms = now;

    flyfire_VehicleStatus msg;
    msg.height_of_home = 0.0f;
    msg.vz = 0.0f;

    const bool armed = hal.util->get_soft_armed();
    msg.state = armed ? FLYFIRE_VEHICLESTATUS_STATE_IN_AIR
                      : FLYFIRE_VEHICLESTATUS_STATE_ON_GROUND;
    msg.is_parachute_connected = _healthy;

    _vehicle_status_pub->broadcast(msg);
}

// ---- ParachuteService (163, command TO parachute) ----

void AP_Parachute_DroneCAN_YHC::send_deploy_force(void)
{
    if (_ap_dronecan == nullptr || _node_id == 0) {
        return;
    }

    flyfire_ParachuteServiceRequest req;
    req.magic_number = FLYFIRE_PARACHUTESERVICE_REQUEST_MAGIC_NUMBER;
    req.type = FLYFIRE_PARACHUTESERVICE_REQUEST_TYPE_DEPLOY_FORCE;

    if (_parachute_service_client == nullptr) {
        _parachute_service_client = NEW_NOTHROW
            Canard::Client<flyfire_ParachuteServiceResponse>{
                _ap_dronecan->get_canard_iface(),
                _parachute_service_res_cb};
        if (_parachute_service_client == nullptr) {
            return;
        }
    }

    _parachute_service_client->request(_node_id, req);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
        "ParaYHC: DEPLOY_FORCE to node %d", _node_id);
}

void AP_Parachute_DroneCAN_YHC::handle_parachute_service_response(
    const CanardRxTransfer& transfer,
    const flyfire_ParachuteServiceResponse &rsp)
{
    if (rsp.result == FLYFIRE_PARACHUTESERVICE_RESPONSE_RESULT_SUCCESS) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ParaYHC: command SUCCESS");
    } else if (rsp.result == FLYFIRE_PARACHUTESERVICE_RESPONSE_RESULT_REJECTED) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ParaYHC: command REJECTED");
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
            "ParaYHC: command FAILED (%d)", rsp.result);
    }
}

// ---- Periodic update ----

void AP_Parachute_DroneCAN_YHC::update(void)
{
    send_vehicle_status();

    const uint32_t now = AP_HAL::millis();
    if (_last_status_ms != 0 &&
        (now - _last_status_ms) > (AP_PARACHUTE_YHC_TIMEOUT_MICROS / 1000)) {
        _healthy = false;
    }
}

#endif // HAL_PARACHUTE_ENABLED
