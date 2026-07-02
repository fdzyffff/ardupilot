/// @file   AP_Parachute_DroneCAN_YHC.h
/// @brief  DroneCAN backend for Firefly Manti4 parachute (YHC variant)
///
/// Strategy:
///   - Always REJECT parachute KILLSWITCH (respond FLYFIRE_VEHICLESERVICE_RESPONSE_RESULT_REJECTED)
///   - When AP_Parachute::release() called, send DEPLOY_FORCE via ParachuteService (163)
///   - Periodically broadcast VehicleStatus (1664) for parachute node binding
///   - Receive ParachuteStatus (1663) for monitoring
#pragma once

#include "AP_Parachute_config.h"

#if HAL_PARACHUTE_ENABLED

#include <AP_DroneCAN/AP_DroneCAN.h>

// timeout for parachute status messages (5 seconds)
#define AP_PARACHUTE_YHC_TIMEOUT_MICROS 5000000

class AP_Parachute_DroneCAN_YHC {
public:
    AP_Parachute_DroneCAN_YHC();

    // initialise CAN subscriptions; called from AP_DroneCAN::init()
    static void subscribe_msgs(AP_DroneCAN *ap_dronecan);

    // periodic: broadcast VehicleStatus and check health
    void update(void);

    // send VehicleStatus broadcast (for node binding)
    void send_vehicle_status(void);

    // send DEPLOY_FORCE command to parachute
    void send_deploy_force(void);

    // accessors
    bool is_healthy() const { return _healthy; }
    uint8_t get_parachute_state() const { return _parachute_state; }

private:
    // ---- ParachuteStatus (1663, broadcast from parachute) ----
    static void handle_parachute_status_trampoline(AP_DroneCAN *ap_dronecan,
        const CanardRxTransfer& transfer,
        const flyfire_ParachuteStatus &msg);
    void handle_parachute_status(const flyfire_ParachuteStatus &msg, uint8_t node_id);

    // ---- VehicleService (164, service request from parachute) ----
    // server callback: called when parachute sends a request
    void handle_vehicle_service_request(const CanardRxTransfer& transfer,
        const flyfire_VehicleServiceRequest &req);
    Canard::ObjCallback<AP_Parachute_DroneCAN_YHC, flyfire_VehicleServiceRequest>
        _vehicle_service_cb{this, &AP_Parachute_DroneCAN_YHC::handle_vehicle_service_request};
    Canard::Server<flyfire_VehicleServiceRequest> *_vehicle_service_server;

    // ---- ParachuteService (163, command to parachute) ----
    void handle_parachute_service_response(const CanardRxTransfer& transfer,
        const flyfire_ParachuteServiceResponse &rsp);
    Canard::ObjCallback<AP_Parachute_DroneCAN_YHC, flyfire_ParachuteServiceResponse>
        _parachute_service_res_cb{this, &AP_Parachute_DroneCAN_YHC::handle_parachute_service_response};
    Canard::Client<flyfire_ParachuteServiceResponse> *_parachute_service_client;

    // ---- VehicleStatus (1664, broadcast to parachute) ----
    Canard::Publisher<flyfire_VehicleStatus> *_vehicle_status_pub;

    // find or allocate singleton backend
    static AP_Parachute_DroneCAN_YHC* get_backend(AP_DroneCAN *ap_dronecan,
        uint8_t node_id);

    AP_DroneCAN *_ap_dronecan;
    uint8_t _node_id;
    uint32_t _last_status_ms;
    uint32_t _last_vehicle_status_send_ms;
    bool _healthy;

    // cached state from ParachuteStatus
    uint8_t _parachute_state;
    bool _is_vehicle_connected;
    bool _is_enable_auto_launch;
    float _height_of_home;
    float _backup_power_voltage;
};

#endif // HAL_PARACHUTE_ENABLED
