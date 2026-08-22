#include "AP_Proximity_LidarN10P.h"

#if AP_PROXIMITY_LIDARN10P_ENABLED

#include <AP_HAL/AP_HAL.h>


void AP_Proximity_LidarN10P::update()
{
    if (_uart == nullptr) {
        return;
    }
    read_sensor_data();
    const uint32_t now = AP_HAL::millis();
    set_status((_last_distance_received_ms != 0 && now - _last_distance_received_ms <= TIMEOUT_MS) ?
               AP_Proximity::Status::Good : AP_Proximity::Status::NoData);
}

bool AP_Proximity_LidarN10P::read_sensor_data()
{
    uint16_t message_count = 0;
    uint32_t nbytes = MIN(uint32_t(4000), uint32_t(_uart->available()));
    while (nbytes-- > 0) {
        uint8_t byte;
        if (!_uart->read(byte)) {
            break;
        }
        _parser.parse(byte);
        if (_parser.consume_frame()) {
            _last_distance_received_ms = AP_HAL::millis();
            process_frame();
            message_count++;
        }
    }
    return message_count > 0;
}

void AP_Proximity_LidarN10P::reset_face_minima()
{
    for (uint8_t i = 0; i < FACE_COUNT; i++) {
        _face_minimum[i].valid = false;
        _face_minimum[i].angle_deg = 0.0f;
        _face_minimum[i].distance_m = 0.0f;
    }
}

void AP_Proximity_LidarN10P::add_reading(float angle_deg, float distance_m)
{
    if (distance_m < distance_min_m() || distance_m > distance_max_m() ||
        ignore_reading(angle_deg, distance_m)) {
        return;
    }
    const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg);
    const uint8_t face_index = face.sector;
    if (face_index >= FACE_COUNT) {
        return;
    }
    FaceMinimum &minimum = _face_minimum[face_index];
    if (!minimum.valid || distance_m < minimum.distance_m) {
        minimum.valid = true;
        minimum.angle_deg = angle_deg;
        minimum.distance_m = distance_m;
    }
    database_push(angle_deg, distance_m);
}

void AP_Proximity_LidarN10P::publish_face_minima()
{
    for (uint8_t i = 0; i < FACE_COUNT; i++) {
        const FaceMinimum &minimum = _face_minimum[i];
        if (!minimum.valid) {
            continue;
        }
        const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(minimum.angle_deg);
        frontend.boundary.set_face_attributes(face, minimum.angle_deg, minimum.distance_m, state.instance);
    }
}

void AP_Proximity_LidarN10P::process_frame()
{
    reset_face_minima();
    const uint8_t *data = reinterpret_cast<const uint8_t *>(&_parser.frame());
    const float start_angle_deg = 0.01f * float(UINT16_VALUE(data[5], data[6]));
    const float end_angle_deg = 0.01f * float(UINT16_VALUE(data[105], data[106]));
    const float span_deg = wrap_360(end_angle_deg - start_angle_deg);
    for (uint8_t point = 0; point < 32; point++) {
        const uint8_t index = uint8_t(7U + 3U * point);
        const uint16_t distance_mm = UINT16_VALUE(data[index], data[index + 1U]);
        if (distance_mm == 0 || distance_mm == 0xFFFF) {
            continue;
        }
        const float sensor_angle_deg = wrap_360(start_angle_deg + float(point) * (span_deg / 31.0f));
        const float body_angle_deg = correct_angle_for_orientation(sensor_angle_deg);
        add_reading(body_angle_deg, float(distance_mm) * 0.001f);
    }
    publish_face_minima();
}

#endif
