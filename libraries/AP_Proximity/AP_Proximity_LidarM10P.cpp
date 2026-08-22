#include "AP_Proximity_LidarM10P.h"

#if AP_PROXIMITY_LIDARM10P_ENABLED

#include <AP_HAL/AP_HAL.h>


void AP_Proximity_LidarM10P::update()
{
    if (_uart == nullptr) {
        return;
    }
    read_sensor_data();
    const uint32_t now = AP_HAL::millis();
    set_status((_last_distance_received_ms != 0 && now - _last_distance_received_ms <= TIMEOUT_MS) ?
               AP_Proximity::Status::Good : AP_Proximity::Status::NoData);
}

bool AP_Proximity_LidarM10P::read_sensor_data()
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

void AP_Proximity_LidarM10P::reset_face_minima()
{
    for (uint8_t i = 0; i < FACE_COUNT; i++) {
        _face_minimum[i].valid = false;
        _face_minimum[i].angle_deg = 0.0f;
        _face_minimum[i].distance_m = 0.0f;
    }
}

void AP_Proximity_LidarM10P::add_reading(float angle_deg, float distance_m)
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

void AP_Proximity_LidarM10P::publish_face_minima()
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

void AP_Proximity_LidarM10P::process_frame()
{
    const uint16_t frame_length = _parser.frame_length();
    if (frame_length < FD1_msg_M10P::FRAME_OVERHEAD || frame_length > FD1_msg_M10P::MAX_FRAME_LENGTH ||
        ((frame_length - FD1_msg_M10P::FRAME_OVERHEAD) % 2U) != 0) {
        return;
    }
    const uint16_t point_count = (frame_length - FD1_msg_M10P::FRAME_OVERHEAD) / 2U;
    if (point_count == 0 || point_count > 70) {
        return;
    }
    reset_face_minima();
    const uint8_t *data = _parser.frame().data;
    const float start_angle_deg = 0.01f * float(UINT16_VALUE(data[4], data[5]));
    for (uint16_t point = 0; point < point_count; point++) {
        const uint16_t index = 8U + 2U * point;
        if (index + 1U >= frame_length - 12U) {
            break;
        }
        const uint16_t distance_mm = UINT16_VALUE(data[index], data[index + 1U]);
        if (distance_mm == 0 || distance_mm == 0xFFFF) {
            continue;
        }
        const float offset_deg = point_count > 1 ? float(point) * (15.0f / float(point_count - 1U)) : 0.0f;
        const float sensor_angle_deg = wrap_360(start_angle_deg + offset_deg);
        const float body_angle_deg = correct_angle_for_orientation(sensor_angle_deg);
        add_reading(body_angle_deg, float(distance_mm) * 0.001f);
    }
    publish_face_minima();
}

#endif
