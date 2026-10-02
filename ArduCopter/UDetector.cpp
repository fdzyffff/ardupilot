#include "Copter.h"
#include "UDetector.h"

// HUOJIAN: 两路近距探测器(lk) 接收与解析，只收不发。

const AP_Param::GroupInfo UDetector::var_info[] = {
    // @Param: DET_TOUT
    // @DisplayName: Detector data timeout
    // @Description: Time in milliseconds without a valid target frame before the detector data is treated as invalid
    // @Units: ms
    // @Range: 100 5000
    // @User: Standard
    AP_GROUPINFO("TOUT", 0, UDetector, timeout_ms, 1000),

    AP_GROUPEND
};

UDetector::UDetector()
{
    AP_Param::setup_object_defaults(this, var_info);
    for (uint8_t i = 0; i < NUM_DETECTORS; i++) {
        _det[i].port = nullptr;
        _det[i].last_frame_ms = 0;
        _det[i].last_byte_ms = 0;
        _det[i].last_fps_ms = 0;
        _det[i].frame_cnt = 0;
        _det[i].fps = 0.0f;
        _det[i].count = 0;
        _det[i].init_ok = false;
        for (uint8_t j = 0; j < FD_MSG_DET_LK_MAX_TARGETS; j++) {
            _det[i].dist[j] = 0.0f;
        }
    }
}

// HUOJIAN: 按 instance 顺序找 SerialProtocol_Detector 串口，找不到只提示不阻塞
void UDetector::init()
{
    for (uint8_t i = 0; i < NUM_DETECTORS; i++) {
        _det[i].init_ok = false;
        _det[i].port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Detector, i);
        if (_det[i].port == nullptr) {
            gcs().send_text(MAV_SEVERITY_WARNING, "UDET %d UART unavailable", (int)i + 1);
            continue;
        }
        _det[i].parser.set_enable();
        _det[i].init_ok = true;
        gcs().send_text(MAV_SEVERITY_WARNING, "UDET %d UART initialized", (int)i + 1);
    }
}

void UDetector::update()
{
    const uint32_t now_ms = AP_HAL::millis();

    for (uint8_t i = 0; i < NUM_DETECTORS; i++) {
        Instance &det = _det[i];
        if ((det.port == nullptr) || !det.init_ok) {
            continue;
        }

        while (det.port->available() > 0) {
            const int16_t read_value = det.port->read();
            if (read_value < 0) {
                break;
            }
            det.last_byte_ms = now_ms;
            det.parser.parse((uint8_t)read_value);
            if (det.parser._msg_1.updated) {
                det.parser._msg_1.updated = false;
                handle_frame(i);
            }
        }

        // 长时间无字节: 丢弃半帧重新同步
        if ((now_ms - det.last_byte_ms) > 500U) {
            det.parser.reset();
        }

        // 帧率统计
        if ((now_ms - det.last_fps_ms) >= 1000U) {
            det.fps = (float)det.frame_cnt * 1000.0f / (float)(now_ms - det.last_fps_ms);
            det.last_fps_ms = now_ms;
            det.frame_cnt = 0;
        }
    }
}

void UDetector::handle_frame(uint8_t inst)
{
    Instance &det = _det[inst];
    const FD_msg_DET_LK::MSG_Collection &msg = det.parser._msg_1.content.msg;

    det.count = msg.count;
    for (uint8_t j = 0; j < FD_MSG_DET_LK_MAX_TARGETS; j++) {
        det.dist[j] = (j < msg.count) ? ((float)msg.dist[j] * 0.25f) : 0.0f;
    }
    det.last_frame_ms = AP_HAL::millis();
    det.frame_cnt++;
}

bool UDetector::initialized(uint8_t inst) const
{
    if (inst >= NUM_DETECTORS) {
        return false;
    }
    return _det[inst].init_ok;
}

bool UDetector::valid(uint8_t inst) const
{
    if (inst >= NUM_DETECTORS) {
        return false;
    }
    const Instance &det = _det[inst];
    if (!det.init_ok || (det.last_frame_ms == 0)) {
        return false;
    }
    return (AP_HAL::millis() - det.last_frame_ms) < (uint32_t)timeout_ms.get();
}

bool UDetector::any_valid() const
{
    for (uint8_t i = 0; i < NUM_DETECTORS; i++) {
        if (valid(i)) {
            return true;
        }
    }
    return false;
}

uint8_t UDetector::target_count(uint8_t inst) const
{
    if (inst >= NUM_DETECTORS) {
        return 0;
    }
    return _det[inst].count;
}

float UDetector::distance_m(uint8_t inst, uint8_t idx) const
{
    if ((inst >= NUM_DETECTORS) || (idx >= FD_MSG_DET_LK_MAX_TARGETS)) {
        return 0.0f;
    }
    return _det[inst].dist[idx];
}

float UDetector::min_distance_m(uint8_t inst) const
{
    if (!valid(inst) || (_det[inst].count == 0)) {
        return 0.0f;
    }
    float min_dist = _det[inst].dist[0];
    for (uint8_t j = 1; j < _det[inst].count; j++) {
        if (_det[inst].dist[j] < min_dist) {
            min_dist = _det[inst].dist[j];
        }
    }
    return min_dist;
}

float UDetector::min_distance_m() const
{
    float ret = 0.0f;
    for (uint8_t i = 0; i < NUM_DETECTORS; i++) {
        if (!valid(i) || (_det[i].count == 0)) {
            continue;
        }
        const float dist = min_distance_m(i);
        if ((ret <= 0.0f) || (dist < ret)) {
            ret = dist;
        }
    }
    return ret;
}

uint16_t UDetector::frame_count(uint8_t inst) const
{
    if (inst >= NUM_DETECTORS) {
        return 0;
    }
    return _det[inst].frame_cnt;
}

float UDetector::get_fps(uint8_t inst) const
{
    if (inst >= NUM_DETECTORS) {
        return 0.0f;
    }
    return _det[inst].fps;
}

// HUOJIAN: 11052 只报最小的 4 个距离，目标个数为两路有效帧之和
uint8_t UDetector::report_count() const
{
    uint16_t total = 0;
    for (uint8_t i = 0; i < NUM_DETECTORS; i++) {
        if (valid(i)) {
            total += _det[i].count;
        }
    }
    if (total > 255U) {
        total = 255U;
    }
    return (uint8_t)total;
}

float UDetector::report_distance_m(uint8_t idx) const
{
    if (idx >= REPORT_TARGETS) {
        return 0.0f;
    }

    float sorted[NUM_DETECTORS * FD_MSG_DET_LK_MAX_TARGETS];
    uint8_t num = 0;
    for (uint8_t i = 0; i < NUM_DETECTORS; i++) {
        if (!valid(i)) {
            continue;
        }
        for (uint8_t j = 0; j < _det[i].count; j++) {
            sorted[num] = _det[i].dist[j];
            num++;
            if (num >= ARRAY_SIZE(sorted)) {
                break;
            }
        }
    }

    // 选择排序取第 idx 小
    for (uint8_t k = 0; (k <= idx) && (k < num); k++) {
        uint8_t sel = k;
        for (uint8_t m = k + 1; m < num; m++) {
            if (sorted[m] < sorted[sel]) {
                sel = m;
            }
        }
        if (sel != k) {
            const float tmp = sorted[k];
            sorted[k] = sorted[sel];
            sorted[sel] = tmp;
        }
    }

    return (idx < num) ? sorted[idx] : 0.0f;
}
