#include "UDelay.h"

UDelay::UDelay() :
    _write_index(0),
    _sample_count(0)
{
}

void UDelay::init()
{
    _write_index = 0;
    _sample_count = 0;
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
        _buffer[i].value.zero();
        _buffer[i].time_ms = 0;
        _buffer[i].valid = false;
    }
}

void UDelay::push(const Vector3f &value)
{
    _buffer[_write_index].value = value;
    _buffer[_write_index].time_ms = AP_HAL::millis();
    _buffer[_write_index].valid = true;

    _write_index++;
    if (_write_index >= BUFFER_SIZE) {
        _write_index = 0;
    }
    if (_sample_count < BUFFER_SIZE) {
        _sample_count++;
    }
}

bool UDelay::get_idx(uint16_t step, Vector3f &value) const
{
    if ((step >= BUFFER_SIZE) || (step >= _sample_count)) {
        return false;
    }

    const uint16_t newest_index = (_write_index == 0) ? (BUFFER_SIZE - 1) : (_write_index - 1);
    const uint16_t read_index = (newest_index >= step) ?
        (newest_index - step) :
        (BUFFER_SIZE + newest_index - step);

    if (!_buffer[read_index].valid) {
        return false;
    }

    // 拒绝过旧样本（时钟回绕/长期未推入的兜底）
    const uint32_t sample_age_ms = AP_HAL::millis() - _buffer[read_index].time_ms;
    if (sample_age_ms > 2500U) {
        return false;
    }

    value = _buffer[read_index].value;
    return true;
}

// 缓冲按推入顺序时间递增；从最新样本往回走，命中 (t_old <= t_target <= t_new) 即返回
bool UDelay::find_bracket(uint32_t t_target_ms, uint16_t &idx_old, uint16_t &idx_new, float &frac) const
{
    if (_sample_count < 2) {
        return false;
    }

    uint16_t newer = (_write_index == 0) ? (BUFFER_SIZE - 1) : (_write_index - 1);

    for (uint16_t k = 1; k < _sample_count; k++) {
        const uint16_t older = (newer == 0) ? (BUFFER_SIZE - 1) : (newer - 1);

        if (!_buffer[older].valid || !_buffer[newer].valid) {
            return false;
        }

        const uint32_t t_new = _buffer[newer].time_ms;
        const uint32_t t_old = _buffer[older].time_ms;

        if (t_old == t_new) {
            // 同一毫秒内多次推入：跳过重复时间戳继续往老的找
            newer = older;
            continue;
        }

        if (t_target_ms > t_new) {
            // 目标时刻比最新样本还新（延迟取 0 时）：调用侧回退为当前值
            return false;
        }

        if (t_target_ms >= t_old) {
            idx_old = older;
            idx_new = newer;
            frac = (float)(t_target_ms - t_old) / (float)(t_new - t_old);
            return true;
        }

        newer = older;
    }

    // 目标时刻比缓冲里最老的样本还老（有效延迟超过缓冲跨度）
    return false;
}

bool UDelay::get_at_time(uint32_t t_target_ms, Vector3f &value) const
{
    uint16_t idx_old = 0;
    uint16_t idx_new = 0;
    float frac = 0.0f;
    if (!find_bracket(t_target_ms, idx_old, idx_new, frac)) {
        return false;
    }

    value = _buffer[idx_old].value + (_buffer[idx_new].value - _buffer[idx_old].value) * frac;
    return true;
}

bool UDelay::get_at_time_angle(uint32_t t_target_ms, Vector3f &value) const
{
    uint16_t idx_old = 0;
    uint16_t idx_new = 0;
    float frac = 0.0f;
    if (!find_bracket(t_target_ms, idx_old, idx_new, frac)) {
        return false;
    }

    const Vector3f &v_old = _buffer[idx_old].value;
    const Vector3f &v_new = _buffer[idx_new].value;
    // 角度分量按最短路径求差，避免偏航角跨 ±180° 时插值跳到错误一侧
    const Vector3f delta_wrapped(wrap_180(v_new.x - v_old.x),
                                 wrap_180(v_new.y - v_old.y),
                                 wrap_180(v_new.z - v_old.z));
    value = v_old + delta_wrapped * frac;
    return true;
}

uint32_t UDelay::get_span_ms() const
{
    if (_sample_count < 2) {
        return 0;
    }

    const uint16_t newest_index = (_write_index == 0) ? (BUFFER_SIZE - 1) : (_write_index - 1);
    // 未写满一圈时最老样本在下标 0；写满后最老样本在写入位置
    const uint16_t oldest_index = (_sample_count < BUFFER_SIZE) ? 0 : _write_index;

    const uint32_t t_new = _buffer[newest_index].time_ms;
    const uint32_t t_old = _buffer[oldest_index].time_ms;
    return (t_new >= t_old) ? (t_new - t_old) : 0;
}
