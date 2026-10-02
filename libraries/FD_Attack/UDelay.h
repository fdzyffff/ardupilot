#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

/*
 * UDelay —— 带时间戳的向量环形缓冲：按"时刻"回取历史姿态/陀螺。
 *
 * 相机量测比机体姿态晚一个管线延迟 Δ，去旋转必须用"量测真实时刻"的姿态。
 * 每周期推入一份样本，查询时按时间戳对相邻两样本线性插值（角度版走 ±180°
 * 最短路径）。400 样本，200Hz 调度下覆盖约 2.0s。
 */
class UDelay {
public:
    static constexpr uint16_t BUFFER_SIZE = 400;

    UDelay();

    void init();
    void push(const Vector3f &value);
    bool get_idx(uint16_t step, Vector3f &value) const;

    // 按时间戳取数（相邻样本线性插值）；目标时刻超出缓冲覆盖范围时返回 false
    bool get_at_time(uint32_t t_target_ms, Vector3f &value) const;

    // 角度专用版：分量跨 ±180° 时按最短路径插值（姿态角用这个，陀螺用上面的）
    bool get_at_time_angle(uint32_t t_target_ms, Vector3f &value) const;

    // 缓冲当前覆盖的时间跨度（最新样本时刻 − 最老样本时刻）
    uint32_t get_span_ms() const;

private:
    struct Sample {
        Vector3f value;
        uint32_t time_ms;
        bool valid;
    };

    Sample _buffer[BUFFER_SIZE];
    uint16_t _write_index;
    uint16_t _sample_count;

    // 在缓冲里找夹住 t_target_ms 的相邻两样本（idx_old 较老 / idx_new 较新）与插值比例
    bool find_bracket(uint32_t t_target_ms, uint16_t &idx_old, uint16_t &idx_new, float &frac) const;
};
