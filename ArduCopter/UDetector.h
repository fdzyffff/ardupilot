#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <FD_UART/FD_msg_DET_LK.h>

/*
 * HUOJIAN: 近距探测器(lk) 管理 —— 两路串口，只收不发。
 * 用途: 目标距离上报，供起爆门限判定与 CHASE_FUZE(11052) 上报。
 */
class UDetector {
public:
    static const uint8_t NUM_DETECTORS = 2;
    static const uint8_t REPORT_TARGETS = 4;    // CHASE_FUZE 只上报前 4 个目标

    UDetector();

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();

    bool initialized(uint8_t inst) const;
    bool valid(uint8_t inst) const;              // 该路最近一帧未超时
    bool any_valid() const;
    uint8_t target_count(uint8_t inst) const;
    float distance_m(uint8_t inst, uint8_t idx) const;
    float min_distance_m(uint8_t inst) const;    // 该路最近目标距离(m)
    float min_distance_m() const;                // 两路取最小，全部无效返回 0
    uint16_t frame_count(uint8_t inst) const;
    float get_fps(uint8_t inst) const;

    // HUOJIAN: 11052 上报口径 —— 两路距离合并后取最小的 REPORT_TARGETS 个
    uint8_t report_count() const;
    float report_distance_m(uint8_t idx) const;

private:
    struct Instance {
        AP_HAL::UARTDriver *port;
        FD_msg_DET_LK parser;
        uint32_t last_frame_ms;
        uint32_t last_byte_ms;
        uint32_t last_fps_ms;
        uint16_t frame_cnt;
        float fps;
        float dist[FD_MSG_DET_LK_MAX_TARGETS];
        uint8_t count;
        bool init_ok;
    };

    Instance _det[NUM_DETECTORS];

    void handle_frame(uint8_t inst);

    // DET_TOUT 探测器数据超时 ms
    AP_Int32 timeout_ms;
};
