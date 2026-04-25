// libraries/AP_Redundancy/AP_Redundancy_config.h
#pragma once

// Pull in board-level defines: AP_HAL.h → AP_HAL_Boards.h → board/chibios.h → hwdef.h
// hwdef.h contains ENABLE_REDUNDANCY_CONTROL from the board's hwdef.dat.
// This must happen BEFORE the #ifndef fallback below.
#include <AP_HAL/AP_HAL.h>

#ifndef ENABLE_REDUNDANCY_CONTROL
#define ENABLE_REDUNDANCY_CONTROL 1
#endif

// Frame version (唯一支持的版本，不保留向前兼容)
#define REDUNDANCY_FRAME_VERSION        3

// Frame headers
#define REDUNDANCY_FRAME_HEADER1        0xA5
#define REDUNDANCY_FRAME_HEADER2        0x5A

// Frame lengths (data bytes, excluding header and checksum)
// Comm frame data layout (v3, 80 data bytes):
//   timestamp(4)+health(1)+mode(1)+unlock(1)+att(12)+vel(12)+pos(12)+pwm(28)
//   +param_hash(4)+frame_ver(1)+relay_mode_low(1)+relay_mode_high(1)
//   +relay_state_low(1)+relay_state_high(1)
#define REDUNDANCY_COMM_FRAME_LEN       80
// FPGA→FMU 数据帧：timestamp(4)+status(1)+adc(21)+switch_reason(1)
//                  +switch_timestamp(4)+fpga_version(1) = 32
#define REDUNDANCY_FPGA_FRAME_LEN       32

// Timing
#define REDUNDANCY_UPDATE_INTERVAL_MS   20    // 50Hz control loop
#define REDUNDANCY_HEALTH_TIMEOUT_MS    100   // Timestamp tolerance for health check (5x comm frame interval)
#define REDUNDANCY_FOLLOW_ARM_RETRY_MS  5000  // Retry interval for follow-arm
#define REDUNDANCY_FOLLOW_ARM_MAX_RETRY 3     // Max follow-arm retries

// Param hash
#define REDUNDANCY_PARAM_HASH_NONE      0     // Hash not yet computed

// PWM channels
#define REDUNDANCY_MAX_PWM_CHANNELS     14

// UART crossbar configuration frame
#define REDUNDANCY_CFG_FRAME_HEADER2    0xA5  // 配置帧第2字节 (vs 数据帧的 0x5A)
#define REDUNDANCY_CFG_FRAME_LEN        0x0A  // 配置帧负载长度 (CMD+8×map+reserved = 10)
#define REDUNDANCY_CFG_CMD_UART_MAP     0x01  // 命令类型: 串口映射
#define REDUNDANCY_CFG_FRAME_TOTAL      14    // 配置帧总字节数 (hdr2+len1+data10+chk1)
#define REDUNDANCY_CFG_SEND_COUNT       3     // 配置帧重发次数
#define REDUNDANCY_CFG_SEND_INTERVAL_MS 100   // 重发间隔

// UART map default values (must match FPGA uart_crossbar defaults)
#define REDUNDANCY_UMAP_DEFAULT_1       1     // UART1 → SERIAL1
#define REDUNDANCY_UMAP_DEFAULT_2       2     // UART2 → SERIAL2
#define REDUNDANCY_UMAP_DEFAULT_3       3     // UART3 → SERIAL3
#define REDUNDANCY_UMAP_DEFAULT_4       4     // UART4 → SERIAL4
#define REDUNDANCY_UMAP_DEFAULT_5       5     // UART5 → SERIAL5
#define REDUNDANCY_UMAP_DEFAULT_6       0     // UART6 → 断开
#define REDUNDANCY_UMAP_DEFAULT_7       0     // UART7 → 断开
#define REDUNDANCY_UMAP_DEFAULT_8       0     // UART8 → 断开
#define REDUNDANCY_UMAP_COUNT           8     // 对外 UART 总数
#define REDUNDANCY_UMAP_MAX_SERIAL      5     // 最大 SERIAL 号
