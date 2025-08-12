#include "Plane.h"
#include <AP_HAL/AP_HAL.h>
#include <stdint.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_BattMonitor/AP_BattMonitor_Backend.h>

#if ENABLE_REDUNDANCY_CONTROL

// 前向声明
struct RedundancyParseContext;
struct RedundancyCommFrame;

// 函数声明
bool parse_redundancy_frame_byte(RedundancyParseContext& ctx, uint8_t byte);
void pack_redundancy_comm_frame(RedundancyCommFrame &frame,
                               uint32_t timestamp, uint8_t redundancy_health, uint8_t work_mode, uint8_t unlock_status,
                               float roll_deg, float pitch_deg, float yaw_deg,
                               float vel_n, float vel_e, float vel_d,
                               int32_t lat, int32_t lng, int32_t alt,
                               const uint16_t pwm[14]);
// 24位ADC值处理函数
uint32_t get_adc_value_24bit(const uint8_t* adc_data, uint8_t channel);
float convert_adc_to_float(uint32_t raw_value);

// 数据帧结构定义
struct RedundancyDataFrame_from_FPGA {
    uint8_t header[2];      // 0xA5 0x5A
    uint8_t frame_length;   // 帧长度
    uint32_t timestamp;     // 时间戳
    uint8_t redundancy_status; // 余度状态
    uint8_t adc_values[21]; // 7路ADC原始值，每路3字节，共21字节
    uint8_t checksum;       // 校验和
} __attribute__((packed));

// 余度间通信协议数据帧结构体
struct RedundancyCommFrame {
    uint8_t header[2];         // 0xA5 0x5A
    uint8_t frame_length;      // 数据区长度
    uint32_t timestamp;        // 4字节
    uint8_t redundancy_health; // 1字节，bit6~7:发送方的余度编号，bit0:余度1，bit1:余度2，bit2:余度3，0-不健康，1-健康
    uint8_t work_mode;         // 1字节
    uint8_t unlock_status;     // 1字节
    float roll_deg;            // 4字节，度
    float pitch_deg;           // 4字节，度
    float yaw_deg;             // 4字节，度
    float vel_n;               // 4字节，m/s
    float vel_e;               // 4字节, m/s
    float vel_d;               // 4字节, m/s
    int32_t lat;               // 4字节，1E7 deg
    int32_t lng;               // 4字节，1E7 deg
    int32_t alt;               // 4字节，cm
    uint16_t pwm[14];          // 28字节，2*14
    uint8_t checksum;          // 1字节
} __attribute__((packed));

// 协议解析状态机
enum class ParseState {
    WAIT_HEADER1,
    WAIT_HEADER2,
    WAIT_FRAME_LENGTH,
    WAIT_DATA
};

// 通用解析上下文结构体
struct RedundancyParseContext {
    ParseState state;
    uint8_t data_buffer[256];
    uint8_t data_index;
    uint8_t expected_length;
    uint32_t frame_count;
    uint32_t last_frame_count_time;
    RedundancyParseContext() : state(ParseState::WAIT_HEADER1), data_index(0), expected_length(0), frame_count(0), last_frame_count_time(0) {}
};

// 通用解析函数
bool parse_redundancy_frame_byte(RedundancyParseContext& ctx, uint8_t byte)
{
    switch (ctx.state) {
        case ParseState::WAIT_HEADER1:
            if (byte == 0xA5) {
                ctx.data_buffer[0] = byte;
                ctx.data_index = 1;
                ctx.state = ParseState::WAIT_HEADER2;
            }
            break;
        case ParseState::WAIT_HEADER2:
            if (byte == 0x5A) {
                ctx.data_buffer[1] = byte;
                ctx.data_index = 2;
                ctx.state = ParseState::WAIT_FRAME_LENGTH;
            } else {
                ctx.state = ParseState::WAIT_HEADER1;
            }
            break;
        case ParseState::WAIT_FRAME_LENGTH:
            ctx.expected_length = byte;
            ctx.data_buffer[2] = byte;
            ctx.data_index = 3;
            ctx.state = ParseState::WAIT_DATA;
            break;
        case ParseState::WAIT_DATA:
            ctx.data_buffer[ctx.data_index++] = byte;
            uint8_t total_expected_length = 2 + 1 + ctx.expected_length + 1;
            if (ctx.data_index >= total_expected_length) {
                // 校验和
                uint8_t checksum = 0;
                for (uint8_t i = 2; i < 2 + 1 + ctx.expected_length; i++) {
                    checksum += ctx.data_buffer[i];
                }
                if (checksum == ctx.data_buffer[total_expected_length - 1]) {
                    ctx.state = ParseState::WAIT_HEADER1;
                    ctx.frame_count++;
                    return true;
                } else {
                    ctx.state = ParseState::WAIT_HEADER1;
                }
            }
            break;
    }
    return false;
}

// 三路解析上下文
static RedundancyParseContext parse_context_from_FPGA;
static RedundancyParseContext parse_context_from_FMUa;
static RedundancyParseContext parse_context_from_FMUb;

// 余度切换初始化函数
void Plane::init_redundancy_control()
{
    // 检查是否已经初始化
    if (redundancy_initialized) {
        return;
    }

    // 获取serial6串口驱动，用于与FPGA通信
    redundancy_uart_to_FPGA = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_redundancy_FPGA, 0);
    
    if (redundancy_uart_to_FPGA == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Redundancy: Serial6 not available");
        return;
    }
    // 配置串口参数
    redundancy_uart_to_FPGA->begin(460800, 256, 256);
    redundancy_uart_to_FPGA->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

    // 获取serial7串口驱动，用于与FMUa通信
    redundancy_uart_to_FMUa = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_redundancy_FMUa, 0);
    if (redundancy_uart_to_FMUa == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Redundancy: Serial7 not available");
        return;
    }
    // 配置串口参数
    redundancy_uart_to_FMUa->begin(460800, 256, 256);
    redundancy_uart_to_FMUa->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

    // 获取serial8串口驱动，用于与FMUb通信
    redundancy_uart_to_FMUb = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_redundancy_FMUb, 0);
    if (redundancy_uart_to_FMUb == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Redundancy: Serial8 not available");
        return;
    }
    // 配置串口参数
    redundancy_uart_to_FMUb->begin(460800, 256, 256);
    redundancy_uart_to_FMUb->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

    // 标记初始化完成
    redundancy_initialized = true;
    last_heartbeat_from_FPGA_ms = 0;
    
    gcs().send_text(MAV_SEVERITY_INFO, "Redundancy initialized");
}

// 余度切换状态更新函数
void Plane::update_redundancy_control()
{
    if (!redundancy_initialized || redundancy_uart_to_FPGA == nullptr || redundancy_uart_to_FMUa == nullptr || redundancy_uart_to_FMUb == nullptr) {
        return;
    }
    
    // 读取FPGA发来的串口数据
    read_redundancy_data_from_FPGA();

    // 读取FMUa发来的串口数据
    read_redundancy_data_from_FMUa();

    // 读取FMUb发来的串口数据
    read_redundancy_data_from_FMUb();

    // 10Hz定时逻辑
    static uint32_t last_check_time = 0;
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_check_time >= 100) {
        last_check_time = now_ms;
        // 只有余度编号都已获取时才允许进入后续逻辑
        if (last_ctrl_redundancy_num != 0 && this_redundancy_num != 0 && last_ctrl_redundancy_num != this_redundancy_num) {
            // 获取控制余度的解锁状态
            bool ctrl_unlocked = false;
            RedundancyCommFrame* ctrl_frame = nullptr;
            if (last_ctrl_redundancy_num == 1) {
                ctrl_frame = (this_redundancy_num == 2 || this_redundancy_num == 3) ? 
                    (RedundancyCommFrame*)parse_context_from_FMUa.data_buffer : nullptr;
            } else if (last_ctrl_redundancy_num == 2) {
                if (this_redundancy_num == 1) {
                    ctrl_frame = (RedundancyCommFrame*)parse_context_from_FMUa.data_buffer;
                } else if (this_redundancy_num == 3) {
                    ctrl_frame = (RedundancyCommFrame*)parse_context_from_FMUb.data_buffer;
                }
            } else if (last_ctrl_redundancy_num == 3) {
                ctrl_frame = (this_redundancy_num == 1 || this_redundancy_num == 2) ? 
                    (RedundancyCommFrame*)parse_context_from_FMUb.data_buffer : nullptr;
            }
            if (ctrl_frame) {
                ctrl_unlocked = (ctrl_frame->unlock_status != 0);
            }
            // 如果控制余度已解锁，且本余度未解锁
            if (ctrl_unlocked) {
                if (arming.is_armed()) {
                    // 自身已解锁，跟随控制余度，持续设置当前位置为目标位置
                    Location tmp_loc;
                    ahrs.get_location(tmp_loc);
                    set_target_location(tmp_loc);
                } else {
                    // 自身未解锁，自动解锁并切模式
                    arming.arm(AP_Arming::Method::SCRIPTING);
                    set_mode(Mode::Number::LOITER, ModeReason::SCRIPTING);
                    Location tmp_loc;
                    ahrs.get_location(tmp_loc);
                    set_target_location(tmp_loc);
                    gcs().send_text(MAV_SEVERITY_INFO, "Auto arm and switch to LOITER, hold position");
                }
            }
        }
    }

    // 1Hz调试输出
    static uint32_t last_debug_time = 0;
    if (now_ms - last_debug_time >= 1000) {
        last_debug_time = now_ms;
        uint8_t eval_health = evaluate_redundancy_health();
        gcs().send_text(MAV_SEVERITY_INFO, "EvalHealth=0x%02X, FPGAHealth=0x%02X", eval_health, redundancy_status);
    }

    // // 测试余度自动切换逻辑：如果当前余度为余度1，则在其解锁10秒后自动进入while(1)死循环
    // if (this_redundancy_num == 1 && arming.is_armed() && arm_time_ms != 0) {
    //     uint32_t armed_duration_ms = now_ms - arm_time_ms;
    //     if (armed_duration_ms >= 10000) { // 10秒 = 10000毫秒
    //         gcs().send_text(MAV_SEVERITY_CRITICAL, "Redundancy1: Entering test dead loop after 10s armed");
    //         while(1) {
    //             // 死循环，用于测试余度自动切换
    //             hal.scheduler->delay(100); // 避免看门狗复位
    //         }
    //     }
    // }

    // 准备发送给FMUa、FMUb、FPGA的帧
    RedundancyCommFrame frame_to_FPGA_FMUa_FMUb;
    
    // 获取速度数据
    Vector3f vel_ned;
    if (!ahrs.get_velocity_NED(vel_ned)) {
        vel_ned.zero();  // 如果获取失败，使用零值
    }
    
    // 获取位置数据
    Location redundancy_loc;
    ahrs.get_location(redundancy_loc);
    
    // 获取PWM输出数据
    uint16_t pwm_values[14] = {0};
    for (uint8_t i = 0; i < 14; i++) {
        pwm_values[i] = SRV_Channels::srv_channel(i)->get_output_pwm();
    }
    
    // 余度健康度评价值
    uint8_t all_redundancy_health = evaluate_redundancy_health();
    
    pack_redundancy_comm_frame(frame_to_FPGA_FMUa_FMUb,
                               last_heartbeat_from_FPGA_ms,
                               all_redundancy_health,         // 余度健康度评价值（临时为0）
                               (uint8_t)control_mode->mode_number(),  // 当前飞行模式编号
                               (uint8_t)arming.is_armed(),        // 解锁状态
                               degrees(ahrs.get_roll()),       // 横滚角
                               degrees(ahrs.get_pitch()),      // 俯仰角
                               degrees(ahrs.get_yaw()),        // 偏航角
                               vel_ned.x,                      // 北向速度，m/s
                               vel_ned.y,                      // 东向速度，m/s
                               vel_ned.z,                      // 地向速度，m/s
                               redundancy_loc.lat,             // 纬度，1E7 deg
                               redundancy_loc.lng,             // 经度，1E7 deg
                               redundancy_loc.alt,             // 高度，cm
                               pwm_values);                    // PWM输出值数组

    // 发送给FPGA
    redundancy_uart_to_FPGA->write((uint8_t*)&frame_to_FPGA_FMUa_FMUb, sizeof(frame_to_FPGA_FMUa_FMUb));

    // 发送给FMUa
    redundancy_uart_to_FMUa->write((uint8_t*)&frame_to_FPGA_FMUa_FMUb, sizeof(frame_to_FPGA_FMUa_FMUb));

    // 发送给FMUb
    redundancy_uart_to_FMUb->write((uint8_t*)&frame_to_FPGA_FMUa_FMUb, sizeof(frame_to_FPGA_FMUa_FMUb));
}

// 读取FPGA发来的串口数据
void Plane::read_redundancy_data_from_FPGA()
{
    if (redundancy_uart_to_FPGA == nullptr) return;
    uint8_t data;
    while (redundancy_uart_to_FPGA->read(&data, 1) == 1) {
        if (parse_redundancy_frame_byte(parse_context_from_FPGA, data)) {
            process_redundancy_frame_from_FPGA();
        }
    }
}

// 处理解析完的FPGA数据帧
void Plane::process_redundancy_frame_from_FPGA()
{
    // 将数据缓冲区转换为结构体
    RedundancyDataFrame_from_FPGA* frame = (RedundancyDataFrame_from_FPGA*)parse_context_from_FPGA.data_buffer;
    
    // 如果当前余度编号为0，则从余度状态字节的bit6~bit7提取余度编号
    if (this_redundancy_num == 0) {
        // 提取bit6~bit7作为余度编号 (右移6位后与0x03进行与操作)
        this_redundancy_num = (frame->redundancy_status >> 6) & 0x03;
        gcs().send_text(MAV_SEVERITY_INFO, "Set this_redundancy_num to %d", this_redundancy_num);
    }

    // 解析当前控制余度编号（bit3~bit4，取值1/2/3）
    uint8_t ctrl_redundancy_num = ((frame->redundancy_status >> 3) & 0x03) + 1; // 取bit3~4，+1使范围为1~3
    // 检查是否切入控制状态
    if (ctrl_redundancy_num == this_redundancy_num && this->last_ctrl_redundancy_num != this_redundancy_num) {
        gcs().send_text(MAV_SEVERITY_INFO, "--- This FMU%d is in control ---", this_redundancy_num);
    }
    this->last_ctrl_redundancy_num = ctrl_redundancy_num;

    // 更新余度状态 (bit0~5)
    redundancy_status = frame->redundancy_status & 0x3F;
    
    // 处理ADC数据 - 读取7路24位ADC值并转换为浮点数
    for (uint8_t i = 0; i < 7; i++) {
        uint32_t raw_adc_value = get_adc_value_24bit(frame->adc_values, i);
        float adc_voltage = convert_adc_to_float(raw_adc_value);
        
        // 存储到类的成员变量中
        adc_value_from_FPGA[i] = adc_voltage;
    }

    // 将两路供电电压值设置到电池监控器程序中
#if AP_BATTERY_SCRIPTING_ENABLED
    // 使用脚本接口设置电池电压值
    BattMonitorScript_State batt_state;
    batt_state.voltage = adc_value_from_FPGA[3] * g2.batt1_voltage_mult;  // 设置电池0的电压
    batt_state.healthy = true;
    batt_state.cell_count = 0;  // 没有单电池电压信息
    battery.handle_scripting(0, batt_state);
    
    batt_state.voltage = adc_value_from_FPGA[4] *g2.batt2_voltage_mult;  // 设置电池1的电压
    battery.handle_scripting(1, batt_state);
#endif

    // 记录时间戳
    last_heartbeat_from_FPGA_ms = frame->timestamp;
    
    // 每秒输出帧解析统计
    uint32_t now = AP_HAL::millis();
    if (now - parse_context_from_FPGA.last_frame_count_time >= 1000) { // 每秒统计一次
        gcs().send_text(MAV_SEVERITY_INFO, "Redundancy: FPS=%lu, Status=0x%02X", 
                       (unsigned long)parse_context_from_FPGA.frame_count, frame->redundancy_status);
        parse_context_from_FPGA.frame_count = 0; // 重置帧计数
        parse_context_from_FPGA.last_frame_count_time = now;
    }
}

// 打包余度间通信帧
void pack_redundancy_comm_frame(RedundancyCommFrame &frame, 
    uint32_t timestamp, uint8_t redundancy_health, uint8_t work_mode, uint8_t unlock_status,
    float roll_deg, float pitch_deg, float yaw_deg,
    float vel_n, float vel_e, float vel_d,
    int32_t lat, int32_t lng, int32_t alt,
    const uint16_t pwm[14])
{
    frame.header[0] = 0xA5;
    frame.header[1] = 0x5A;
    frame.frame_length = sizeof(RedundancyCommFrame) - 4;  // 自动计算帧长，减去帧头2字节、帧长1个字节、校验和1个字节
    frame.timestamp = timestamp;
    frame.redundancy_health = redundancy_health;
    frame.work_mode = work_mode;
    frame.unlock_status = unlock_status;
    frame.roll_deg = roll_deg;
    frame.pitch_deg = pitch_deg;
    frame.yaw_deg = yaw_deg;
    frame.vel_n = vel_n;
    frame.vel_e = vel_e;
    frame.vel_d = vel_d;
    frame.lat = lat;
    frame.lng = lng;
    frame.alt = alt;
    for (int i = 0; i < 14; i++) frame.pwm[i] = pwm[i];
    // 校验和计算（从frame_length到最后一个pwm）
    uint8_t *p = (uint8_t*)&frame.frame_length;
    uint8_t checksum = 0;
    for (int i = 0; i < frame.frame_length; i++) checksum += p[i];
    frame.checksum = checksum;
}

// 读取FMUa发来的串口数据
void Plane::read_redundancy_data_from_FMUa()
{
    if (redundancy_uart_to_FMUa == nullptr) return;
    uint8_t data;
    while (redundancy_uart_to_FMUa->read(&data, 1) == 1) {
        if (parse_redundancy_frame_byte(parse_context_from_FMUa, data)) {
            process_redundancy_frame_from_FMUa();
        }
    }
}

// 处理FMUa数据帧
void Plane::process_redundancy_frame_from_FMUa()
{
    RedundancyCommFrame* frame = (RedundancyCommFrame*)parse_context_from_FMUa.data_buffer;
    fmu_a_last_fpga_timestamp = frame->timestamp;
    // 每秒输出帧解析统计
    uint32_t now = AP_HAL::millis();
    if (now - parse_context_from_FMUa.last_frame_count_time >= 1000) { // 每秒统计一次
        gcs().send_text(MAV_SEVERITY_INFO, "FMUa: FPS=%lu, mode=%d, unlock=%d, health=0x%02X", 
                       (unsigned long)parse_context_from_FMUa.frame_count, 
                       frame->work_mode, frame->unlock_status,
                       frame->redundancy_health);
        parse_context_from_FMUa.frame_count = 0; // 重置帧计数
        parse_context_from_FMUa.last_frame_count_time = now;
    }
}

// 读取FMUb发来的串口数据
void Plane::read_redundancy_data_from_FMUb()
{
    if (redundancy_uart_to_FMUb == nullptr) return;
    uint8_t data;
    while (redundancy_uart_to_FMUb->read(&data, 1) == 1) {
        if (parse_redundancy_frame_byte(parse_context_from_FMUb, data)) {
            process_redundancy_frame_from_FMUb();
        }
    }
}

// 处理FMUb数据帧
void Plane::process_redundancy_frame_from_FMUb()
{
    RedundancyCommFrame* frame = (RedundancyCommFrame*)parse_context_from_FMUb.data_buffer;
    fmu_b_last_fpga_timestamp = frame->timestamp;
    // 每秒输出帧解析统计
    uint32_t now = AP_HAL::millis();
    if (now - parse_context_from_FMUb.last_frame_count_time >= 1000) { // 每秒统计一次
        gcs().send_text(MAV_SEVERITY_INFO, "FMUb: FPS=%lu, mode=%d, unlock=%d, health=0x%02X", 
                       (unsigned long)parse_context_from_FMUb.frame_count, 
                       frame->work_mode, frame->unlock_status,
                       frame->redundancy_health);
        parse_context_from_FMUb.frame_count = 0; // 重置帧计数
        parse_context_from_FMUb.last_frame_count_time = now;
    }
}

// 余度健康度评价值计算函数
uint8_t Plane::evaluate_redundancy_health()
{
    // 1. 未知自身余度编号，全部健康（待完善，是否根据当前传感器健康度综合判断？）
    if (this_redundancy_num == 0) {
        return 0x07;
    }
    uint8_t health = 0;
    // 2. 余度编号映射与健康判断（待完善，是否根据三个余度在姿态解算、输出值方面的差值综合判断？）
    // 获取各余度的健康状态
    bool self_healthy = true; // 自身始终健康
    bool fmu_a_healthy = (fmu_a_last_fpga_timestamp >= last_heartbeat_from_FPGA_ms - 10);
    bool fmu_b_healthy = (fmu_b_last_fpga_timestamp >= last_heartbeat_from_FPGA_ms - 10);
    // 3. bit映射
    switch (this_redundancy_num) {
        case 1: // 余度1
            if (self_healthy) health |= 0x01;      // bit0
            if (fmu_a_healthy) health |= 0x02;     // bit1
            if (fmu_b_healthy) health |= 0x04;     // bit2
            break;
        case 2: // 余度2
            if (fmu_a_healthy) health |= 0x01;     // bit0
            if (self_healthy) health |= 0x02;      // bit1
            if (fmu_b_healthy) health |= 0x04;     // bit2
            break;
        case 3: // 余度3
            if (fmu_a_healthy) health |= 0x01;     // bit0
            if (fmu_b_healthy) health |= 0x02;     // bit1
            if (self_healthy) health |= 0x04;      // bit2
            break;
        default:
            // 理论上不会到这里，保险起见全部健康
            health = 0x07;
            break;
    }
    return health;
}

// 获取指定通道的24位ADC值
uint32_t get_adc_value_24bit(const uint8_t* adc_data, uint8_t channel)
{
    if (channel >= 7) {
        return 0; // 通道号超出范围
    }
    
    // 计算该通道在数组中的起始位置（每个通道3字节）
    const uint8_t* channel_data = adc_data + channel * 3;
    
    // 24位有符号整数
    uint32_t value = ((uint32_t)channel_data[0] << 16) | 
                    ((uint32_t)channel_data[1] << 8) | 
                    (uint32_t)channel_data[2];
    
    return value;
}

// 将24位ADC原始值转换为浮点数
float convert_adc_to_float(uint32_t num)
{
    float temp;
    
    // 检查是否为负数（bit23为1）
    if (num >= (1 << 23)) {
        // 负数：使用二进制补码转换
        temp = -(float)(((1 << 24) - num) / (float)(1 << 17));
    } else {
        // 正数：直接除以缩放因子
        temp = (float)(num / (float)(1 << 17));
    }
    
    return temp;
}

#endif
