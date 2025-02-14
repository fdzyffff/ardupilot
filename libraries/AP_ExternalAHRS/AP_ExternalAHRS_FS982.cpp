#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_FS982_ENABLED

#include "AP_ExternalAHRS_FS982.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Common/Bitmask.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL &hal;

// constructor
AP_ExternalAHRS_FS982::AP_ExternalAHRS_FS982(AP_ExternalAHRS *_frontend,
                                             AP_ExternalAHRS::state_t &_state) : AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "FS982 ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    // don't offer IMU by default, at 100Hz it is too slow for many aircraft
    // 默认不提供IMU信息，因为100Hz的更新频率对于大多数载具是不够的
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS));

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_FS982::update_thread, void), "FS982", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0))
    {
        AP_HAL::panic("FS982 Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FS982 ExternalAHRS initialised");
}

/*
  check the UART for more data
  returns true if we have consumed potentially valid bytes
 */
bool AP_ExternalAHRS_FS982::check_uart()
{
    WITH_SEMAPHORE(state.sem);

    if (!setup_complete)
    { // 未初始化
        return false;
    }

    // ensure we own the uart
    uart->begin(0);
    uint32_t n = uart->available();
    if (n == 0)
    { // 串口中无数据
        return false;
    }

    uint8_t new_byte;

    for (uint32_t i = 0; i < n; i++)
    {
        if (!uart->read(new_byte))
        { // 依次读取所有字节
            return false;
        }

        switch (decode_state)
        {
        case 0: // 找帧头1
            if (new_byte == 0xAA)
            {
                nav_msg.nav.header1 = new_byte;
                decode_state++;
            }
            break;

        case 1:
            if (new_byte == 0x55) // 找帧头2
            {
                nav_msg.nav.header2 = new_byte;
                decode_state++;
            }
            else
            {
                decode_state = 0;
            }
            break;

        case 2: // 帧ID低8位
            nav_msg.nav.ID = new_byte;
            decode_state++;
            break;

        case 3: // 帧ID高8位
            nav_msg.nav.ID += ((uint16_t)new_byte << 8);
            decode_state++;
            break;

        case 4: // 帧长低8位
            nav_msg.nav.data_length = new_byte;
            decode_state++;
            break;

        case 5: // 帧长高8位
            nav_msg.nav.data_length += ((uint16_t)new_byte << 8);
            if (nav_msg.nav.data_length > sizeof(nav_msg))
            {
                decode_state = 0;
                continue;
            }

            frame_data_counter = 0;
            decode_state++;
            break;

        case 6: // 帧数据
            nav_msg.bytes[frame_data_counter + 6] = new_byte;
            frame_data_counter++;
            if (frame_data_counter >= nav_msg.nav.data_length)
            {
                decode_state++;
            }
            break;

        case 7: // CRC bit0~7
            received_crc = new_byte;
            decode_state++;
            break;

        case 8: // CRC bit8~15
            received_crc += ((uint32_t)new_byte << 8);
            decode_state++;
            break;

        case 9: // CRC bit16~23
            received_crc += ((uint32_t)new_byte << 16);
            decode_state++;
            break;

        case 10: // CRC bit24~31
            decode_state = 0;
            received_crc += ((uint32_t)new_byte << 24);
            calculated_crc = crc_crc32(1, nav_msg.bytes, nav_msg.nav.data_length + 6);
            if (received_crc == calculated_crc) // 校验通过
            {
                parse_msg(); // 提取帧中消息
                return true;
            }
            else
            {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FS982: CRC error");
                return false;
            }
            break;

        default:
            decode_state = 0;
            break;
        }
    }

    return false;
}

void AP_ExternalAHRS_FS982::parse_msg()
{
    if (nav_msg.nav.ID == 0x0166)
    {
        last_nav_msg_ms = AP_HAL::millis();
        state.last_location_update_us = AP_HAL::millis();

        // 解析GNSS信息
        gps_data.gps_week = nav_msg.nav.GNSS_week;
        gps_data.ms_tow = nav_msg.nav.GNSS_tow_ms;

        switch (nav_msg.nav.fix_type)
        {
        case 0:
            gps_data.fix_type = 1;
            break;

        case 1:
            gps_data.fix_type = 3;
            break;

        case 2:
            gps_data.fix_type = 4;
            break;

        case 4:
            gps_data.fix_type = 6;
            break;

        case 5:
            gps_data.fix_type = 5;
            break;

        default:
            gps_data.fix_type = 0;
            break;
        }

        gps_data.satellites_in_view = nav_msg.nav.sat_num;
        gps_data.hdop = nav_msg.nav.hdop_cm;
        gps_data.latitude = nav_msg.nav.latitude;
        gps_data.longitude = nav_msg.nav.longitude;
        gps_data.msl_altitude = nav_msg.nav.altitude_mm / 10;
        gps_data.ned_vel_north = nav_msg.nav.velocity_north_m_s;
        gps_data.ned_vel_east = nav_msg.nav.velocity_east_m_s;
        gps_data.ned_vel_down = nav_msg.nav.velocity_down_m_s;

        uint8_t instance;
        if (AP::gps().get_first_external_instance(instance))
        {
            AP::gps().handle_external(gps_data, instance);
        }

        if ((gps_data.fix_type >= 3) && (gps_data.satellites_in_view >= 8)) // 已经良好定位
        {
            if (!state.have_origin) // 未设置原点
            {
                state.origin = Location{
                    gps_data.latitude,
                    gps_data.longitude,
                    gps_data.msl_altitude,
                    Location::AltFrame::ABSOLUTE};
                state.have_origin = true;
            }
        }

        // 解析位置信息
        state.location.lat = nav_msg.nav.latitude;
        state.location.lng = nav_msg.nav.longitude;
        state.location.alt = nav_msg.nav.altitude_mm / 10;
        state.have_location = true;

        // 解析速度信息
        state.velocity = Vector3f(nav_msg.nav.velocity_north_m_s,
                                     nav_msg.nav.velocity_east_m_s,
                                     nav_msg.nav.velocity_down_m_s);
        state.have_velocity = true;

        // 解析姿态信息
        state.quat.from_euler(radians(nav_msg.nav.roll_deg),
                              radians(nav_msg.nav.pitch_deg),
                              radians(nav_msg.nav.yaw_deg));
        state.have_quaternion = true;

        // 解析IMU信息
        Vector3f accel = Vector3f(nav_msg.nav.acc_x_g, nav_msg.nav.acc_y_g, nav_msg.nav.acc_z_g);
        ins_data.accel = accel * GRAVITY_MSS;
        Vector3f gyro = Vector3f(nav_msg.nav.gyro_x_deg_s, nav_msg.nav.gyro_y_deg_s, nav_msg.nav.gyro_z_deg_s);
        ins_data.gyro = gyro * DEG_TO_RAD;
        ins_data.temperature = nav_msg.nav.temperature_dc;
        AP::ins().handle_external(ins_data);
        state.accel = ins_data.accel;
        state.gyro = ins_data.gyro;
    }
}

void AP_ExternalAHRS_FS982::update_thread()
{
    // Open port in the thread
    uart->begin(baudrate, 1024, 512);

    /*
      we assume the user has already configured the device
     */

    setup_complete = true;
    while (true)
    {
        check_uart();
        hal.scheduler->delay_microseconds(100); // 每100us查看一下串口数据
    }
}

static const uint32_t crc32_tab[] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d};

uint32_t AP_ExternalAHRS_FS982::crc_crc32(uint32_t crc, const uint8_t *buf, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        crc = crc32_tab[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);
    }
    return crc;
}

// get serial port number for the uart
int8_t AP_ExternalAHRS_FS982::get_port(void) const
{
    if (!uart)
    {
        return -1;
    }
    return port_num;
};

// accessors for AP_AHRS
bool AP_ExternalAHRS_FS982::healthy(void) const
{
    WITH_SEMAPHORE(state.sem);
    return AP_HAL::millis() - last_nav_msg_ms < 50;
}

bool AP_ExternalAHRS_FS982::initialised(void) const
{
    if (!setup_complete)
    {
        return false;
    }
    return true;
}

bool AP_ExternalAHRS_FS982::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!setup_complete)
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "FS982 setup failed");
        return false;
    }
    if (!healthy())
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "FS982 unhealthy");
        return false;
    }
    WITH_SEMAPHORE(state.sem);
    uint32_t now = AP_HAL::millis();
    if (now - last_nav_msg_ms > 20)
    {
        hal.util->snprintf(failure_msg, failure_msg_len, "FS982 not up to date");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_FS982::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    if (last_nav_msg_ms != 0)
    {
        status.flags.initalized = true;
    }
    if (healthy() && last_nav_msg_ms != 0)
    {
        status.flags.attitude = true;

        // if (nav_msg.nav.INS_status & (0x01 << 4))  // 此值只有在进入RTK状态才会置一
        if ((gps_data.fix_type >= 3) && (gps_data.satellites_in_view >= 8))  // 定位良好
        {
            status.flags.vert_vel = true;
            status.flags.vert_pos = true;
            status.flags.horiz_vel = true;
            status.flags.horiz_pos_rel = true;
            status.flags.horiz_pos_abs = true;
            status.flags.pred_horiz_pos_rel = true;
            status.flags.pred_horiz_pos_abs = true;
            status.flags.using_gps = true;
        }
    }
}

// send an EKF_STATUS message to GCS
void AP_ExternalAHRS_FS982::send_status_report(GCS_MAVLINK &link) const
{
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude)
    {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel)
    {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel)
    {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel)
    {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs)
    {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos)
    {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt)
    {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode)
    {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel)
    {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs)
    {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized)
    {
        flags |= EKF_UNINITIALIZED;
    }

    // send message
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags,
                                       0,
                                       0,
                                       0,
                                       0, 
                                       0, 
                                       0);
}

#endif // AP_EXTERNAL_AHRS_FS982_ENABLED
