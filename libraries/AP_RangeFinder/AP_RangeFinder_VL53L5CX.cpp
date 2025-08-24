/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  driver for ST VL53L5CX lidar

  Many thanks to Pololu, https://github.com/pololu/vl53l5Cx-arduino and
  the ST example code
 */
#include <string.h>
#include "AP_RangeFinder_VL53L5CX.h"
#include <GCS_MAVLink/GCS.h>

#if AP_RANGEFINDER_VL53L5CX_ENABLED

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>
#include <cstddef>

extern const AP_HAL::HAL& hal;

static const uint8_t MEASUREMENT_TIME_MS = 50; // Start continuous readings at a rate of one measurement every 50 ms

AP_RangeFinder_VL53L5CX::AP_RangeFinder_VL53L5CX(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev)
    : AP_RangeFinder_Backend(_state, _params)
    , dev(std::move(_dev)) 
{
    // _print_enable_text = true;
    // _print_enable_gcs = true;
}

/*
   detect if a VL53L5CX rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_VL53L5CX::detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, DistanceMode mode)
{
    if (!dev) 
    {
        return nullptr;
    }

    AP_RangeFinder_VL53L5CX *sensor = new AP_RangeFinder_VL53L5CX(_state, _params, std::move(dev));

    if (!sensor) 
    {
        delete sensor;
        return nullptr;
    }

    sensor->dev->get_semaphore()->take_blocking();

    if (!sensor->check_id() || !sensor->init()) 
    {
        sensor->dev->get_semaphore()->give();
        delete sensor;
        printf("\r\nVL53 Init detect Fail\r\n");
        gcs().send_text(MAV_SEVERITY_INFO, "VL53 Init detect Fail");
        return nullptr;
    }
    sensor->dev->get_semaphore()->give();

    return sensor;
}

// check sensor ID registers
bool AP_RangeFinder_VL53L5CX::check_id(void)
{
    uint8_t v1, v2;
    bool Res = false;

    Res = write_register(0x7FFF, 0x00);
    if(!(read_register(0x00, v1) && read_register(0x01, v2))) 
    {
        if (_print_enable_text) {printf("\r\n\r\n Read ID is False \r\n\r\n");}
        if (_print_enable_gcs) {gcs().send_text(MAV_SEVERITY_INFO, "Read ID is False");}
        return false;
    }
    if ((v1 != 0xF0) || (v2 != 0x02))
    {
        if (_print_enable_text) {printf("\r\n\r\n Check ID is different :0x%x,0x%x,%d \r\n\r\n",v1,v2,Res);}
        if (_print_enable_gcs) {gcs().send_text(MAV_SEVERITY_INFO, "Check ID is different :0x%x,0x%x,%d \r\n\r\n",v1,v2,Res);}
        return false;
    }
    Res = write_register(0x7FFF, 0x02);             
    if (_print_enable_text) {printf("Detected VL53L5CX on bus 0x%x\r\n", dev->get_bus_id());}      //0x2901
    if (_print_enable_gcs) {gcs().send_text(MAV_SEVERITY_INFO, "Detected VL53L5CX on bus 0x%lx\r\n", (long)dev->get_bus_id());}      //0x2901
    return Res;
}

bool AP_RangeFinder_VL53L5CX::reset(void) 
{
    if (dev->get_bus_id()!= 0x29) 
    {
        // if sensor is on a different port than the default do not  reset sensor otherwise we will lose the addess.
        // we assume it is already confirgured.
        return true;
    }
    if (!write_register(SOFT_RESET, 0x00)) 
    {
        return false;
    }
    hal.scheduler->delay_microseconds(100);
    if (!write_register(SOFT_RESET, 0x01)) 
    {
        return false;
    }
    hal.scheduler->delay(1000);
    return true;
}
//_vl53l5cx_poll_for_answer
bool AP_RangeFinder_VL53L5CX::PollForAnser(uint8_t Size,uint8_t pos,uint16_t address,uint8_t mask,uint8_t expected_value)
{

    uint16_t Cnt = 0;
    bool Rec = false;

    if(pos >= Size)         //
        return Rec;
        
    while(1)
    {
        ReadData(address,temp_buffer, Size);        
        hal.scheduler->delay(10);   
        if(Cnt >= 200)
        {
            Rec = false;                            //返回错误
            if (_print_enable_text) {printf("\r\n Over Time \r\n");}
            if (_print_enable_gcs) {gcs().send_text(MAV_SEVERITY_INFO, "Over Time");}
            break;
        }
        else if((Size > 3) && temp_buffer[2] >= 0x7F)
        {
            if (_print_enable_text) {printf("\r\nMUC ERROR\r\n");}
            if (_print_enable_gcs) {gcs().send_text(MAV_SEVERITY_INFO, "MUC ERROR");}
            Rec = false;
            break;
        }
        else
            Cnt++;
        if((temp_buffer[pos] & mask) == expected_value)
        {
            Rec = true; 
            break;  
        }       
    }
    return Rec;
}
//_vl53l5cx_poll_for_mcu_boot
bool AP_RangeFinder_VL53L5CX::PollMCU_Boot(void)
{
    uint8_t go2_status0 = 0, go2_status1 = 0;
    uint16_t Cnt = 0;
    bool Rec = false;
    while(1)
    {
        Rec = read_register(0x06,go2_status0);
    
        if((go2_status0 & 0x80) != 0)                       
        {
            Rec = read_register(0x07,go2_status1);
            if (_print_enable_text) {printf("\r\ngo2_status1:%d\r\n",go2_status1);}
            break;
        }
        hal.scheduler->delay(1);
        Cnt++;
        if((go2_status0 & 0x01) != 0)
        {
            if (_print_enable_text) {printf("\r\nMCU ReBoot Finish\r\n");}
            if (_print_enable_gcs) {gcs().send_text(MAV_SEVERITY_INFO, "MCU ReBoot Finish");}
            Rec = true;
            break;
        }
        if(Cnt > 500)
        {
            //if (_print_enab_textle) {printf("\r\nMCU ReBoot OverTime\r\n");}
            Rec = false;
            break;
        }
    }
    return Rec;
}
//_vl53l5cx_send_offset_data
bool AP_RangeFinder_VL53L5CX::SendOffsetData(uint8_t resolution)
{
    uint32_t SignalGrid[64];
    uint16_t  RangGrid[64];
    uint8_t  Dss_4x4[8] = {0x0F,0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07};
    uint8_t  Footer[8] = {0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x01, 0xE4};
    uint16_t  i, j;
    uint16_t k;
    bool Rec = false;

    (void)memcpy(temp_buffer,offset_data, VL53L5CX_OFFSET_BUFFER_SIZE);

    /* Data extrapolation is required for 4X4 offset */
    if(resolution == VL53L5CX_RESOLUTION_4X4)
    {
        (void)memcpy(&temp_buffer[0x10], Dss_4x4, sizeof(Dss_4x4));
        SwapBuffer(temp_buffer, VL53L5CX_OFFSET_BUFFER_SIZE);
        (void)memcpy(SignalGrid,&temp_buffer[0x3C],sizeof(SignalGrid));
        (void)memcpy(RangGrid,&temp_buffer[0x140],sizeof(RangGrid));        
    
        for (j = 0; j < 4; j++)
        {
            for (i = 0; i < 4 ; i++)
            {
                SignalGrid[i+(4*j)] =(SignalGrid[(2*i)+(16*j)+ 0] + SignalGrid[(2*i)+(16*j)+1] + SignalGrid[(2*i)+(16*j)+8] + SignalGrid[(2*i)+(16*j)+9]) /4;
                RangGrid[i+(4*j)] = (RangGrid[(2*i)+(16*j)]+ RangGrid[(2*i)+(16*j)+1]+ RangGrid[(2*i)+(16*j)+8]+ RangGrid[(2*i)+(16*j)+9])/4;
            }
        }
        (void)memset(&RangGrid[0x10], 0, 96);
        (void)memset(&SignalGrid[0x10], 0, 192);
        (void)memcpy(&temp_buffer[0x3C],SignalGrid, sizeof(SignalGrid));
        (void)memcpy(&temp_buffer[0x140],RangGrid, sizeof(RangGrid));
         SwapBuffer(temp_buffer, VL53L5CX_OFFSET_BUFFER_SIZE);      
    }
    for(k = 0; k < (VL53L5CX_OFFSET_BUFFER_SIZE - 4); k++)
    {
        temp_buffer[k] = temp_buffer[k+8];
    }
    (void)memcpy(&temp_buffer[0x1E0], Footer, 8);
    WriteData(0x2E18,temp_buffer,VL53L5CX_OFFSET_BUFFER_SIZE);
    Rec = PollForAnser(4,1,VL53L5CX_UI_CMD_STATUS,0xFF,0x03);
    return Rec;
}
//_vl53l5cx_send_xtalk_data
bool AP_RangeFinder_VL53L5CX::SendXTalkData(uint8_t resolution)
{
    uint8_t res4x4[] = {0x0F, 0x04, 0x04, 0x17, 0x08, 0x10, 0x10, 0x07};
    uint8_t dss_4x4[] = {0x00, 0x78, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08};
    uint8_t profile_4x4[] = {0xA0, 0xFC, 0x01, 0x00};
    uint32_t signal_grid[64];
    uint16_t i, j;
    bool Rec = false;

    (void)memcpy(temp_buffer, &(xtalk_data[0]),VL53L5CX_XTALK_BUFFER_SIZE);
    
    if(resolution == VL53L5CX_RESOLUTION_4X4)
    {
        (void)memcpy(&(temp_buffer[0x8]),   res4x4, sizeof(res4x4));
        (void)memcpy(&(temp_buffer[0x020]), dss_4x4, sizeof(dss_4x4));

        SwapBuffer(temp_buffer, VL53L5CX_XTALK_BUFFER_SIZE);
        (void)memcpy(signal_grid, &(temp_buffer[0x34]),sizeof(signal_grid));

        for (j = 0; j < (int8_t)4; j++)
        {
            for (i = 0; i < (int8_t)4 ; i++)
            {
                signal_grid[i+(4*j)] =(signal_grid[(2*i)+(16*j)+0] + signal_grid[(2*i)+(16*j)+1] + signal_grid[(2*i)+(16*j)+8]  + signal_grid[(2*i)+(16*j)+9])/(uint32_t)4;
            }
        }
        (void)memset(&signal_grid[0x10], 0, (uint32_t)192);
        (void)memcpy(&(temp_buffer[0x34]),signal_grid, sizeof(signal_grid));
        SwapBuffer(temp_buffer, VL53L5CX_XTALK_BUFFER_SIZE);
        (void)memcpy(&(temp_buffer[0x134]),profile_4x4, sizeof(profile_4x4));
        (void)memset(&(temp_buffer[0x078]),0 ,(uint32_t)4*sizeof(uint8_t));     
    }
    WriteData(0x2CF8,temp_buffer,VL53L5CX_XTALK_BUFFER_SIZE);       //写入数据
    Rec = PollForAnser(4,1,VL53L5CX_UI_CMD_STATUS,0xFF,0x03);       //确认数据写入完成
    return Rec;
}
//vl53l5cx_is_alive
bool AP_RangeFinder_VL53L5CX::Is_alive(void)
{
    bool Rec = false;
    uint8_t DevID = 0,RevID = 0;

    Rec = write_register(0x7FFF, 0x00);
    Rec = read_register(0,DevID);
    Rec = read_register(1,RevID);
    Rec = write_register(0x7FFF, 0x00);
    if((DevID == 0xF0) && (RevID == 0x02))
    {
        Rec = true;
        if (_print_enable_text) {printf("VL53L5 IS alive\r\n");}
        if (_print_enable_gcs) {gcs().send_text(MAV_SEVERITY_INFO, "VL53L5 IS alive");}
    }       
    return Rec;
}
/*
  initialise sensor
 */
//vl53l5cx_init
bool AP_RangeFinder_VL53L5CX::init()
{
    // we need to do resets and delays in order to configure the sensor, don't do this if we are trying to fast boot
    // if (hal.util->was_watchdog_armed()) 
    // {
    //     return false;
    // }
    uint8_t tmp;
    bool Rec = false;

    default_xtalk = (uint8_t*)VL53L5CX_DEFAULT_XTALK;
    default_configuration = (uint8_t*)VL53L5CX_DEFAULT_CONFIGURATION;
    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    /* SW reboot sequence */
    Rec = write_register(0x7FFF,0X00);
    Rec = write_register(0x0009,0X04);
    Rec = write_register(0x000F,0X40);
    Rec = write_register(0x000A,0X03);
    Rec = read_register(0x7FFF,tmp);
    Rec = write_register(0x000C,0X01);          //6
    Rec = write_register(0x0101, 0x00);
    Rec = write_register(0x0102, 0x00);
    Rec = write_register(0x010A, 0x01);
    Rec = write_register(0x4002, 0x01);
    Rec = write_register(0x4002, 0x00);
    Rec = write_register(0x010A, 0x03);
    Rec = write_register(0x0103, 0x01);
    Rec = write_register(0x000C, 0x00);
    Rec = write_register(0x000F, 0x43);         //15
    hal.scheduler->delay(1);
    Rec = write_register(0x000F, 0x40);
    Rec = write_register(0x000A, 0x01);         //17
    hal.scheduler->delay(100);
    /* Wait for sensor booted (several ms required to get sensor ready ) */
    Rec = write_register(0x7FFF,0X00);  
    Rec = PollForAnser(1,0,0x06,0xFF,1);        //19
    if(!Rec) 
    {
        if (_print_enable_text) {printf("\r\nWait Sensor boot Fail\r\n");}
        if (_print_enable_gcs) {gcs().send_text(MAV_SEVERITY_INFO, "Wait Sensor boot Fail");}
        return Rec;
    }       
    Rec = write_register(0x000E, 0x01);
    Rec = write_register(0x7fff, 0x02);     //21
    /* Enable FW access */
    Rec = write_register(0x03, 0x0D);
    Rec = write_register(0x7fff, 0x01); 
    Rec = PollForAnser(1,0,0x21,0x10,0x10); //22
    Rec = write_register(0x7fff, 0x00);
    /* Enable host access to GO1 */
    Rec = read_register(0x7fff,tmp);
    Rec = write_register(0x0C, 0x01);
    /* Power ON status */
    Rec = write_register(0x7fff, 0x00);
    Rec = write_register(0x101, 0x00);
    Rec = write_register(0x102, 0x00);
    Rec = write_register(0x010A, 0x01);
    Rec = write_register(0x4002, 0x01);
    Rec = write_register(0x4002, 0x00);
    Rec = write_register(0x010A, 0x03);
    Rec = write_register(0x103, 0x01);
    Rec = write_register(0x400F, 0x00);
    Rec = write_register(0x21A, 0x43);
    Rec = write_register(0x21A, 0x03);
    Rec = write_register(0x21A, 0x01);
    Rec = write_register(0x21A, 0x00);
    Rec = write_register(0x219, 0x00);
    Rec = write_register(0x21B, 0x00);//42
    /* Wake up MCU */
    Rec = write_register(0x7FFF, 0x00);
    Rec = read_register(0x7FFF, tmp);
    Rec = write_register(0x0C, 0x00);
    Rec = write_register(0x7FFF, 0x01);
    Rec = write_register(0x20, 0x07);
    Rec = write_register(0x20, 0x06);
    
    /* Download FW into VL53L5 */
    Rec = write_register(0x7FFF, 0x09);
    WriteData(0,(uint8_t *)&VL53L5CX_FIRMWARE[0],0x8000);
    Rec = write_register(0x7FFF, 0x0A);
    WriteData(0,(uint8_t *)&VL53L5CX_FIRMWARE[0x8000],0x8000);
    Rec = write_register(0x7FFF, 0x0B);
    WriteData(0,(uint8_t *)&VL53L5CX_FIRMWARE[0x10000],0x5000);
    Rec = write_register(0x7fff, 0x01);
    /* Check if FW correctly downloaded */
    Rec = write_register(0x7fff, 0x02);
    Rec = write_register(0x03, 0x0D);
    Rec = write_register(0x7fff, 0x01);
    Rec = PollForAnser(1,0,0x21,0x10,0x10); //23
    if(!Rec) 
    {
        if (_print_enable_text) {printf("\r\nDownload FW Fail\r\n");}
        if (_print_enable_gcs) {gcs().send_text(MAV_SEVERITY_INFO, "Download FW Fail");}
        return Rec;
    }   
    Rec = write_register(0x7FFF, 0x00);
    Rec = read_register(0x7FFF, tmp);
    Rec = write_register(0x0C, 0x01);
    /* Reset MCU and wait boot */
    Rec = write_register(0x7FFF, 0x00);
    Rec = write_register(0x114, 0x00);
    Rec = write_register(0x115, 0x00);
    Rec = write_register(0x116, 0x42);
    Rec = write_register(0x117, 0x00);
    Rec = write_register(0x0B, 0x00);
    Rec = read_register(0x7FFF, tmp);
    Rec = write_register(0x0C, 0x00);   
    Rec = write_register(0x0B, 0x01);
    Rec = PollMCU_Boot();                       //24
    if(!Rec) 
    {
        if (_print_enable_text) {printf("\r\nMCU Reboot Fail\r\n");}
        if (_print_enable_gcs) {gcs().send_text(MAV_SEVERITY_INFO, "MCU Reboot Fail");}
        return Rec;
    }   
    Rec = write_register(0x7FFF, 0x02);
    /* Get offset NVM data and store them into the offset buffer */
    WriteData(0x2FD8,(uint8_t *)&VL53L5CX_GET_NVM_CMD[0x0],sizeof(VL53L5CX_GET_NVM_CMD));
    Rec = PollForAnser(4,0,VL53L5CX_UI_CMD_STATUS,0xFF,2);      //25
    hal.scheduler->delay(10);

    ReadData(VL53L5CX_UI_CMD_START,temp_buffer, VL53L5CX_NVM_DATA_SIZE);
    memcpy(offset_data,temp_buffer,VL53L5CX_OFFSET_BUFFER_SIZE);
    Rec = SendOffsetData(VL53L5CX_RESOLUTION_4X4);
    /* Set default Xtalk shape. Send Xtalk to sensor */
    (void)memcpy(xtalk_data, (uint8_t*)VL53L5CX_DEFAULT_XTALK,VL53L5CX_XTALK_BUFFER_SIZE);
    Rec = SendXTalkData(VL53L5CX_RESOLUTION_4X4);
    /* Send default configuration to VL53L5CX firmware */   
    WriteData(0x2C34,default_configuration,sizeof(VL53L5CX_DEFAULT_CONFIGURATION));
    Rec = PollForAnser(4,1,VL53L5CX_UI_CMD_STATUS,0xFF,0x03);
    if(!Rec)
    {
        if (_print_enable_text) {printf("\r\n Download Default config is Error \r\n");}
        if (_print_enable_gcs) {gcs().send_text(MAV_SEVERITY_INFO, "Download Default config is Error");}
        return Rec;
    }
    else {
        if (_print_enable_text) {printf("\r\n Download Default config is OK \r\n");}
        if (_print_enable_gcs) {gcs().send_text(MAV_SEVERITY_INFO, "Download Default config is OK");}
    }

    uint8_t pipe_ctrl[] = {VL53L5CX_NB_TARGET_PER_ZONE, 0x00, 0x01, 0x00};
    uint32_t single_range = 0x01;
    DCI_WriteData(pipe_ctrl,VL53L5CX_DCI_PIPE_CONTROL,sizeof(pipe_ctrl));   
    Rec = DCI_WriteData((uint8_t *)&single_range,VL53L5CX_DCI_SINGLE_RANGE,sizeof(single_range));
    //完成初始化，配置VL53LC5的工作方式；
    if(Rec)
    {   
        Object.IsRanging = 0U;
        Object.IsBlocking = 0U;
        Object.IsContinuous = 0U;
        Object.IsAmbientEnabled = 0U;
        Object.IsSignalEnabled = 0U;
        Object.IsInitialized = 1U;
        Rec = Get_Capabilities(&Cap);
        if (_print_enable_text) {printf("\r\nGet Capabilities:%d\r\n",Rec);}
        if (_print_enable_gcs) {gcs().send_text(MAV_SEVERITY_INFO, "Get Capabilities:%d",Rec);}
        Profile.RangingProfile = VL53L5CX_PROFILE_8x8_CONTINUOUS;
        Profile.TimingBudget = TIMING_BUDGET;               /* 5 ms < TimingBudget < 100 ms */
        Profile.Frequency = RANGING_FREQUENCY;              /* Ranging frequency Hz (shall be consistent with TimingBudget value) */
        Profile.EnableAmbient = 0;                          /* Enable: 1, Disable: 0 */
        Profile.EnableSignal = 0;                           /* Enable: 1, Disable: 0 */     
        /* set the profile if different from default one */
        Rec = Set_ConfigProfile(&Object,&Profile);
        Rec = Start_Ranging(&Object,VL53L5CX_MODE_BLOCKING_CONTINUOUS); 
        
        if (_print_enable_text) {printf("\r\nStart Read Data:%d\r\n",Rec);}
        if (_print_enable_gcs) {gcs().send_text(MAV_SEVERITY_INFO, "Start Read Data:%d",Rec);}
    }

    if (Rec) {
        // call timer() every MEASUREMENT_TIME_MS. We expect new data to be available every MEASUREMENT_TIME_MS
        dev->register_periodic_callback(MEASUREMENT_TIME_MS * 1000,FUNCTOR_BIND_MEMBER(&AP_RangeFinder_VL53L5CX::timer, void));
    }

    // if (Rec) {
    //     if (!hal.scheduler->thread_create(
    //             FUNCTOR_BIND_MEMBER(&AP_RangeFinder_VL53L5CX::timer_loop, void), "TOFF", 4096,
    //             AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
    //         printf("AP_RangeFinder_VL53L5CX: couldn't create thread\n");
    //         return false;
    //     }
    // }

    // if (Rec) {
    //     timer_loop();
    // }
    return Rec;
}
uint8_t AP_RangeFinder_VL53L5CX::SwapBuffer(uint8_t *pBuf,uint16_t size)
{
  uint32_t i, tmp;

  /* Example of possible implementation using <string.h> */
  for(i = 0; i < size; i = i + 4)
  {
    tmp = (*(pBuf+i)<<24) |(*(pBuf+i+1)<<16)|(*(pBuf+i+2)<<8) |(*(pBuf+i+3));
    memcpy(pBuf+i, &tmp, 4);
  } 
  return 0;
}
//vl53l5cx_set_i2c_address
bool AP_RangeFinder_VL53L5CX::SetI2C_Addr(uint16_t Addr)
{
    bool Rec = false;

    Rec = write_register(0x7FFF, 0x00);
    Rec = write_register(0x04, (uint8_t)(Addr>>1));
    Rec = write_register(0x7FFF, 0x02);

    return Rec;
}
//vl53l5cx_get_power_mode
bool AP_RangeFinder_VL53L5CX::GetPowerMode(uint8_t &PowerMode)
{
    uint8_t tmp;
    bool Rec = false;

    Rec = write_register(0x7FFF, 0x00);
    Rec = read_register(0x09,tmp);
    switch(tmp)
    {
        case 4: PowerMode = VL53L5CX_POWER_MODE_WAKEUP;
                break;
        case 2: PowerMode = VL53L5CX_POWER_MODE_SLEEP;
                break;
        default:PowerMode = VL53L5CX_POWER_MODE_SLEEP;
                break;
    }
    Rec = write_register(0x7FFF, 0x02);
    return Rec;
}
//vl53l5cx_set_power_mode
bool AP_RangeFinder_VL53L5CX::SetPowerMode(uint8_t PowerMode)
{
    bool Rec = false;

    uint8_t CurPowerMode = VL53L5CX_POWER_MODE_SLEEP;

    Rec = GetPowerMode(CurPowerMode);

    if(PowerMode != CurPowerMode)
    {
        switch(PowerMode)
        {
            case VL53L5CX_POWER_MODE_WAKEUP:
                    Rec = write_register(0x7FFF, 0x00);
                    Rec = write_register(0x09, 0x04);
                    Rec = PollForAnser(1,0,0x06,0x01,0x01);
                    break;
            case VL53L5CX_POWER_MODE_SLEEP:
                    Rec = write_register(0x7FFF, 0x00);
                    Rec = write_register(0x09, 0x02);
                    Rec = PollForAnser(1,0,0x06,0x01,0x00);
                    break;
            default:Rec = false;break;
        }
        Rec = write_register(0x7FFF, 0x02);
    }
    return Rec;
}
//vl53l5cx_start_ranging(VL53L5CX_Configuration     *p_dev)
bool AP_RangeFinder_VL53L5CX::StartRanging(void)
{
    uint8_t Resolution = 0;
    uint32_t i = 0;
    uint32_t HeaderConfig[2] = {0,0};
    
    BLOCK_HEADER *bh_ptr;
    uint8_t cmd[4] = {0x00,0x03,0x00,0x00};
    bool Rec = false;

    Rec = GetResolution(Resolution);
    if (_print_enable_text) {printf("\r\nStart Ranging Resolution :%d\r\n",Resolution);}
    if (_print_enable_gcs) {gcs().send_text(MAV_SEVERITY_INFO, "Start Ranging Resolution :%d",Resolution);}
    data_read_size = 0;
    streamcount = 255;
    /* Enable mandatory output (meta and common data) */
    uint32_t output_bh_enable[4] = {0x00000007U,0x00000000U,0x00000000U,0xC0000000U};
    /* Send addresses of possible output */
    uint32_t output[12] ={
        VL53L5CX_START_BH,
        VL53L5CX_METADATA_BH,
        VL53L5CX_COMMONDATA_BH,
        VL53L5CX_AMBIENT_RATE_BH,
        VL53L5CX_SPAD_COUNT_BH,
        VL53L5CX_NB_TARGET_DETECTED_BH,
        VL53L5CX_SIGNAL_RATE_BH,
        VL53L5CX_RANGE_SIGMA_MM_BH,
        VL53L5CX_DISTANCE_BH,
        VL53L5CX_REFLECTANCE_BH,
        VL53L5CX_TARGET_STATUS_BH,
        VL53L5CX_MOTION_DETECT_BH
        };

    /* Enable selected outputs in the 'platform.h' file */
#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
    output_bh_enable[0] += (uint32_t)8;
#endif
#ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
    output_bh_enable[0] += (uint32_t)16;
#endif
#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
    output_bh_enable[0] += (uint32_t)32;
#endif
#ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
    output_bh_enable[0] += (uint32_t)64;
#endif
#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
    output_bh_enable[0] += (uint32_t)128;
#endif
#ifndef VL53L5CX_DISABLE_DISTANCE_MM
    output_bh_enable[0] += (uint32_t)256;
#endif
#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
    output_bh_enable[0] += (uint32_t)512;
#endif
#ifndef VL53L5CX_DISABLE_TARGET_STATUS
    output_bh_enable[0] += (uint32_t)1024;
#endif
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
    output_bh_enable[0] += (uint32_t)2048;
#endif
    //if (_print_enab_textle) {printf("\r\noutput_bh_enable[0] : %d\r\n",output_bh_enable[0]);}
/* Update data size */
    for (i = 0; i < (uint32_t)(sizeof(output)/sizeof(uint32_t)); i++)
    {
        if ((output[i] == (uint8_t)0) || ((output_bh_enable[i/(uint32_t)32]&((uint32_t)1 << (i%(uint32_t)32))) == (uint32_t)0))
        {

            continue;
        }

        bh_ptr = (union Block_header *)&(output[i]);
        if (((uint8_t)bh_ptr->ST.type >= (uint8_t)0x1) && ((uint8_t)bh_ptr->ST.type < (uint8_t)0x0d))
        {
            if ((bh_ptr->ST.idx >= (uint16_t)0x54d0)   && (bh_ptr->ST.idx < (uint16_t)(0x54d0 + 960)))
            {
                bh_ptr->ST.size = Resolution;
            }
            else
            {
                bh_ptr->ST.size = (uint16_t)((uint16_t)Resolution * (uint16_t)VL53L5CX_NB_TARGET_PER_ZONE);
            }
            data_read_size += bh_ptr->ST.type * bh_ptr->ST.size;
        }
        else
        {
            data_read_size += bh_ptr->ST.size;
        }
        data_read_size += (uint32_t)4;
    }
    data_read_size += (uint32_t)24;
    if (_print_enable_text) {printf("\r\ndata_read_size:%d\r\n",data_read_size);}
    Rec = DCI_WriteData((uint8_t*)&(output),VL53L5CX_DCI_OUTPUT_LIST,(uint16_t)sizeof(output));
    HeaderConfig[0] = data_read_size;
    HeaderConfig[1] = i + 1;
    Rec = DCI_WriteData((uint8_t*)&(HeaderConfig),VL53L5CX_DCI_OUTPUT_CONFIG,(uint16_t)sizeof(HeaderConfig));
    Rec = DCI_WriteData((uint8_t*)&(output_bh_enable),VL53L5CX_DCI_OUTPUT_ENABLES,(uint16_t)sizeof(output_bh_enable));
    /* Start xshut bypass (interrupt mode) */
    Rec = write_register(0x7fff, 0x00);
    Rec = write_register(0x09, 0x05);
    Rec = write_register(0x7fff, 0x02);
    /* Start ranging session */
    WriteData(VL53L5CX_UI_CMD_END -(uint16_t)(4 - 1),(uint8_t*)cmd, sizeof(cmd));
    Rec = PollForAnser(4,1,VL53L5CX_UI_CMD_STATUS,0xFF,0x03);
    if(!Rec) {
        if (_print_enable_text) {printf("\r\nStart Ranging Session Fail\r\n");}
    }


    /* Read ui range data content and compare if data size is the correct one */
    DCI_ReadData(temp_buffer,0x5440,12);
    uint16_t Tmp = 0;
    (void)memcpy(&Tmp,&temp_buffer[0x08],sizeof(Tmp));
    if (_print_enable_text) {printf("\r\nDCI Read Size: Tmp:%d,DataLen:%d\r\n",Tmp,data_read_size);}
    if(Tmp != data_read_size)
    {
        if (_print_enable_text) {printf("\r\nDCI Read Fail: Tmp:%d,DataLen:%d\r\n",Tmp,data_read_size);}
        Rec = false;
    }
    else
        Rec = true; 

    return Rec;
}
//vl53l5cx_stop_ranging(VL53L5CX_Configuration      *p_dev)
bool AP_RangeFinder_VL53L5CX::StopRanging(void)
{
    uint8_t tmp = 0;
    uint16_t timeout = 0;
    uint32_t auto_stop_flag = 0;
    bool Rec = false;

    ReadData(0x2FFC,(uint8_t *)&auto_stop_flag,4);
    if(auto_stop_flag != 0x4FF)
    {
        Rec = write_register(0x7FFF, 0X00);

        /* Provoke MCU stop */
        Rec = write_register(0x15, 0X16);
        Rec = write_register(0x14, 0X01);
        /* Poll for G02 status 0 MCU stop */
        while(((tmp & (uint8_t)0x80) >> 7) == (uint8_t)0x00)
        {
            Rec = read_register(0x6, tmp);
            hal.scheduler->delay(10);
            /* Timeout reached after 5 seconds */
            timeout++;
            if(timeout > (uint16_t)500)
            {
                if (_print_enable_text) {printf("\r\nMCU stop Time Over\r\n");}
                break;
            }
        }
    }
    /* Check GO2 status 1 if status is still OK */
    Rec = read_register(0x6, tmp);
    if((tmp & (uint8_t)0x80) != (uint8_t)0)
    {
        Rec = read_register(0x07, tmp);
        if((tmp != (uint8_t)0x84) && (tmp != (uint8_t)0x85))
        {
           if (_print_enable_text) {printf("\r\nRead 07 Code:%d\r\n",tmp);}
        }
    }
    /* Undo MCU stop */
    Rec = write_register(0x7FFF, 0X00);
    Rec = write_register(0x14, 0X00);
    Rec = write_register(0x15, 0X00);

    /* Stop xshut bypass */
    Rec = write_register(0x09, 0X04);
    Rec = write_register(0x7FFF, 0X02);

    return Rec;
}
//vl53l5cx_check_data_ready(VL53L5CX_Configuration      *p_dev,uint8_t              *p_isReady)
bool AP_RangeFinder_VL53L5CX::Check_DataReady(uint8_t &isReady)
{
    bool Rec = false;
    ReadData(0,temp_buffer,4);
    if((temp_buffer[0] != streamcount) && (temp_buffer[0] != (uint8_t)255)  && (temp_buffer[1] == (uint8_t)0x5) && ((temp_buffer[2] & (uint8_t)0x5) == (uint8_t)0x5)
            && ((temp_buffer[3] & (uint8_t)0x10) ==(uint8_t)0x10))
    {       
        isReady = 1;
        streamcount = temp_buffer[0];
        if (_print_enable_text) {printf("\r\n Read Data is Ready DataSize:%d \r\n",streamcount);}
        Rec = true;
    }
    else            /* Return GO2 error status */
    {
        if((temp_buffer[3] & 0x80) != 0)
        {
            if (_print_enable_text) {printf("\r\nError Code :%d \r\n",temp_buffer[2]);   }
            Rec = false;
        }
    }
    return Rec;
}
//vl53l5cx_get_ranging_data(VL53L5CX_Configuration      *p_dev, VL53L5CX_ResultsData        *p_results)
bool AP_RangeFinder_VL53L5CX::Get_RangingData(VL53L5CX_ResultsData       *p_results)
{
    BLOCK_HEADER *bh_ptr = &BlockHeade;
    uint16_t header_id, footer_id;
    uint32_t i, j, msize;
    bool Rec = false;

    ReadData(0,temp_buffer,data_read_size);
    streamcount = temp_buffer[0];
    if (_print_enable_text) {printf("\r\ndata_read_size:%d , streamcount:%d\r\n",data_read_size,streamcount);}
    SwapBuffer(temp_buffer, data_read_size);
    /* Start conversion at position 16 to avoid headers */
    for (i = 16U; i < (uint32_t)data_read_size; i+=4U)
    {
        memcpy(bh_ptr,&temp_buffer[i],sizeof(BLOCK_HEADER));
        if ((bh_ptr->ST.type > 0x1U) && (bh_ptr->ST.type < 0xdU))
        {
            msize = bh_ptr->ST.type * bh_ptr->ST.size;
        }
        else
        {
            msize = bh_ptr->ST.size;
        }
        switch(bh_ptr->ST.idx)
        {
            case VL53L5CX_METADATA_IDX:
                p_results->silicon_temp_degc = temp_buffer[12+i];//(int8_t)p_dev->temp_buffer[i + (uint32_t)12];
                break;

#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
            case VL53L5CX_AMBIENT_RATE_IDX:
                (void)memcpy(p_results->ambient_per_spad,&temp_buffer[4+i], msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
            case VL53L5CX_SPAD_COUNT_IDX:
                (void)memcpy(p_results->nb_spads_enabled,&temp_buffer[4+i], msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
            case VL53L5CX_NB_TARGET_DETECTED_IDX:
                (void)memcpy(p_results->nb_target_detected, &temp_buffer[4+i], msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
            case VL53L5CX_SIGNAL_RATE_IDX:
                (void)memcpy(p_results->signal_per_spad,&temp_buffer[4+i], msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
            case VL53L5CX_RANGE_SIGMA_MM_IDX:
                (void)memcpy(p_results->range_sigma_mm,&temp_buffer[4+i], msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_DISTANCE_MM
            case VL53L5CX_DISTANCE_IDX:
                (void)memcpy(p_results->distance_mm,&temp_buffer[4+i], msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
            case VL53L5CX_REFLECTANCE_EST_PC_IDX:
                (void)memcpy(p_results->reflectance,&temp_buffer[4+i], msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_TARGET_STATUS
            case VL53L5CX_TARGET_STATUS_IDX:
                (void)memcpy(p_results->target_status,&temp_buffer[4+i], msize);
                break;
#endif
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
            case VL53L5CX_MOTION_DETEC_IDX:
                (void)memcpy(&p_results->motion_indicator,&temp_buffer[4+i], msize);
                break;
#endif
            default:
                break;
        }
        i += msize;
    }

#ifndef VL53L5CX_USE_RAW_FORMAT

    /* Convert data into their real format */
#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
    for(i = 0; i < (uint32_t)VL53L5CX_RESOLUTION_8X8; i++)
    {
        p_results->ambient_per_spad[i] /= (uint32_t)2048;
    }
#endif

    for(i = 0; i < (uint32_t)(VL53L5CX_RESOLUTION_8X8 * VL53L5CX_NB_TARGET_PER_ZONE); i++)
    {
#ifndef VL53L5CX_DISABLE_DISTANCE_MM
        p_results->distance_mm[i] /= 4;
        if(p_results->distance_mm[i] < 0)
        {
            p_results->distance_mm[i] = 0;
        }
#endif
#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
        p_results->reflectance[i] /= (uint8_t)2;
#endif
#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
        p_results->range_sigma_mm[i] /= (uint16_t)128;
#endif
#ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
        p_results->signal_per_spad[i] /= (uint32_t)2048;
#endif
    }
    /* Set target status to 255 if no target is detected for this zone */
#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
    for(i = 0; i < (uint32_t)VL53L5CX_RESOLUTION_8X8; i++)
    {
        if(p_results->nb_target_detected[i] == (uint8_t)0)
        {
            for(j = 0; j < (uint32_t)VL53L5CX_NB_TARGET_PER_ZONE; j++)
            {
#ifndef VL53L5CX_DISABLE_TARGET_STATUS
                p_results->target_status[((uint32_t)VL53L5CX_NB_TARGET_PER_ZONE*(uint32_t)i) + j]=(uint8_t)255;
#endif
            }
        }
    }
#endif

#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
    for(i = 0; i < (uint32_t)32; i++)
    {
        p_results->motion_indicator.motion[i] /= (uint32_t)65535;
    }
#endif

#endif
    /* Check if footer id and header id are matching. This allows to detect
     * corrupted frames */
    header_id = ((uint16_t)(temp_buffer[0x8])<<8) & 0xFF00U;
    header_id |= ((uint16_t)(temp_buffer[0x9])) & 0x00FFU;

    footer_id = ((uint16_t)(temp_buffer[data_read_size - (uint32_t)4]) << 8) & 0xFF00U;
    footer_id |= ((uint16_t)(temp_buffer[data_read_size - (uint32_t)3])) & 0xFFU;

    if(header_id != footer_id)
    {
        if (_print_enable_text) {printf("\r\nHead ID is Error HeadID:%d ; FooterID:%d\r\n",header_id,footer_id);}
        Rec = false;
    }   
    else
        Rec = true;

    return Rec;
}
//vl53l5cx_get_resolution(VL53L5CX_Configuration        *p_dev,uint8_t              *p_resolution)
bool AP_RangeFinder_VL53L5CX::GetResolution(uint8_t &Resolution)
{
    bool Rec = false;
    Rec = DCI_ReadData(temp_buffer,VL53L5CX_DCI_ZONE_CONFIG,8);
    Resolution = temp_buffer[0] * temp_buffer[1];

    return Rec;
}
//vl53l5cx_set_resolution(VL53L5CX_Configuration         *p_dev,uint8_t             resolution)
bool AP_RangeFinder_VL53L5CX::SetResolution(uint8_t Resolution)
{
    bool Rec = false;

    switch(Resolution)
    {
        case VL53L5CX_RESOLUTION_4X4:
            Rec = DCI_ReadData(temp_buffer,VL53L5CX_DCI_DSS_CONFIG, 16);
            temp_buffer[0x04] = 64;
            temp_buffer[0x06] = 64;
            temp_buffer[0x09] = 4;
            Rec = DCI_WriteData(temp_buffer,VL53L5CX_DCI_DSS_CONFIG, 16);
            Rec = DCI_ReadData(temp_buffer,VL53L5CX_DCI_ZONE_CONFIG, 8);
            temp_buffer[0x00] = 4;
            temp_buffer[0x01] = 4;
            temp_buffer[0x04] = 8;
            temp_buffer[0x05] = 8;
            Rec = DCI_WriteData(temp_buffer,VL53L5CX_DCI_ZONE_CONFIG, 8);
            break;

        case VL53L5CX_RESOLUTION_8X8:
            Rec = DCI_ReadData(temp_buffer,VL53L5CX_DCI_DSS_CONFIG, 16);
            temp_buffer[0x04] = 16;
            temp_buffer[0x06] = 16;
            temp_buffer[0x09] = 1;
            Rec = DCI_WriteData(temp_buffer,VL53L5CX_DCI_DSS_CONFIG, 16);
            Rec = DCI_ReadData(temp_buffer,VL53L5CX_DCI_ZONE_CONFIG, 8);
            temp_buffer[0x00] = 8;
            temp_buffer[0x01] = 8;
            temp_buffer[0x04] = 4;
            temp_buffer[0x05] = 4;
            Rec = DCI_WriteData(temp_buffer,VL53L5CX_DCI_ZONE_CONFIG, 8);       
            break;

        default:
            if (_print_enable_text) {printf("\r\nSet Resolution Fail\r\n");}
            Rec = false;
            break;
        }
    Rec = SendOffsetData(Resolution);
    Rec = SendXTalkData(Resolution);
    return Rec; 
}
//vl53l5cx_get_ranging_frequency_hz
bool AP_RangeFinder_VL53L5CX::Get_RangingFreqHz(uint8_t &FreqHz)
{
    bool Rec = false;

    Rec = DCI_ReadData(temp_buffer,VL53L5CX_DCI_FREQ_HZ,4);
    FreqHz = temp_buffer[1];

    return Rec;
}
//vl53l5cx_set_ranging_frequency_hz(VL53L5CX_Configuration      *p_dev,uint8_t              frequency_hz)
bool AP_RangeFinder_VL53L5CX::Set_RangingFreqHz(uint8_t FreqHz)
{
    bool Rec = false;

    Rec = DCI_ReplaceData(temp_buffer,VL53L5CX_DCI_FREQ_HZ,4,(uint8_t*)&FreqHz,1,0x01);
    return Rec;
}
//vl53l5cx_get_integration_time_ms(VL53L5CX_Configuration       *p_dev,uint32_t         *p_time_ms)
bool AP_RangeFinder_VL53L5CX::Get_IntegrationTime_ms(uint32_t &Time_ms)
{
    bool Rec = false;

    Rec = DCI_ReadData(temp_buffer,VL53L5CX_DCI_INT_TIME,20);
    (void)memcpy(&Time_ms,temp_buffer, 4);
    Time_ms /= (uint32_t)1000;

    return Rec;
}
//vl53l5cx_set_integration_time_ms(VL53L5CX_Configuration       *p_dev,uint32_t         integration_time_ms)
bool AP_RangeFinder_VL53L5CX::Set_IntegrationTime_ms(uint32_t Time_ms)
{
    bool Rec = false;
    uint32_t integration = Time_ms;

    /* Integration time must be between 2ms and 1000ms */
    if((integration < (uint32_t)2)  || (integration > (uint32_t)1000))
    {
        if (_print_enable_text) {printf("\r\nTime Param is invalid\r\n");}
    }
    else
    {
        integration *= (uint32_t)1000;
        Rec = DCI_ReplaceData(temp_buffer,VL53L5CX_DCI_INT_TIME, 20,(uint8_t*)&integration, 4, 0x00);

    }   

    return Rec;
}
//vl53l5cx_get_sharpener_percent(VL53L5CX_Configuration     *p_dev,uint8_t              *p_sharpener_percent)
bool AP_RangeFinder_VL53L5CX::Get_SharpenerPercent(uint8_t &SharpenerPercent)
{
    bool Rec = false;

    Rec = DCI_ReadData(temp_buffer,VL53L5CX_DCI_SHARPENER,16);
    SharpenerPercent = (temp_buffer[0x0D]*100)/255;

    return Rec;
}

//vl53l5cx_set_sharpener_percent(VL53L5CX_Configuration     *p_dev,uint8_t              sharpener_percent)
bool AP_RangeFinder_VL53L5CX::Set_SharpenerPercent(uint8_t SharpenerPercent)
{
    bool Rec = false;

    uint8_t sharpener;

    if (SharpenerPercent >= 100) {
        if (_print_enable_text) {printf("\r\nSharpenerPercent Param Invalid\r\n");}
    }
    else
    {
        sharpener = (SharpenerPercent*(uint8_t)255)/(uint8_t)100;
        Rec = DCI_ReplaceData(temp_buffer,VL53L5CX_DCI_SHARPENER, 16,(uint8_t*)&sharpener, 1, 0x0D);
    }

    return Rec; 
}
//vl53l5cx_get_target_order(VL53L5CX_Configuration  *p_dev, uint8_t *p_target_order)
bool AP_RangeFinder_VL53L5CX::Get_TargetOrder(uint8_t &target_order)
{
    bool Rec = false;

    Rec = DCI_ReadData(temp_buffer,VL53L5CX_DCI_TARGET_ORDER,4);
    target_order = temp_buffer[0];

    return Rec; 
}
//vl53l5cx_set_target_order(VL53L5CX_Configuration      *p_dev,uint8_t              target_order)
bool AP_RangeFinder_VL53L5CX::Set_TargetOrder(uint8_t target_order)
{
    bool Rec = false;

    if((target_order == VL53L5CX_TARGET_ORDER_CLOSEST)  || (target_order == VL53L5CX_TARGET_ORDER_STRONGEST))
    {
        Rec = DCI_ReplaceData(temp_buffer,VL53L5CX_DCI_TARGET_ORDER, 4,(uint8_t*)&target_order, 1, 0x00);
    }
    else
        if (_print_enable_text) {printf("\r\nTargetOrder Param Invalid\r\n");}
    
    return Rec; 
}
//vl53l5cx_get_ranging_mode(VL53L5CX_Configuration      *p_dev,uint8_t              *p_ranging_mode)
bool AP_RangeFinder_VL53L5CX::Get_RangingMode(uint8_t &ranging_mode)
{
    bool Rec = false;


    Rec = DCI_ReadData(temp_buffer,VL53L5CX_DCI_RANGING_MODE,8);

    if(temp_buffer[1] == 1)
        ranging_mode = VL53L5CX_RANGING_MODE_CONTINUOUS;
    else
        ranging_mode = VL53L5CX_RANGING_MODE_AUTONOMOUS;

    return Rec; 
}

//vl53l5cx_set_ranging_mode(VL53L5CX_Configuration      *p_dev,uint8_t              ranging_mode)
bool AP_RangeFinder_VL53L5CX::Set_RangingMode(uint8_t ranging_mode)
{
    bool Rec = false;
    uint32_t single_range = 0x00;

    Rec = DCI_ReadData(temp_buffer,VL53L5CX_DCI_RANGING_MODE,8);

    switch(ranging_mode)
    {
        case VL53L5CX_RANGING_MODE_CONTINUOUS:
            temp_buffer[0x01] = 0x1;
            temp_buffer[0x03] = 0x3;
            single_range = 0x00;
            break;

        case VL53L5CX_RANGING_MODE_AUTONOMOUS:
            temp_buffer[0x01] = 0x3;
            temp_buffer[0x03] = 0x2;
            single_range = 0x01;
            break;

        default:
            if (_print_enable_text) {printf("\r\ranging_mode Param Invalid\r\n");}
            break;
    }
    Rec = DCI_WriteData(temp_buffer,VL53L5CX_DCI_RANGING_MODE,(uint16_t)8);
    Rec = DCI_WriteData((uint8_t*)&single_range,VL53L5CX_DCI_SINGLE_RANGE,(uint16_t)sizeof(single_range));

    return Rec; 
}

//vl53l5cx_enable_internal_cp(VL53L5CX_Configuration *p_dev)
bool AP_RangeFinder_VL53L5CX::Enable_InternalCP(void)
{
    bool Rec = false;
    uint8_t vcsel_bootup_fsm = 1;
    uint8_t analog_dynamic_pad_0 = 0;

    Rec = DCI_ReplaceData(temp_buffer,VL53L5CX_DCI_INTERNAL_CP,16,(uint8_t*)&vcsel_bootup_fsm,1,0x0A);
    Rec = DCI_ReplaceData(temp_buffer,VL53L5CX_DCI_INTERNAL_CP,16,(uint8_t*)&analog_dynamic_pad_0,1,0x0E);

    return Rec; 
}
//vl53l5cx_disable_internal_cp(VL53L5CX_Configuration *p_dev)
bool AP_RangeFinder_VL53L5CX::Disable_InternalCP(void)
{
    bool Rec = false;

    uint8_t vcsel_bootup_fsm = 0;
    uint8_t analog_dynamic_pad_0 = 1;

    Rec = DCI_ReplaceData(temp_buffer,VL53L5CX_DCI_INTERNAL_CP,16,(uint8_t*)&vcsel_bootup_fsm,1,0x0A);
    Rec = DCI_ReplaceData(temp_buffer,VL53L5CX_DCI_INTERNAL_CP,16,(uint8_t*)&analog_dynamic_pad_0,1,0x0E);
    return Rec; 
}
//vl53l5cx_dci_read_data
bool AP_RangeFinder_VL53L5CX::DCI_ReadData(uint8_t *pData,uint32_t Index,uint16_t data_size)
{
    uint16_t i = 0;
    uint32_t RdLen = (uint32_t) data_size + (uint32_t)12;
    uint8_t cmd[12] = {0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x0f,0x00, 0x02, 0x00, 0x08};
    bool Rec = false;

    if(RdLen > VL53L5CX_TEMPORARY_BUFFER_SIZE)
    {
        if (_print_enable_text) {printf("\r\nDataLen :%d,MaxLen:%d\r\n",data_size+12,VL53L5CX_TEMPORARY_BUFFER_SIZE);}
        Rec = false;
    }
    else
    {
        cmd[0] = (uint8_t)(Index >> 8); 
        cmd[1] = (uint8_t)(Index & (uint32_t)0xff);         
        cmd[2] = (uint8_t)((data_size & (uint16_t)0xff0) >> 4);
        cmd[3] = (uint8_t)((data_size & (uint16_t)0xf) << 4);
        /* Request data reading from FW */
        WriteData(VL53L5CX_UI_CMD_END-(uint16_t)11,cmd,sizeof(cmd));
        Rec = PollForAnser(4,1,VL53L5CX_UI_CMD_STATUS,0xFF,0x03);
        /* Read new data sent (4 bytes header + data_size + 8 bytes footer) */
        ReadData(VL53L5CX_UI_CMD_START,temp_buffer, RdLen);
        SwapBuffer(temp_buffer, data_size + (uint16_t)12);
        /* Copy data from FW into input stSwapBufferructure (-4 bytes to remove header) */
        for(i = 0 ; i < (int16_t)data_size;i++)
        {
            pData[i] = temp_buffer[i + 4];
        }
    }
    return Rec;
}
//vl53l5cx_dci_write_data
bool AP_RangeFinder_VL53L5CX::DCI_WriteData(uint8_t *pData,uint32_t Index,uint16_t Size)
{
    uint8_t headers[] = {0x00, 0x00, 0x00, 0x00};
    uint8_t footer[] = {0x00, 0x00, 0x00, 0x0f, 0x05, 0x01, (uint8_t)((Size + 8) >> 8),(uint8_t)((Size + 8) & 0xFF)};

    uint16_t address = VL53L5CX_UI_CMD_END - (Size + 12) + 1;
    int16_t i;
    bool Rec = false;
    
    /* Check if cmd buffer is large enough */
    if((Size + 12) > VL53L5CX_TEMPORARY_BUFFER_SIZE)
    {
        if (_print_enable_text) {printf("\r\nDCI Data Len Over:%d\r\n",Size + 12);}
        Rec = false;
    }
    else
    {
        headers[0] = (uint8_t)(Index >> 8);
        headers[1] = (uint8_t)(Index & (uint32_t)0xff);
        headers[2] = (uint8_t)(((Size & (uint16_t)0xff0) >> 4));
        headers[3] = (uint8_t)((Size & (uint16_t)0xf) << 4);
        
    /* Copy data from structure to FW format (+4 bytes to add header) */
        SwapBuffer(pData, Size);
        for(i = Size - 1 ; i >= 0; i--)
        {
            temp_buffer[i + 4] = pData[i];
        }

    /* Add headers and footer */
        (void)memcpy(&temp_buffer[0], headers, sizeof(headers));
        (void)memcpy(&temp_buffer[Size + 4],footer, sizeof(footer));
    /* Send data to FW */
        WriteData(address,temp_buffer,(uint32_t)((uint32_t)Size + (uint32_t)12));
        Rec = PollForAnser(4,1,VL53L5CX_UI_CMD_STATUS,0xFF,0x03);

        SwapBuffer(pData, Size);
    }
    return Rec;
}
//vl53l5cx_dci_replace_data
bool AP_RangeFinder_VL53L5CX::DCI_ReplaceData(uint8_t *pData,uint32_t Index,uint16_t DataLen,uint8_t *pNewData,uint32_t NewDataLen,uint16_t NewDataPos)
{
    bool Rec = false;
    Rec = DCI_ReadData(pData,Index,DataLen);
    (void)memcpy(pData+NewDataPos, pNewData, NewDataLen);
    Rec = DCI_WriteData(pData,Index,DataLen);
    return Rec;
}


//

bool AP_RangeFinder_VL53L5CX::ReadID(uint32_t &pID)
{
    uint8_t device_id = 0;
    uint8_t revision_id = 0;    
    bool Rec = false;
    Rec = write_register(0x7FFF,0);
    Rec = read_register(0x00,device_id);
    Rec = read_register(0x01,revision_id);
    Rec = write_register(0x7FFF,2);

    if(Rec)
        pID = device_id<<8 | revision_id;
    return Rec;  
}


bool AP_RangeFinder_VL53L5CX::Get_Capabilities(VL53L5CX_Capabilities_t *pCap)
{
    bool Rec = false;
    if(pCap == nullptr)
        Rec = false;
    else
    {
        pCap->NumberOfZones = VL53L5CX_RESOLUTION_8X8;
        pCap->MaxNumberOfTargetsPerZone = VL53L5CX_TARGET_PER_ZONE;
        pCap->CustomROI = 0;
        pCap->ThresholdDetection = 1;   
        Rec = true; 
    }
    return Rec;
}
//VL53L5CX_ConfigProfile
bool AP_RangeFinder_VL53L5CX::Set_ConfigProfile(VL53L5CX_Object_t *pObject,VL53L5CX_ProfileConfig_t *pConfig)
{
    bool Rec = false;
    uint8_t profile;
    uint8_t resolution;
    uint8_t ranging_mode;
    uint8_t ranging_frequency;
    uint32_t integration_time;
    if(pConfig == nullptr)
    {
        Rec = false;
        if (_print_enable_text) {printf("\r\nConfigProfile Param invalid\r\n");}
        return Rec;
    }       
    else
    {
        profile = pConfig->RangingProfile;
        integration_time = pConfig->TimingBudget;
        ranging_frequency = (uint8_t)pConfig->Frequency;
    }
    //if (_print_enab_textle) {printf("\r\nProfile : %d \r\n",profile);}
    switch (profile)
    {
        case VL53L5CX_PROFILE_4x4_CONTINUOUS:
            resolution = VL53L5CX_RESOLUTION_4X4;
            ranging_mode = VL53L5CX_RANGING_MODE_CONTINUOUS;
            break;
        case VL53L5CX_PROFILE_4x4_AUTONOMOUS:
            resolution = VL53L5CX_RESOLUTION_4X4;
            ranging_mode = VL53L5CX_RANGING_MODE_AUTONOMOUS;
            break;
        case VL53L5CX_PROFILE_8x8_CONTINUOUS:
            resolution = VL53L5CX_RESOLUTION_8X8;
            ranging_mode = VL53L5CX_RANGING_MODE_CONTINUOUS;
            break;
        case VL53L5CX_PROFILE_8x8_AUTONOMOUS:
            resolution = VL53L5CX_RESOLUTION_8X8;
            ranging_mode = VL53L5CX_RANGING_MODE_AUTONOMOUS;
            break;
        default:
            resolution = 0;     /* silence MISRA rule 1.3 warning */
            ranging_mode = 0;   /* silence MISRA rule 1.3 warning */
            Rec = false;
            break;
    }
    if(!SetResolution(resolution))
    {
        if (_print_enable_text) {printf("\r\nSetResolution Fail\r\n");}
        Rec = false;    
    }       
    else if(!Set_RangingMode(ranging_mode))
    {
        if (_print_enable_text) {printf("\r\nSet_RangingMode Fail\r\n");}
        Rec = false;
    }       
    else if(!Set_IntegrationTime_ms(integration_time))
    {
        if (_print_enable_text) {printf("\r\nSet_IntegrationTime_ms Fail\r\n");}
        Rec = false;
    }       
    else if(!Set_RangingFreqHz(ranging_frequency))
    {
        if (_print_enable_text) {printf("\r\nSet_RangingFreqHz Fail\r\n");}
        Rec = false;    
    }
    else 
    {
        pObject->IsAmbientEnabled = (pConfig->EnableAmbient == 0U) ? 0U : 1U;
        pObject->IsSignalEnabled = (pConfig->EnableSignal == 0U) ? 0U : 1U;
        Rec = true;
    }   
    return Rec;
}
// VL53L5CX_Start
bool AP_RangeFinder_VL53L5CX::Start_Ranging(VL53L5CX_Object_t *pObj,uint32_t Mode)
{
    bool Rec = false;

    if(StartRanging())
    {
        pObj->IsRanging = 1;
        Rec = true;
        if (_print_enable_text) {printf("\r\n StartRanging Mode :%d\r\n",Mode);}
        switch (Mode)
        {
            case VL53L5CX_MODE_BLOCKING_CONTINUOUS:
                pObj->IsContinuous = 1U;
                pObj->IsBlocking = 1U;
                break;
            case VL53L5CX_MODE_BLOCKING_ONESHOT:
                pObj->IsContinuous = 0U;
                pObj->IsBlocking = 1U;
                break;
            case VL53L5CX_MODE_ASYNC_CONTINUOUS:
                pObj->IsContinuous = 1U;
                pObj->IsBlocking = 0U;
                break;
            case VL53L5CX_MODE_ASYNC_ONESHOT:
                pObj->IsContinuous = 0U;
                pObj->IsBlocking = 0U;
                break;
            default:
                pObj->IsRanging = 0U;
                Rec = false;
                break;
        }
    }
    return Rec;
}
//vl53l5cx_poll_for_measurement
bool AP_RangeFinder_VL53L5CX::Poll_For_Measurement(uint32_t Timeout)
{
  static uint32_t TickStart;
  uint8_t NewDataReady = 0; 
  bool Rec = false;

  TickStart = AP_HAL::millis();

    while(AP_HAL::millis() - TickStart < Timeout)
    {
        Rec = Check_DataReady(NewDataReady);
        if(NewDataReady)
        {
            Rec = true;
            if (_print_enable_text) {printf("\r\n Data is OK\r\n");}
            break;
        }           
        else {
            Rec = false;
        }
        hal.scheduler->delay_microseconds(100);
    }
    return Rec;
}
bool AP_RangeFinder_VL53L5CX::Get_Result(VL53L5CX_Object_t *pObj,RANGING_SENSOR_Result_t *pResult)
{
    uint8_t i, j;
    uint8_t resolution;
    uint8_t target_status;
    static VL53L5CX_ResultsData data;

    bool Rec = false;

    if(!GetResolution(resolution))
    {
        if (_print_enable_text) {printf("\r\nGet Result resolution Fail\r\n");}
        Rec = false;
    }
    else if(!Get_RangingData(&data))
    {
        if (_print_enable_text) {printf("\r\nGet RangingData Fail\r\n");}
        Rec = false;
    }
    else
    {
        pResult->NumberOfZones = resolution;
        for (i = 0; i < resolution; i++)
        {
            pResult->ZoneResult[i].NumberOfTargets = data.nb_target_detected[i];

            for (j = 0; j < data.nb_target_detected[i]; j++)
            {
                pResult->ZoneResult[i].Distance[j] = (uint32_t)data.distance_mm[(VL53L5CX_NB_TARGET_PER_ZONE * i) + j];

                /* return Ambient value if ambient rate output is enabled */
                if (pObj->IsAmbientEnabled == 1U)
                {
                    /* apply ambient value to all targets in a given zone */
                    pResult->ZoneResult[i].Ambient[j] = (float_t)data.ambient_per_spad[i];
                }
                else
                {
                    pResult->ZoneResult[i].Ambient[j] = 0.0f;
                }

                /* return Signal value if signal rate output is enabled */
                if (pObj->IsSignalEnabled == 1U)
                {
                    pResult->ZoneResult[i].Signal[j] = (float_t)data.signal_per_spad[(VL53L5CX_NB_TARGET_PER_ZONE * i) + j];
                }
                else
                {
                    pResult->ZoneResult[i].Signal[j] = 0.0f;
                }
                //if (_print_enab_textle) {printf("\r\ndata:%d %d %d\r\n",i,j,pResult->ZoneResult[i].Distance[j]);}
                target_status = data.target_status[(VL53L5CX_NB_TARGET_PER_ZONE * i) + j];
                pResult->ZoneResult[i].Status[j] = Map_TargetStatus(target_status);
            }       
        }
        Rec = true;
    }
    return Rec;
}
uint8_t AP_RangeFinder_VL53L5CX::Map_TargetStatus(uint8_t status)
{
    uint8_t ret = 0;
  if ((status == 5U) || (status == 9U))
    {
        ret = 0U; /* ranging is OK */
    }
    else if (status == 0U)
    {
        ret = 255U; /* no update */
    }
    else
    {
        ret = status; /* return device status otherwise */
    }
    return ret;
}

bool AP_RangeFinder_VL53L5CX::GetDistance(VL53L5CX_Object_t *pObj,RANGING_SENSOR_Result_t *pResult)
{
    bool Rec = false;

    if(pObj->IsRanging == 0)
    {
        if (_print_enable_text) {printf("\r\nVL53 not working\r\n");}
        Rec = false;
    }
    else
    {
        if(pObj->IsBlocking == 1)
        {
            Rec = Poll_For_Measurement(V53L5CX_POLL_TIMEOUT);
            if (_print_enable_text) {printf("\r\nDelay Measurement :%d\r\n",Rec);}
        }
        else
        {
            Rec = Poll_For_Measurement(0);
            if (_print_enable_text) {printf("\r\nMeasurement :%d\r\n",Rec);}
        }
    }
    if(Rec) {
        Rec = Get_Result(pObj,pResult);
    }

    return Rec;
}





// set distance mode to Short, Medium, or Long
// based on VL53L1_SetDistanceMode()
bool AP_RangeFinder_VL53L5CX::setDistanceMode(DistanceMode distance_mode)
{
    // save existing timing budget
    uint32_t budget_us = 0;
    if (!getMeasurementTimingBudget(budget_us)) {
        return false;
    }

    switch (distance_mode) {
      case DistanceMode::Short:
            // from VL53L1_preset_mode_standard_ranging_short_range()

            if (!(// timing config
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07) &&
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05) &&
                  write_register(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38) &&

                  // dynamic config
                  write_register(SD_CONFIG__WOI_SD0, 0x07) &&
                  write_register(SD_CONFIG__WOI_SD1, 0x05) &&
                  write_register(SD_CONFIG__INITIAL_PHASE_SD0, 6) && // tuning parm default
                  write_register(SD_CONFIG__INITIAL_PHASE_SD1, 6))) { // tuning parm default
                return false;
            }

            break;

        case DistanceMode::Medium:
            // from VL53L1_preset_mode_standard_ranging()

            if (!(// timing config
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B) &&
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09) &&
                  write_register(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78) &&

                  // dynamic config
                  write_register(SD_CONFIG__WOI_SD0, 0x0B) &&
                  write_register(SD_CONFIG__WOI_SD1, 0x09) &&
                  write_register(SD_CONFIG__INITIAL_PHASE_SD0, 10) && // tuning parm default
                  write_register(SD_CONFIG__INITIAL_PHASE_SD1, 10))) { // tuning parm default
                return false;
            }

            break;

        case DistanceMode::Long:
            // from VL53L1_preset_mode_standard_ranging_long_range()

            if (!(// timing config
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F) &&
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D) &&
                  write_register(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8) &&

                  // dynamic config
                  write_register(SD_CONFIG__WOI_SD0, 0x0F) &&
                  write_register(SD_CONFIG__WOI_SD1, 0x0D) &&
                  write_register(SD_CONFIG__INITIAL_PHASE_SD0, 14) && // tuning parm default
                  write_register(SD_CONFIG__INITIAL_PHASE_SD1, 14))) { // tuning parm default
                return false;
            }

            break;

        default:
            // unrecognized mode - do nothing
            return false;
    }

    // reapply timing budget
    return setMeasurementTimingBudget(budget_us);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement. A longer timing budget allows for more accurate measurements.
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool AP_RangeFinder_VL53L5CX::setMeasurementTimingBudget(uint32_t budget_us)
{
    // assumes PresetMode is LOWPOWER_AUTONOMOUS
    if (budget_us <= TimingGuard) {
        return false;
    }

    uint32_t range_config_timeout_us = budget_us - TimingGuard;
    if (range_config_timeout_us > 1100000) {
        return false; // FDA_MAX_TIMING_BUDGET_US * 2
    }

    range_config_timeout_us /= 2;

    // VL53L1_calc_timeout_register_values() begin

    uint8_t range_config_vcsel_period = 0;
    if (!read_register(RANGE_CONFIG__VCSEL_PERIOD_A, range_config_vcsel_period)) {
        return false;
    }

    // "Update Macro Period for Range A VCSEL Period"
    uint32_t macro_period_us = calcMacroPeriod(range_config_vcsel_period);

    // "Update Phase timeout - uses Timing A"
    // Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
    // via VL53L1_get_preset_mode_timing_cfg().
    uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
    if (phasecal_timeout_mclks > 0xFF) {
        phasecal_timeout_mclks = 0xFF;
    }

    if (!( write_register(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks) &&

          // "Update MM Timing A timeout"
          // Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
          // via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
          // actually ends up with a slightly different value because it gets assigned,
          // retrieved, recalculated with a different macro period, and reassigned,
          // but it probably doesn't matter because it seems like the MM ("mode
          // mitigation"?) sequence steps are disabled in low power auto mode anyway.
          write_register16(MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
              timeoutMicrosecondsToMclks(1, macro_period_us))) &&

          // "Update Range Timing A timeout"
          write_register16(RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
              timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us))) &&

          // "Update Macro Period for Range B VCSEL Period"
          read_register(RANGE_CONFIG__VCSEL_PERIOD_B, range_config_vcsel_period)
         )) {
        return false;
    }

    // "Update Macro Period for Range B VCSEL Period"
    macro_period_us = calcMacroPeriod(range_config_vcsel_period);

    // "Update MM Timing B timeout"
    // (See earlier comment about MM Timing A timeout.)
    return write_register16(MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
               timeoutMicrosecondsToMclks(1, macro_period_us))) &&

           // "Update Range Timing B timeout"
           write_register16(RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
               timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));
}

// Get the measurement timing budget in microseconds
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool AP_RangeFinder_VL53L5CX::getMeasurementTimingBudget(uint32_t &budget)
{
    // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
    // enabled: VHV, PHASECAL, DSS1, RANGE

    // "Update Macro Period for Range A VCSEL Period"
    uint8_t range_config_vcsel_period_a = 0;
    if (!read_register(RANGE_CONFIG__VCSEL_PERIOD_A, range_config_vcsel_period_a)) {
        return false;
    }

    uint32_t macro_period_us = calcMacroPeriod(range_config_vcsel_period_a);

    uint16_t timeout_macrop_a = 0;
    if (!read_register16(RANGE_CONFIG__TIMEOUT_MACROP_A, timeout_macrop_a)) {
        return false;
    }

    // "Get Range Timing A timeout"
    uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(timeout_macrop_a), macro_period_us);

    budget = 2 * range_config_timeout_us + TimingGuard;
    return true;
}

// Start continuous ranging measurements, with the given inter-measurement
// period in milliseconds determining how often the sensor takes a measurement.
bool AP_RangeFinder_VL53L5CX::startContinuous(uint32_t period_ms)
{
    // fix for actual measurement period shorter than set
    uint32_t adjusted_period_ms = period_ms + (period_ms * 64 / 1000);

    // from VL53L1_set_inter_measurement_period_ms()
    return write_register32(SYSTEM__INTERMEASUREMENT_PERIOD, adjusted_period_ms * osc_calibrate_val) &&
           write_register(SYSTEM__INTERRUPT_CLEAR, 0x01) && // sys_interrupt_clear_range
           write_register(SYSTEM__MODE_START, 0x40); // mode_range__timed
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L1_decode_timeout()
uint32_t AP_RangeFinder_VL53L5CX::decodeTimeout(uint16_t reg_val)
{
    return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L1_encode_timeout()
uint16_t AP_RangeFinder_VL53L5CX::encodeTimeout(uint32_t timeout_mclks)
{
    // encoded format: "(LSByte * 2^MSByte) + 1"
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0) 
    {
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0) 
        {
            ls_byte >>= 1;
            ms_byte++;
        }
        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    else {
        return 0;
    }
}

// Convert sequence step timeout from macro periods to microseconds with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_us()
uint32_t AP_RangeFinder_VL53L5CX::timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
    return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

// Convert sequence step timeout from microseconds to macro periods with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_mclks()
uint32_t AP_RangeFinder_VL53L5CX::timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
    return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

// Calculate macro period in microseconds (12.12 format) with given VCSEL period
// assumes fast_osc_frequency has been read and stored
// based on VL53L1_calc_macro_period_us()
uint32_t AP_RangeFinder_VL53L5CX::calcMacroPeriod(uint8_t vcsel_period) const
{
    // from VL53L1_calc_pll_period_us()
    // fast osc frequency in 4.12 format; PLL period in 0.24 format
    uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

    // from VL53L1_decode_vcsel_period()
    uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

    // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
    uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks;
    macro_period_us >>= 6;

    return macro_period_us;
}

// "Setup ranges after the first one in low power auto mode by turning off
// FW calibration steps and programming static values"
// based on VL53L1_low_power_auto_setup_manual_calibration()
bool AP_RangeFinder_VL53L5CX::setupManualCalibration(void)
{
    uint8_t saved_vhv_init = 0;
    uint8_t saved_vhv_timeout = 0;
    uint8_t phasecal_result_vcsel_start = 0;

    return // "save original vhv configs"
           read_register(VHV_CONFIG__INIT, saved_vhv_init) &&
           read_register(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout) &&

           // "disable VHV init"
           write_register(VHV_CONFIG__INIT, saved_vhv_init & 0x7F) &&

          // "set loop bound to tuning param"
          write_register(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
                         (saved_vhv_timeout & 0x03) + (3 << 2)) && // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)

          // "override phasecal"
          write_register(PHASECAL_CONFIG__OVERRIDE, 0x01) &&
          read_register(PHASECAL_RESULT__VCSEL_START, phasecal_result_vcsel_start) &&
          write_register(CAL_CONFIG__VCSEL_START, phasecal_result_vcsel_start);
}

// check if sensor has new reading available
// assumes interrupt is active low (GPIO_HV_MUX__CTRL bit 4 is 1)
bool AP_RangeFinder_VL53L5CX::dataReady(void)
{
    uint8_t gpio_tio_hv_status = 0;

    return read_register(GPIO__TIO_HV_STATUS, gpio_tio_hv_status) &&
           ((gpio_tio_hv_status & 0x01) == 0);
}

// // read - return last value measured by sensor
// bool AP_RangeFinder_VL53L5CX::get_reading(uint16_t &reading_mm)
// {

//     if (final_dist_mm == 0) {
//         reading_mm = 0;
//         return false;
//     }

//     reading_mm = final_dist_mm;
//     return true;
// }

bool AP_RangeFinder_VL53L5CX::read_register(uint16_t reg, uint8_t &value)
{
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };  
    return dev->transfer(b, 2, &value, 1);
}

bool AP_RangeFinder_VL53L5CX::read_register16(uint16_t reg, uint16_t & value)
{
    uint16_t v = 0;
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    if (!dev->transfer(b, 2, (uint8_t *)&v, 2)) 
    {
        return false;
    }
    value = be16toh(v);
    return true;
}
bool AP_RangeFinder_VL53L5CX::ReadData(uint16_t reg,uint8_t *pData,uint16_t DataLen)
{
    bool Rec = false;
    uint16_t loop = 0;          
    uint16_t remain = 0;        
    uint16_t i = 0;
    uint16_t BaseAddress = reg;
    uint16_t WriteAddress = reg;
    uint16_t SizeBuf = 128;

    uint8_t ReadAddress[2] = {0};

    uint8_t *pTemp = (uint8_t *)malloc(SizeBuf);

    loop = DataLen / SizeBuf;
    remain = DataLen % SizeBuf;

    if(loop > 0)
    {
        for(i = 0;i < loop;i++)                         
        {
            WriteAddress = i*SizeBuf + BaseAddress;
            ReadAddress[0] = uint8_t(WriteAddress >> 8);
            ReadAddress[1] = uint8_t(WriteAddress & 0xFF);          
            Rec = dev->transfer(ReadAddress, 2, pTemp, SizeBuf);
            (void)memcpy(pData+(i*SizeBuf),pTemp,SizeBuf);
        }       
    }   
    if(remain > 0)
    {
        WriteAddress = i*SizeBuf + BaseAddress;
        ReadAddress[0] = uint8_t(WriteAddress >> 8);
        ReadAddress[1] = uint8_t(WriteAddress & 0xFF);      
        Rec = dev->transfer(ReadAddress, 2, pTemp, remain);
        (void)memcpy(pData+(i*SizeBuf),pTemp,remain);
    }
    free(pTemp);
    return Rec;
}
bool AP_RangeFinder_VL53L5CX::WriteData(uint16_t reg,uint8_t *pData,uint16_t DataLen)
{
    bool Rec = false;
    uint16_t loop = 0;          
    uint16_t remain = 0;        
    uint16_t i = 0;
    uint16_t BaseAddress = reg;
    uint16_t WriteAddress = reg;
    uint16_t SizeBuf = 128;

    uint8_t *pTemp = (uint8_t *)malloc(SizeBuf+2);

    loop = DataLen / SizeBuf;
    remain = DataLen % SizeBuf;

    if(loop > 0 )
    {
        for(i = 0;i < loop;i++)                             
        {
            WriteAddress = i*SizeBuf + BaseAddress;
            *(pTemp) = uint8_t(WriteAddress >> 8);
            *(pTemp+1) = uint8_t(WriteAddress & 0xFF);
            (void)memcpy(pTemp+2,pData+(i*SizeBuf),SizeBuf);
            Rec = dev->transfer(pTemp, SizeBuf+2, nullptr, 0);
        }
    }
    if(remain > 0)                                  
    {
        WriteAddress = i*SizeBuf + BaseAddress;
        *(pTemp) = uint8_t(WriteAddress >> 8);
        *(pTemp+1) = uint8_t(WriteAddress & 0xFF);
        (void)memcpy(pTemp+2,pData+(i*SizeBuf),remain);
        Rec = dev->transfer(pTemp, remain+2, nullptr, 0);
    }
    free(pTemp);
    return Rec;
}
bool AP_RangeFinder_VL53L5CX::write_register(uint16_t reg, uint8_t value)
{
    uint8_t b[3] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), value };
    return dev->transfer(b, 3, nullptr, 0);
}

bool AP_RangeFinder_VL53L5CX::write_register16(uint16_t reg, uint16_t value)
{
    uint8_t b[4] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), uint8_t(value >> 8), uint8_t(value & 0xFF) };
    return dev->transfer(b, 4, nullptr, 0);
}
bool AP_RangeFinder_VL53L5CX::write_register32(uint16_t reg, uint32_t value)
{
    uint8_t b[6] = { uint8_t(reg >> 8),
                     uint8_t(reg & 0xFF),
                     uint8_t((value >> 24) & 0xFF),
                     uint8_t((value >> 16) & 0xFF),
                     uint8_t((value >>  8) & 0xFF),
                     uint8_t((value)       & 0xFF) };
    return dev->transfer(b, 6, nullptr, 0);
}

bool AP_RangeFinder_VL53L5CX::print_result(RANGING_SENSOR_Result_t *pResult)
{
    uint8_t i, j, l;
    int8_t k;
    uint8_t zones_per_line;

    zones_per_line = ((Profile.RangingProfile == VL53L5CX_PROFILE_8x8_AUTONOMOUS) || (Profile.RangingProfile == VL53L5CX_PROFILE_8x8_CONTINUOUS)) ? 8 : 4;


    if (pResult->NumberOfZones == 64 && zones_per_line == 8) {
        if ((pResult->ZoneResult[3*8+3].NumberOfTargets > 0)
            &&(pResult->ZoneResult[3*8+4].NumberOfTargets > 0)
            &&(pResult->ZoneResult[4*8+3].NumberOfTargets > 0)
            &&(pResult->ZoneResult[4*8+4].NumberOfTargets > 0)
            )
        {
            final_dist_mm = 0;
            final_dist_mm = pResult->ZoneResult[3*8+3].Distance[0] + pResult->ZoneResult[3*8+4].Distance[0] + pResult->ZoneResult[4*8+3].Distance[0] + pResult->ZoneResult[4*8+4].Distance[0];
            final_dist_mm = final_dist_mm/4;
            // if (_print_enable_text) {printf("\n pResult->ZoneResult[3*8+3].Distance[0]: %d", (int)pResult->ZoneResult[3*8+3].Distance[0]);}
            // if (_print_enable_text) {printf("\n pResult->ZoneResult[3*8+4].Distance[0]: %d", (int)pResult->ZoneResult[3*8+4].Distance[0]);}
            // if (_print_enable_text) {printf("\n pResult->ZoneResult[4*8+3].Distance[0]: %d", (int)pResult->ZoneResult[4*8+3].Distance[0]);}
            // if (_print_enable_text) {printf("\n pResult->ZoneResult[4*8+4].Distance[0]: %d", (int)pResult->ZoneResult[4*8+4].Distance[0]);}
        } else {
            if (_print_enable_text) {printf("\n pResult->ZoneResult[3*8+3].NumberOfTargets: %d", pResult->ZoneResult[3*8+3].NumberOfTargets);}
            if (_print_enable_text) {printf("\n pResult->ZoneResult[3*8+4].NumberOfTargets: %d", pResult->ZoneResult[3*8+4].NumberOfTargets);}
            if (_print_enable_text) {printf("\n pResult->ZoneResult[4*8+3].NumberOfTargets: %d", pResult->ZoneResult[4*8+3].NumberOfTargets);}
            if (_print_enable_text) {printf("\n pResult->ZoneResult[4*8+4].NumberOfTargets: %d", pResult->ZoneResult[4*8+4].NumberOfTargets);}
        }
    } else {
        if (_print_enable_text) {printf("\n pResult->NumberOfZones: %d", pResult->NumberOfZones);}
        if (_print_enable_text) {printf("\n zones_per_line: %d", zones_per_line);}
    }


    if (pResult->NumberOfZones == 64 && zones_per_line == 8) {
        mavlink_wxbs_tof_distance_t packet;

        for (uint8_t j_t = 0; j_t < 8; j_t++) {
            for (uint8_t k_t = 0; k_t < 8; k_t++) {
                uint8_t i_t = j_t*8 + k_t;
                packet.dist[i_t] = (uint16_t)pResult->ZoneResult[i_t].Distance[0];
            }
        }
        AP::fd_data().send_mav_tof_matrix(&packet);
    }

    if (_print_enable_text) {printf("\r\n");}
    if (_print_enable_text) {printf("Cell Format :\r\n\r\n");}
    for (l = 0; l < RANGING_SENSOR_NB_TARGET_PER_ZONE; l++)
    {
        if (_print_enable_text) {printf(" %20s : %20s\r\n", "Distance [mm]", "Status");}
        if ((Profile.EnableAmbient != 0) || (Profile.EnableSignal != 0))
        {
            if (_print_enable_text) {printf(" %20s : %20s\r\n", "Signal [kcps/spad]", "Ambient [kcps/spad]");}
        }
    }
    if (_print_enable_text) {printf("\r\n NumberOfZones:%d   line:%d\r\n",pResult->NumberOfZones,zones_per_line);}
    if (_print_enable_text) {printf("\r\n");}

    for (j = 0; j < pResult->NumberOfZones; j += zones_per_line)
    {
        for (i = 0; i < zones_per_line; i++) /* number of zones per line */
        {
            // if (_print_enable_text) {printf(" ----------------");}
        }      
        // if (_print_enable_text) {printf("\r\n");}

        for (i = 0; i < zones_per_line; i++)
        {
            // if (_print_enable_text) {printf("|                 ");   }
        }
            
        // if (_print_enable_text) {printf("|\r\n");}

        for (l = 0; l < RANGING_SENSOR_NB_TARGET_PER_ZONE; l++)
        {
            /* Print distance and status */
            for (k = (zones_per_line - 1); k >= 0; k--)
            {
                if (pResult->ZoneResult[j+k].NumberOfTargets > 0)
                {
                    if ((long)pResult->ZoneResult[j+k].Distance[l] < 500)
                    {
                        //if (_print_enab_textle) {printf("| \033[38;5;9m%5ld\033[0m  :  %5ld ",}
                        // if (_print_enable_text) {printf("| %5ld :  %5ld ",(long)pResult->ZoneResult[j+k].Distance[l],(long)pResult->ZoneResult[j+k].Status[l]);}
                    } 
                    else
                    {
                        //if (_print_enab_textle) {printf("| \033[38;5;10m%5ld\033[0m  :  %5ld ",}
                        // if (_print_enable_text) {printf("| %5ld  : %5ld ",(long)pResult->ZoneResult[j+k].Distance[l],(long)pResult->ZoneResult[j+k].Status[l]);}
                    }
                }
                else { 
                    if (_print_enable_text) {printf("| %5s  :  %5s ", "X", "X");}
                }
            }
            // if (_print_enable_text) {printf("|\r\n");}

            if ((Profile.EnableAmbient != 0) || (Profile.EnableSignal != 0))
            {
                /* Print Signal and Ambient */
                for (k = (zones_per_line - 1); k >= 0; k--)
                {
                    if (pResult->ZoneResult[j+k].NumberOfTargets > 0)
                    {
                        if (Profile.EnableSignal != 0){
                            // if (_print_enable_text) {printf("| %5ld  :  ", (long)pResult->ZoneResult[j+k].Signal[l]);}
                        }
                        else{
                            // if (_print_enable_text) {printf("| %5s  :  ", "X");}
                        }

                        if (Profile.EnableAmbient != 0){
                            // if (_print_enable_text) {printf("%5ld ", (long)pResult->ZoneResult[j+k].Ambient[l]);}
                        }
                        else{
                            // if (_print_enable_text) {printf("%5s ", "X");}
                        }
                    }
                    else{
                        // if (_print_enable_text) {printf("| %5s  :  %5s ", "X", "X");}
                    }
                }
                // if (_print_enable_text) {printf("|\r\n");}
            }
        }
    }
    for (i = 0; i < zones_per_line; i++) {
        // if (_print_enable_text) {printf(" -----------------");}
    }
    // if (_print_enable_text) {printf("\r\n");}
    if (_print_enable_text) {printf("\r\nPrintf Result is Over!\r\n");}
    return true;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_VL53L5CX::update(void)
{
    // WITH_SEMAPHORE(_sem);

    // if (counter > 0) 
    // {
    //     if (_print_enable_text) {printf("\r\nHave %d Sensor\r\n",counter);}
    //     state.distance_m = (sum_mm * 0.001f) / counter;
    //     state.last_reading_ms = AP_HAL::millis();
    //     update_status();
    //     sum_mm = 0;
    //     counter = 0;
    // } 
    // else if (AP_HAL::millis() - state.last_reading_ms > 200) 
    // {
    //     // if no updates for 0.2s set no-data
    //     set_status(RangeFinder::Status::NoData);
    // }   

}

/*
  timer called at 20Hz
*/
void AP_RangeFinder_VL53L5CX::timer(void)
{
    if (GetDistance(&Object,&Result) ) {
        print_result(&Result);
        counter++;
    }

    if (counter > 0) {
        WITH_SEMAPHORE(_sem);
        // if (_print_enable_text) {printf("\r\nHave %d Sensor\r\n",counter);}
        state.distance_m = ((float)final_dist_mm * 0.001f);
        state.last_reading_ms = AP_HAL::millis();
        update_status();
        counter = 0;
    } else if (AP_HAL::millis() - state.last_reading_ms > 2000) {
        // if no updates for 0.2s set no-data
        set_status(RangeFinder::Status::NoData);
    }
    if (true) {printf("\n++++++++++++++++\n");}
    if (true) {printf("\nfinal_dist_mm :%f\n", state.distance_m);}

}

void AP_RangeFinder_VL53L5CX::timer_loop()
{
    while (true) {
        {
            WITH_SEMAPHORE(dev->get_semaphore());
            timer();
        }
        hal.scheduler->delay(500);
    }
}

#endif  // AP_RANGEFINDER_VL53L5CX_ENABLED
