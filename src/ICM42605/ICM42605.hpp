#pragma once
#include <iostream>
#include "../SPI/LinuxSPI.hpp"
#include <chrono>
#include <unistd.h>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <time.h>
#include "../filter.h"
#include "../_thirdparty/eigen/Eigen/Dense"
#include "../_thirdparty/eigen/Eigen/LU"

#define MPUTypeI2C 0
#define MPUTypeSPI 1

struct ICM4Data
{
    int DeviceType = 0;
    int _uORB_ICM42605_IMUUpdateTime = 0;
    int _uORB_ICM42605_AccelCountDown = 0;

    int _uORB_ICM42605_A_X = 0;
    int _uORB_ICM42605_A_Y = 0;
    int _uORB_ICM42605_A_Z = 0;
    int _uORB_ICM42605_G_X = 0;
    int _uORB_ICM42605_G_Y = 0;
    int _uORB_ICM42605_G_Z = 0;
};

struct ICM4Config
{
    bool MPUType = MPUTypeSPI;
    const char *MPUSPIChannel = "/dev/spidev0.0";
    int ICM42605_SPI_Freq = 400000;
    int TargetFreqency = 1000;
    int AccTargetFreqency = 1000;
    float MPU_Flip_Pitch = 0;
    float MPU_Flip__Roll = 0;
    float MPU_Flip___Yaw = 270;

};

class RPiICM42605
{
public:
    inline RPiICM42605(ICM4Config mpuConfig)
    {
        struct timespec tv;
        clock_gettime(CLOCK_MONOTONIC, &tv);
        LastUpdate = GetTimestamp();
        IMUstartuptime = (((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000))));
        IMUSensorsDeviceInit();

    }

    // inline ICMData ICMSensorsDataGet()
    // {
    //     IMUSensorsDataRead();

    //     // return PrivateData;
    // }

private:
    inline void IMUSensorsDeviceInit()
    {
        // double OutputSpeedCal = (MPU_250HZ_LPF_SPEED / (float)PrivateConfig.TargetFreqency) - 1.f;
        if (PrivateConfig.MPUType == MPUTypeSPI)
        {
            ICM42605_fd = _s_spiOpen(PrivateConfig.MPUSPIChannel, PrivateConfig.ICM42605_SPI_Freq, 0);
            if (ICM42605_fd < 0)
                throw std::invalid_argument("[SPI] MPU device can't open");
            uint8_t ICM42605_SPI_Config_WHOAMI[2] = {0xf5, 0x00};
            _s_spiXfer(ICM42605_fd, ICM42605_SPI_Config_WHOAMI, ICM42605_SPI_Config_WHOAMI, PrivateConfig.ICM42605_SPI_Freq, 2);
            PrivateData.DeviceType = ICM42605_SPI_Config_WHOAMI[1];
            //---------------------reset--------------------------//
            uint8_t ICM42605_SPI_Config_RESET[2] = {0x76, 0x00};
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_RESET, PrivateConfig.ICM42605_SPI_Freq, 2); // bank0
            usleep(500);
            uint8_t ICM42605_SPI_Config_RESET2[2] = {0x11, 0x01};
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_RESET2, PrivateConfig.ICM42605_SPI_Freq, 2); // soft reset
            usleep(500);

            uint8_t ICM42605_SPI_Config_CONF[2] = {0x76, 0x01}; 
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF, PrivateConfig.ICM42605_SPI_Freq, 2); // bank1
            usleep(500);
            uint8_t ICM42605_SPI_Config_CONF2[2] = {0x11, 0x02};
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF2, PrivateConfig.ICM42605_SPI_Freq, 2); // 4 wire spi mode
            usleep(500);            

            uint8_t ICM42605_SPI_Config_CONF3[2] = {0x76, 0x00}; 
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF3, PrivateConfig.ICM42605_SPI_Freq, 2); // bank0
            usleep(500); 
            uint8_t ICM42605_SPI_Config_CONF4[2] = {0x16, 0x40}; 
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF4, PrivateConfig.ICM42605_SPI_Freq, 2); // Stream-to-FIFO Mode
            usleep(500); 
            uint8_t ICM42605_SPI_Config_CONF5[2] = {0xe5, 0x00}; 
            _s_spiXfer(ICM42605_fd, ICM42605_SPI_Config_CONF5, ICM42605_SPI_Config_RESET7, PrivateConfig.ICM42605_SPI_Freq, 2);
            usleep(500); 
            uint8_t ICM42605_SPI_Config_CONF6[2] = {0x60, 0x00}; 
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF6, PrivateConfig.ICM42605_SPI_Freq, 2); // watermark
            usleep(500); 
            uint8_t ICM42605_SPI_Config_CONF7[2] = {0x61, 0x02}; 
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF7, PrivateConfig.ICM42605_SPI_Freq, 2); // watermark
            usleep(500); 
            uint8_t ICM42605_SPI_Config_CONF8[2] = {0x65, ICM42605_SPI_Config_CONF5[1]}; 
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF8, PrivateConfig.ICM42605_SPI_Freq, 2); 
            usleep(500);     
            uint8_t ICM42605_SPI_Config_CONF9[2] = {0x65, }; 
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF9, PrivateConfig.ICM42605_SPI_Freq, 2); 
            usleep(500);    
            // uint8_t ICM42605_SPI_Config_RESET3[2] = {0x6b, 0x00};
            // _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_RESET3, PrivateConfig.ICM42605_SPI_Freq, 2); // reset
            // usleep(500);
            // uint8_t ICM42605_SPI_Config_RESET4[2] = {0x6b, 0x01};
            // _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_RESET4, PrivateConfig.ICM42605_SPI_Freq, 2); // reset
            // usleep(1000);
            // uint8_t ICM42605_SPI_Config_RESET5[2] = {0X6C, 0x00};
            // _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_RESET4, PrivateConfig.ICM42605_SPI_Freq, 2); //  acc | gyro on
            // usleep(1000);


            // uint8_t ICM42605_SPI_Config_ALPF[2] = {0x1d, 0x00};                                   // FChoice 1, DLPF 3 , dlpf cut off 44.8hz for accel is 0x03, but now 0x00 is not apply accel hardware
            // _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_ALPF, PrivateConfig.ICM42605_SPI_Freq, 2); // Accel2
            // usleep(15);
            // uint8_t ICM42605_SPI_Config_Acce[2] = {0x1c, 0x18};                                   // Full AccelScale +- 16g
            // _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_Acce, PrivateConfig.ICM42605_SPI_Freq, 2); // Accel
            // usleep(15);
            // uint8_t ICM42605_SPI_Config_Gyro[2] = {0x1b, 0x18};                                   // Full GyroScale +-2000dps, dlpf 250hz
            // _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_Gyro, PrivateConfig.ICM42605_SPI_Freq, 2); // Gryo
            // usleep(15);


            // uint8_t ICM42605_SPI_Config_GLPF[2] = {0x1a, 0x00};                                   // DLPF_CFG is 000 , with Gyro dlpf is 250hz
            // _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_GLPF, PrivateConfig.ICM42605_SPI_Freq, 2); // config
            // usleep(15);

            // uint8_t MPU6500_SPI_Config_INTC[2] = {0x37, 0x22};
            // _s_spiWrite(ICM42605_fd, MPU6500_SPI_Config_INTC, PrivateConfig.MPU6500_SPI_Freq, 2);
            // usleep(500);
            // uint8_t MPU6500_SPI_Config_INTE[2] = {0x38, 0x01};
            // _s_spiWrite(ICM42605_fd, MPU6500_SPI_Config_INTE, PrivateConfig.MPU6500_SPI_Freq, 2);
            // usleep(500);
        }
        else if (PrivateConfig.MPUType == MPUTypeI2C)
        {
        }
    }

    inline void IMUSensorsDataRead()
    {
        if (PrivateConfig.MPUType == MPUTypeI2C)
        {
            //
        }
        else if (PrivateConfig.MPUType == MPUTypeSPI)
        {
            uint8_t Tmp_ICM42605_SPI_BufferX[2] = {0};
            Tmp_ICM42605_SPI_BufferX[0] = 0xBA;
            _s_spiXfer(ICM42605_fd, Tmp_ICM42605_SPI_BufferX, Tmp_ICM42605_SPI_BufferX, PrivateConfig.ICM42605_SPI_Freq, 2);

            if (Tmp_ICM42605_SPI_BufferX[1] & 0x01)
            {
                PrivateData._uORB_ICM42605_IMUUpdateTime = GetTimestamp() - LastUpdate;
                LastUpdate = GetTimestamp();

                uint8_t Tmp_ICM42605_SPI_Buffer[8] = {0};
                uint8_t Tmp_ICM42605_SPI_Bufferout[8] = {0};
                Tmp_ICM42605_SPI_Buffer[0] = 0xBB;
                if (PrivateData._uORB_ICM42605_AccelCountDown >= (PrivateConfig.TargetFreqency / PrivateConfig.AccTargetFreqency))
                {
                    _s_spiXfer(ICM42605_fd, Tmp_ICM42605_SPI_Buffer, Tmp_ICM42605_SPI_Bufferout, PrivateConfig.ICM42605_SPI_Freq, 8);
                    int Tmp_AX = (short)((int)Tmp_ICM42605_SPI_Bufferout[1] << 8 | (int)Tmp_ICM42605_SPI_Bufferout[2]);
                    int Tmp_AY = (short)((int)Tmp_ICM42605_SPI_Bufferout[3] << 8 | (int)Tmp_ICM42605_SPI_Bufferout[4]);
                    int Tmp_AZ = (short)((int)Tmp_ICM42605_SPI_Bufferout[5] << 8 | (int)Tmp_ICM42605_SPI_Bufferout[6]);

                    // Step 1: rotate Yaw
                    int Tmp_A2X = Tmp_AX * cos(DEG2RAD((PrivateConfig.MPU_Flip___Yaw))) + Tmp_AY * sin(DEG2RAD((PrivateConfig.MPU_Flip___Yaw)));
                    int Tmp_A2Y = Tmp_AY * cos(DEG2RAD((PrivateConfig.MPU_Flip___Yaw))) + Tmp_AX * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip___Yaw)));

                    // Step 2: rotate Pitch
                    int Tmp_A3X = Tmp_A2X * cos(DEG2RAD(PrivateConfig.MPU_Flip_Pitch)) + Tmp_AZ * sin(DEG2RAD((PrivateConfig.MPU_Flip_Pitch)));
                    int Tmp_A3Z = Tmp_AZ * cos(DEG2RAD((PrivateConfig.MPU_Flip_Pitch))) + Tmp_A2X * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip_Pitch)));
                    // Step 3: rotate Roll
                    PrivateData._uORB_ICM42605_A_Y = Tmp_A2Y * cos(DEG2RAD((PrivateConfig.MPU_Flip__Roll))) + Tmp_A3Z * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip__Roll)));
                    PrivateData._uORB_ICM42605_A_Z = Tmp_A3Z * cos(DEG2RAD((PrivateConfig.MPU_Flip__Roll))) + Tmp_A2Y * sin(DEG2RAD((PrivateConfig.MPU_Flip__Roll)));
                    PrivateData._uORB_ICM42605_A_X = Tmp_A3X;

                    //
                    PrivateData._uORB_ICM42605_AccelCountDown = 0;
                }
                PrivateData._uORB_ICM42605_AccelCountDown++;
                {
                    uint8_t Tmp_ICM42605_SPI_GBuffer[8] = {0};
                    uint8_t Tmp_ICM42605_SPI_GBufferout[8] = {0};
                    Tmp_ICM42605_SPI_GBuffer[0] = 0xC3;
                    _s_spiXfer(ICM42605_fd, Tmp_ICM42605_SPI_GBuffer, Tmp_ICM42605_SPI_GBufferout, PrivateConfig.ICM42605_SPI_Freq, 8);
                    int Tmp_GX = (short)((int)Tmp_ICM42605_SPI_GBufferout[1] << 8 | (int)Tmp_ICM42605_SPI_GBufferout[2]);
                    int Tmp_GY = (short)((int)Tmp_ICM42605_SPI_GBufferout[3] << 8 | (int)Tmp_ICM42605_SPI_GBufferout[4]);
                    int Tmp_GZ = (short)((int)Tmp_ICM42605_SPI_GBufferout[5] << 8 | (int)Tmp_ICM42605_SPI_GBufferout[6]);
                    // Step 1: rotate Yaw
                    int Tmp_G2X = Tmp_GX * cos(DEG2RAD((PrivateConfig.MPU_Flip___Yaw))) + Tmp_GY * sin(DEG2RAD((PrivateConfig.MPU_Flip___Yaw)));
                    int Tmp_G2Y = Tmp_GY * cos(DEG2RAD((PrivateConfig.MPU_Flip___Yaw))) + Tmp_GX * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip___Yaw)));
                    // Step 2: rotate Pitch
                    int Tmp_G3X = Tmp_G2X * cos(DEG2RAD(PrivateConfig.MPU_Flip_Pitch)) + Tmp_GZ * sin(DEG2RAD((PrivateConfig.MPU_Flip_Pitch)));
                    int Tmp_G3Z = Tmp_GZ * cos(DEG2RAD((PrivateConfig.MPU_Flip_Pitch))) + Tmp_G2X * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip_Pitch)));
                    // Step 3: rotate Roll
                    PrivateData._uORB_ICM42605_G_Y = Tmp_G2Y * cos(DEG2RAD((PrivateConfig.MPU_Flip__Roll))) + Tmp_G3Z * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip__Roll)));
                    PrivateData._uORB_ICM42605_G_Z = Tmp_G3Z * cos(DEG2RAD((PrivateConfig.MPU_Flip__Roll))) + Tmp_G2Y * sin(DEG2RAD((PrivateConfig.MPU_Flip__Roll)));
                    PrivateData._uORB_ICM42605_G_X = Tmp_G3X;
               
                }
            }
        }
    }

        inline int GetTimestamp()
    {
        struct timespec tv;
        clock_gettime(CLOCK_MONOTONIC, &tv);
        return (((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000))) - IMUstartuptime);
    }

    int ICM42605_fd;
    int LastUpdate = 0;
    int IMUstartuptime = 0;

    ICM4Data PrivateData;
    ICM4Config PrivateConfig;
};