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

struct ICMData
{
    int DeviceType = 0;
};

struct ICMConfig
{
    bool MPUType = MPUTypeSPI;
    const char *MPUSPIChannel = "/dev/spidev0.0";
    int MPU6500_SPI_Freq = 400000;

};

class RPiICM20602
{
public:
    inline RPiICM20602()
    {
        IMUSensorsDeviceInit();

    }
private:
    inline void IMUSensorsDeviceInit()
    {
        // double OutputSpeedCal = (MPU_250HZ_LPF_SPEED / (float)PrivateConfig.TargetFreqency) - 1.f;
        if (PrivateConfig.MPUType == MPUTypeSPI)
        {
            ICM20602_fd = _s_spiOpen(PrivateConfig.MPUSPIChannel, PrivateConfig.MPU6500_SPI_Freq, 0);
            if (ICM20602_fd < 0)
                throw std::invalid_argument("[SPI] MPU device can't open");
            uint8_t MPU6500_SPI_Config_WHOAMI[2] = {0xf5, 0x00};
            _s_spiXfer(ICM20602_fd, MPU6500_SPI_Config_WHOAMI, MPU6500_SPI_Config_WHOAMI, PrivateConfig.MPU6500_SPI_Freq, 2);
            PrivateData.DeviceType = MPU6500_SPI_Config_WHOAMI[1];

            std::cout << "MPU6500_SPI_Config_WHOAMI[1]: " << static_cast<int>(MPU6500_SPI_Config_WHOAMI[1]) << "\r\n";
            // uint8_t MPU6500_SPI_Config_RESET[2] = {0x6b, 0x80};
            // _s_spiWrite(ICM20602_fd, MPU6500_SPI_Config_RESET, PrivateConfig.MPU6500_SPI_Freq, 2); // reset
            // usleep(500);
            // uint8_t MPU6500_SPI_Config_RESET2[2] = {0x68, 0x07};
            // _s_spiWrite(ICM20602_fd, MPU6500_SPI_Config_RESET, PrivateConfig.MPU6500_SPI_Freq, 2); // BIT_GYRO | BIT_ACC | BIT_TEMP reset
            // usleep(500);
            // uint8_t MPU6500_SPI_Config_RESET3[2] = {0x6b, 0x00};
            // _s_spiWrite(ICM20602_fd, MPU6500_SPI_Config_RESET3, PrivateConfig.MPU6500_SPI_Freq, 2); // reset
            // usleep(500);
            // uint8_t MPU6500_SPI_Config_RESET4[2] = {0x6b, 0x01};
            // _s_spiWrite(ICM20602_fd, MPU6500_SPI_Config_RESET4, PrivateConfig.MPU6500_SPI_Freq, 2); // reset
            // usleep(1000);

            // uint8_t MPU6500_SPI_Config_ALPF[2] = {0x1d, 0x00};                                   // FChoice 1, DLPF 3 , dlpf cut off 44.8hz for accel is 0x03, but now 0x00 is not apply accel hardware
            // _s_spiWrite(ICM20602_fd, MPU6500_SPI_Config_ALPF, PrivateConfig.MPU6500_SPI_Freq, 2); // Accel2
            // usleep(15);
            // uint8_t MPU6500_SPI_Config_Acce[2] = {0x1c, 0x18};                                   // Full AccelScale +- 16g
            // _s_spiWrite(ICM20602_fd, MPU6500_SPI_Config_Acce, PrivateConfig.MPU6500_SPI_Freq, 2); // Accel
            // usleep(15);
            // uint8_t MPU6500_SPI_Config_Gyro[2] = {0x1b, 0x18};                                   // Full GyroScale +-2000dps, dlpf 250hz
            // _s_spiWrite(ICM20602_fd, MPU6500_SPI_Config_Gyro, PrivateConfig.MPU6500_SPI_Freq, 2); // Gryo
            // usleep(15);
            // uint8_t MPU6500_SPI_Config_GLPF[2] = {0x1a, 0x00};                                   // DLPF_CFG is 000 , with Gyro dlpf is 250hz
            // _s_spiWrite(ICM20602_fd, MPU6500_SPI_Config_GLPF, PrivateConfig.MPU6500_SPI_Freq, 2); // config
            // usleep(15);

            // uint8_t MPU6500_SPI_Config_INTC[2] = {0x37, 0x22};
            // _s_spiWrite(ICM20602_fd, MPU6500_SPI_Config_INTC, PrivateConfig.MPU6500_SPI_Freq, 2);
            // usleep(500);
            // uint8_t MPU6500_SPI_Config_INTE[2] = {0x38, 0x01};
            // _s_spiWrite(ICM20602_fd, MPU6500_SPI_Config_INTE, PrivateConfig.MPU6500_SPI_Freq, 2);
            // usleep(500);
        }
        else if (PrivateConfig.MPUType == MPUTypeI2C)
        {
        }
    }
    int ICM20602_fd;

    ICMData PrivateData;
    ICMConfig PrivateConfig;
};