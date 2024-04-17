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

#define ICM42605_ACCEL_LSB 2048.f
#define MAX_ACC_NEARNESS 0.33 // 33% or G error soft-accepted (0.67-1.33G)
#define GravityAccel 9.80665

#define ICMTypeI2C 0
#define ICMTypeSPI 1
#define ICMMixKalman 1
#define ICMMixTradition 0
#define ICMAccelNoseUp 0
#define ICMAccelNoseDown 1
#define ICMAccelNoseRight 2
#define ICMAccelNoseLeft 3
#define ICMAccelNoseTop 4
#define ICMAccelNoseRev 5
#define ICMAccelCaliGet 6
#define ICMAccelCaliX 7
#define ICMAccelCaliY 8
#define ICMAccelCaliZ 9
#define ICMAccelScalX 10
#define ICMAccelScalY 11
#define ICMAccelScalZ 12
#define ICMAccelTRIM_Roll 13
#define ICMAccelTRIMPitch 14

#define ACC_CLIPPING_THRESHOLD_G 7.9f
#define DYNAMIC_NOTCH_DEFAULT_CENTER_HZ 350.f
#define DYN_NOTCH_SMOOTH_FREQ_HZ 50.f
#define ACC_VIBE_FLOOR_FILT_HZ 5.f
#define ACC_VIBE_FILT_HZ 2.f

#define FilterLPFPT1 0
#define FilterLPFBiquad 1
#define ICMTypeSPI 1
#ifndef PI
#define PI 3.14
#endif
#define ICM_250HZ_LPF_SPEED 8000.f
#define DEG2RAD(x) (x * PI / 180.f)

#define ICMTypeI2C 0
#define ICMTypeSPI 1

#define GAXR 0
#define GAYP 1
#define GAZY 2
#define AAXN 0
#define AAYN 1
#define AAZN 2

#define IMU_CALI_MAX_LOOP 500.0

#define DEG2RAD(x) (x * PI / 180.f)

struct ICM4Data
{
    int DeviceType = 0;

    int _uORB_ICM42605_A_X = 0;
    int _uORB_ICM42605_A_Y = 0;
    int _uORB_ICM42605_A_Z = 0;
    int _uORB_ICM42605_G_X = 0;
    int _uORB_ICM42605_G_Y = 0;
    int _uORB_ICM42605_G_Z = 0;
    int _uORB_ICM42605_M_X = 0;
    int _uORB_ICM42605_M_Y = 0;
    int _uORB_ICM42605_M_Z = 0;

    float _uORB_ICM42605_ADF_X = 0;
    float _uORB_ICM42605_ADF_Y = 0;
    float _uORB_ICM42605_ADF_Z = 0;

    float _uORB_ICM42605_A_Vector = 0;
    float _uORB_ICM42605_A_Static_Vector = 0;

    float _uORB_Accel_VIBE_X = 0;
    float _uORB_Accel_VIBE_Y = 0;
    float _uORB_Accel_VIBE_Z = 0;

    float _uORB_Gryo__Roll = 0;
    float _uORB_Gryo_Pitch = 0;
    float _uORB_Gryo___Yaw = 0;
    float _uORB_Real__Roll = 0;
    float _uORB_Real_Pitch = 0;
    float _uORB_Real___Yaw = 0;
    float _uORB_Accel__Roll = 0;
    float _uORB_Accel_Pitch = 0;
    float _uORB_Acceleration_X = 0;
    float _uORB_Acceleration_Y = 0;
    float _uORB_Acceleration_Z = 0;
    float _uORB_Raw_QuaternionQ[4] = {0};

    double _flag_ICM42605_A_X_Scal = 1.f;
    double _flag_ICM42605_A_Y_Scal = 1.f;
    double _flag_ICM42605_A_Z_Scal = 1.f;
    double _flag_ICM42605_A_X_Cali = 0;
    double _flag_ICM42605_A_Y_Cali = 0;
    double _flag_ICM42605_A_Z_Cali = 0;
    double _flag_ICM42605_A_TP_Cali = 0;
    double _flag_ICM42605_A_TR_Cali = 0;

    float _uORB_ICM42605_A_Static_X = 0;
    float _uORB_ICM42605_A_Static_Y = 0;
    float _uORB_ICM42605_A_Static_Z = 0;

    int _flag_ICM42605_G_X_Cali;
    int _flag_ICM42605_G_Y_Cali;
    int _flag_ICM42605_G_Z_Cali;

    bool _uORB_ICM42605_ACC_Clipped = false;
    float ICMMixTraditionBeta = 0.05;

    Eigen::Matrix3d _uORB_ICM42605_RotationMatrix;
    Eigen::Quaternion<double> _uORB_ICM42605_Quaternion;

    int _uORB_ICM42605_IMUUpdateTime = 0;
    int _uORB_ICM42605_AccelCountDown = 0;
};

struct ICM4Config
{
    bool ICMType = ICMTypeSPI;
    const char *ICMSPIChannel = "/dev/spidev0.0";
    float ICM_Flip_Pitch = 0;
    float ICM_Flip__Roll = 0;
    float ICM_Flip___Yaw = 270;

    int TargetFreqency = 1000;

    int ICM42605_SPI_Freq = 400000;
    float ICM_42605_LSB = 65.5 / 4;

    bool DynamicNotchEnable = true;

    float GyroToAccelBeta = 0.2;

    int GyroFilterNotchCutOff = 0;
    int GyroFilterType = FilterLPFPT1;
    int GyroFilterCutOff = 90;
    int GyroFilterTypeST2 = FilterLPFPT1;
    int GyroFilterCutOffST2 = 0;

    int AccelFilterType = FilterLPFBiquad;
    int AccTargetFreqency = 1000;
    int AccelFilterCutOff = 30;

    int AccelFilterNotchCutOff = 0;
};

class RPiICM42605
{
public:
    inline RPiICM42605(ICM4Config icmConfig)
    {
        struct timespec tv;
        clock_gettime(CLOCK_MONOTONIC, &tv);
        LastUpdate = GetTimestamp();
        IMUstartuptime = (((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000))));
        PrivateConfig = icmConfig;
        {
            int DT = (float)(1.f / (float)PrivateConfig.TargetFreqency) * 1000000;
            int ACCDT = (float)(1.f / (float)PrivateConfig.AccTargetFreqency) * 1000000;
            switch (PrivateConfig.GyroFilterType)
            {
            case FilterLPFPT1:
            {
                pt1FilterInit(&GryoFilterLPF[GAXR], PrivateConfig.GyroFilterCutOff, DT * 1e-6f);
                pt1FilterInit(&GryoFilterLPF[GAYP], PrivateConfig.GyroFilterCutOff, DT * 1e-6f);
                pt1FilterInit(&GryoFilterLPF[GAZY], PrivateConfig.GyroFilterCutOff, DT * 1e-6f);
            }
            break;
            case FilterLPFBiquad:
            {
                biquadFilterInitLPF(&GryoFilterBLPF[GAXR], PrivateConfig.GyroFilterCutOff, DT);
                biquadFilterInitLPF(&GryoFilterBLPF[GAYP], PrivateConfig.GyroFilterCutOff, DT);
                biquadFilterInitLPF(&GryoFilterBLPF[GAZY], PrivateConfig.GyroFilterCutOff, DT);
            }
            break;
            }
            switch (PrivateConfig.GyroFilterTypeST2)
            {
            case FilterLPFPT1:
            {
                pt1FilterInit(&GryoFilterLPFST2[GAXR], PrivateConfig.GyroFilterCutOffST2, DT * 1e-6f);
                pt1FilterInit(&GryoFilterLPFST2[GAYP], PrivateConfig.GyroFilterCutOffST2, DT * 1e-6f);
                pt1FilterInit(&GryoFilterLPFST2[GAZY], PrivateConfig.GyroFilterCutOffST2, DT * 1e-6f);
            }
            break;
            case FilterLPFBiquad:
            {
                biquadFilterInitLPF(&GryoFilterBLPFST2[GAXR], PrivateConfig.GyroFilterCutOffST2, DT);
                biquadFilterInitLPF(&GryoFilterBLPFST2[GAYP], PrivateConfig.GyroFilterCutOffST2, DT);
                biquadFilterInitLPF(&GryoFilterBLPFST2[GAZY], PrivateConfig.GyroFilterCutOffST2, DT);
            }
            break;
            }
            switch (PrivateConfig.AccelFilterType)
            {
            case FilterLPFPT1:
            {
                pt1FilterInit(&AccelFilterLPF[AAXN], PrivateConfig.AccelFilterCutOff, ACCDT * 1e-6f);
                pt1FilterInit(&AccelFilterLPF[AAYN], PrivateConfig.AccelFilterCutOff, ACCDT * 1e-6f);
                pt1FilterInit(&AccelFilterLPF[AAZN], PrivateConfig.AccelFilterCutOff, ACCDT * 1e-6f);
            }
            break;
            case FilterLPFBiquad:
            {
                biquadFilterInitLPF(&AccelFilterBLPF[AAXN], PrivateConfig.AccelFilterCutOff, ACCDT);
                biquadFilterInitLPF(&AccelFilterBLPF[AAYN], PrivateConfig.AccelFilterCutOff, ACCDT);
                biquadFilterInitLPF(&AccelFilterBLPF[AAZN], PrivateConfig.AccelFilterCutOff, ACCDT);
            }
            break;
            }
            if (PrivateConfig.DynamicNotchEnable)
            {
                biquadFilterInitLPF(&GryoFilterDynamicFreq[GAXR], DYN_NOTCH_SMOOTH_FREQ_HZ, (DT * 16 * (PrivateConfig.TargetFreqency / 1000)));
                biquadFilterInitLPF(&GryoFilterDynamicFreq[GAYP], DYN_NOTCH_SMOOTH_FREQ_HZ, (DT * 16 * (PrivateConfig.TargetFreqency / 1000)));
                biquadFilterInitLPF(&GryoFilterDynamicFreq[GAZY], DYN_NOTCH_SMOOTH_FREQ_HZ, (DT * 16 * (PrivateConfig.TargetFreqency / 1000)));
                biquadFilterInit(&GryoFilterDynamicNotch[GAXR], DYNAMIC_NOTCH_DEFAULT_CENTER_HZ, DT, 1.0f, FILTER_NOTCH);
                biquadFilterInit(&GryoFilterDynamicNotch[GAYP], DYNAMIC_NOTCH_DEFAULT_CENTER_HZ, DT, 1.0f, FILTER_NOTCH);
                biquadFilterInit(&GryoFilterDynamicNotch[GAZY], DYNAMIC_NOTCH_DEFAULT_CENTER_HZ, DT, 1.0f, FILTER_NOTCH);
            }
            pt1FilterInit(&VibeFloorLPF[AAXN], ACC_VIBE_FLOOR_FILT_HZ, ACCDT * 1e-6f);
            pt1FilterInit(&VibeFloorLPF[AAYN], ACC_VIBE_FLOOR_FILT_HZ, ACCDT * 1e-6f);
            pt1FilterInit(&VibeFloorLPF[AAZN], ACC_VIBE_FLOOR_FILT_HZ, ACCDT * 1e-6f);
            pt1FilterInit(&VibeLPF[AAXN], ACC_VIBE_FILT_HZ, ACCDT * 1e-6f);
            pt1FilterInit(&VibeLPF[AAYN], ACC_VIBE_FILT_HZ, ACCDT * 1e-6f);
            pt1FilterInit(&VibeLPF[AAZN], ACC_VIBE_FILT_HZ, ACCDT * 1e-6f);
        }
        IMUSensorsDeviceInit();
        AHRSSys.reset(new MadgwickAHRS(PrivateConfig.GyroToAccelBeta, PrivateConfig.TargetFreqency));
    }

    inline int ICMCalibration(double *AccelCaliData)
    {
        PrivateData._flag_ICM42605_A_X_Scal = AccelCaliData[ICMAccelScalX];
        PrivateData._flag_ICM42605_A_Y_Scal = AccelCaliData[ICMAccelScalY];
        PrivateData._flag_ICM42605_A_Z_Scal = AccelCaliData[ICMAccelScalZ];
        PrivateData._flag_ICM42605_A_X_Cali = AccelCaliData[ICMAccelCaliX];
        PrivateData._flag_ICM42605_A_Y_Cali = AccelCaliData[ICMAccelCaliY];
        PrivateData._flag_ICM42605_A_Z_Cali = AccelCaliData[ICMAccelCaliZ];
        PrivateData._flag_ICM42605_A_TR_Cali = AccelCaliData[ICMAccelTRIM_Roll];
        PrivateData._flag_ICM42605_A_TP_Cali = AccelCaliData[ICMAccelTRIMPitch];

        double _Tmp_Gryo_X_Cali = 0;
        double _Tmp_Gryo_Y_Cali = 0;
        double _Tmp_Gryo_Z_Cali = 0;
        double _Tmp_Accel_Static_Cali = 0;
        PrivateData._flag_ICM42605_G_X_Cali = 0;
        PrivateData._flag_ICM42605_G_Y_Cali = 0;
        PrivateData._flag_ICM42605_G_Z_Cali = 0;

        for (int cali_count = 0; cali_count < IMU_CALI_MAX_LOOP; cali_count++)
        {
            ICMSensorsDataRead();
            ResetICMMixAngle();
            _Tmp_Gryo_X_Cali += PrivateData._uORB_ICM42605_G_X;
            _Tmp_Gryo_Y_Cali += PrivateData._uORB_ICM42605_G_Y;
            _Tmp_Gryo_Z_Cali += PrivateData._uORB_ICM42605_G_Z;
            usleep((int)(1.f / (float)PrivateConfig.TargetFreqency * 1000000.f));
        }

        for (int cali_count = 0; cali_count < IMU_CALI_MAX_LOOP; cali_count++)
        {
            ICMSensorsDataGet();
            _Tmp_Accel_Static_Cali += PrivateData._uORB_ICM42605_A_Vector;
            usleep((int)(1.f / (float)PrivateConfig.TargetFreqency * 1000000.f));
        }
        PrivateData._flag_ICM42605_G_X_Cali = _Tmp_Gryo_X_Cali / IMU_CALI_MAX_LOOP;
        PrivateData._flag_ICM42605_G_Y_Cali = _Tmp_Gryo_Y_Cali / IMU_CALI_MAX_LOOP;
        PrivateData._flag_ICM42605_G_Z_Cali = _Tmp_Gryo_Z_Cali / IMU_CALI_MAX_LOOP;
        PrivateData._uORB_ICM42605_A_Static_Vector = _Tmp_Accel_Static_Cali / IMU_CALI_MAX_LOOP;
        return 0;
    }

    inline void ICMAccelCalibration(int AccelCaliAction, double *AccelCaliData)
    {
        int AccelCaliTmpTotal = 0;
        AccelCaliData[AccelCaliAction] = 0;
        for (int cali_count = 0; cali_count < 2000; cali_count++)
        {
            ICMSensorsDataRead();
            switch (AccelCaliAction)
            {
            case ICMAccelNoseUp:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_ICM42605_A_X;
                break;
            case ICMAccelNoseDown:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_ICM42605_A_X;
                break;
            case ICMAccelNoseRight:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_ICM42605_A_Y;
                break;
            case ICMAccelNoseLeft:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_ICM42605_A_Y;
                break;
            case ICMAccelNoseTop:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_ICM42605_A_Z;
                break;
            case ICMAccelNoseRev:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_ICM42605_A_Z;
                break;
            }
            usleep((int)(1.f / (float)PrivateConfig.AccTargetFreqency * 1000000.f));
        }
        if (AccelCaliAction == ICMAccelCaliGet)
        {
            AccelCaliData[ICMAccelNoseUp] /= 2000.f;
            AccelCaliData[ICMAccelNoseDown] /= 2000.f;
            AccelCaliData[ICMAccelNoseRight] /= 2000.f;
            AccelCaliData[ICMAccelNoseLeft] /= 2000.f;
            AccelCaliData[ICMAccelNoseTop] /= 2000.f;
            AccelCaliData[ICMAccelNoseRev] /= 2000.f;

            std::cout << "Data-X-Max:" << AccelCaliData[ICMAccelNoseUp] << "\r\n";
            std::cout << "Data-X-Min:" << AccelCaliData[ICMAccelNoseDown] << "\r\n";
            std::cout << "Data-Y-Max:" << AccelCaliData[ICMAccelNoseLeft] << "\r\n";
            std::cout << "Data-Y-Min:" << AccelCaliData[ICMAccelNoseRight] << "\r\n";
            std::cout << "Data-Z-Max:" << AccelCaliData[ICMAccelNoseTop] << "\r\n";
            std::cout << "Data-Z-Min:" << AccelCaliData[ICMAccelNoseRev] << "\r\n";

            AccelCaliData[ICMAccelScalX] = std::abs(ICM42605_Accel_LSB / ((AccelCaliData[ICMAccelNoseUp] - AccelCaliData[ICMAccelNoseDown]) / 2.f));
            AccelCaliData[ICMAccelScalY] = std::abs(ICM42605_Accel_LSB / ((AccelCaliData[ICMAccelNoseLeft] - AccelCaliData[ICMAccelNoseRight]) / 2.f));
            AccelCaliData[ICMAccelScalZ] = std::abs(ICM42605_Accel_LSB / ((AccelCaliData[ICMAccelNoseTop] - AccelCaliData[ICMAccelNoseRev]) / 2.f));
            AccelCaliData[ICMAccelCaliX] = ((AccelCaliData[ICMAccelNoseUp] + AccelCaliData[ICMAccelNoseDown]) / 2.f) * AccelCaliData[ICMAccelScalX];
            AccelCaliData[ICMAccelCaliY] = ((AccelCaliData[ICMAccelNoseRight] + AccelCaliData[ICMAccelNoseLeft]) / 2.f) * AccelCaliData[ICMAccelScalY];
            AccelCaliData[ICMAccelCaliZ] = ((AccelCaliData[ICMAccelNoseTop] + AccelCaliData[ICMAccelNoseRev]) / 2.f) * AccelCaliData[ICMAccelScalZ];
        }
    }

    inline ICM4Data ICMSensorsDataGet()
    {
        ICMSensorsDataRead();
        // PrivateData._uORB_ICM42605_G_X -= PrivateData._flag_ICM42605_G_X_Cali;
        // PrivateData._uORB_ICM42605_G_Y -= PrivateData._flag_ICM42605_G_Y_Cali;
        // PrivateData._uORB_ICM42605_G_Z -= PrivateData._flag_ICM42605_G_Z_Cali;

        // PrivateData._uORB_ICM42605_A_X = PrivateData._uORB_ICM42605_A_X * PrivateData._flag_ICM42605_A_X_Scal - PrivateData._flag_ICM42605_A_X_Cali;
        // PrivateData._uORB_ICM42605_A_Y = PrivateData._uORB_ICM42605_A_Y * PrivateData._flag_ICM42605_A_Y_Scal - PrivateData._flag_ICM42605_A_Y_Cali;
        // PrivateData._uORB_ICM42605_A_Z = PrivateData._uORB_ICM42605_A_Z * PrivateData._flag_ICM42605_A_Z_Scal - PrivateData._flag_ICM42605_A_Z_Cali;

        //==================================================Gyro Filter
        {
            switch (PrivateConfig.GyroFilterType)
            {
            case FilterLPFPT1:
            {
                if (PrivateConfig.GyroFilterCutOff)
                {
                    PrivateData._uORB_Gryo__Roll = pt1FilterApply(&GryoFilterLPF[GAXR], ((float)PrivateData._uORB_ICM42605_G_X / PrivateConfig.ICM_42605_LSB));
                    PrivateData._uORB_Gryo_Pitch = pt1FilterApply(&GryoFilterLPF[GAYP], ((float)PrivateData._uORB_ICM42605_G_Y / PrivateConfig.ICM_42605_LSB));
                    PrivateData._uORB_Gryo___Yaw = pt1FilterApply(&GryoFilterLPF[GAZY], ((float)PrivateData._uORB_ICM42605_G_Z / PrivateConfig.ICM_42605_LSB));
                }
                else
                {
                    PrivateData._uORB_Gryo__Roll = ((float)PrivateData._uORB_ICM42605_G_X / PrivateConfig.ICM_42605_LSB);
                    PrivateData._uORB_Gryo_Pitch = ((float)PrivateData._uORB_ICM42605_G_Y / PrivateConfig.ICM_42605_LSB);
                    PrivateData._uORB_Gryo___Yaw = ((float)PrivateData._uORB_ICM42605_G_Z / PrivateConfig.ICM_42605_LSB);
                }
                break;
            }
            case FilterLPFBiquad:
            {
                if (PrivateConfig.GyroFilterCutOff)
                {
                    PrivateData._uORB_Gryo__Roll = biquadFilterApply(&GryoFilterBLPF[GAXR], ((float)PrivateData._uORB_ICM42605_G_X / PrivateConfig.ICM_42605_LSB));
                    PrivateData._uORB_Gryo_Pitch = biquadFilterApply(&GryoFilterBLPF[GAYP], ((float)PrivateData._uORB_ICM42605_G_Y / PrivateConfig.ICM_42605_LSB));
                    PrivateData._uORB_Gryo___Yaw = biquadFilterApply(&GryoFilterBLPF[GAZY], ((float)PrivateData._uORB_ICM42605_G_Z / PrivateConfig.ICM_42605_LSB));
                }
                else
                {
                    PrivateData._uORB_Gryo__Roll = ((float)PrivateData._uORB_ICM42605_G_X / PrivateConfig.ICM_42605_LSB);
                    PrivateData._uORB_Gryo_Pitch = ((float)PrivateData._uORB_ICM42605_G_Y / PrivateConfig.ICM_42605_LSB);
                    PrivateData._uORB_Gryo___Yaw = ((float)PrivateData._uORB_ICM42605_G_Z / PrivateConfig.ICM_42605_LSB);
                }
            }
            break;
            }
            switch (PrivateConfig.GyroFilterTypeST2)
            {
            case FilterLPFPT1:
                if (PrivateConfig.GyroFilterCutOffST2)
                {
                    PrivateData._uORB_Gryo__Roll = pt1FilterApply(&GryoFilterLPFST2[GAXR], PrivateData._uORB_Gryo__Roll);
                    PrivateData._uORB_Gryo_Pitch = pt1FilterApply(&GryoFilterLPFST2[GAYP], PrivateData._uORB_Gryo_Pitch);
                    PrivateData._uORB_Gryo___Yaw = pt1FilterApply(&GryoFilterLPFST2[GAZY], PrivateData._uORB_Gryo___Yaw);
                }
                break;
            case FilterLPFBiquad:
                if (PrivateConfig.GyroFilterCutOffST2)
                {
                    PrivateData._uORB_Gryo__Roll = biquadFilterApply(&GryoFilterBLPFST2[GAXR], PrivateData._uORB_Gryo__Roll);
                    PrivateData._uORB_Gryo_Pitch = biquadFilterApply(&GryoFilterBLPFST2[GAYP], PrivateData._uORB_Gryo_Pitch);
                    PrivateData._uORB_Gryo___Yaw = biquadFilterApply(&GryoFilterBLPFST2[GAZY], PrivateData._uORB_Gryo___Yaw);
                }
                break;
            }

            if (PrivateConfig.GyroFilterNotchCutOff)
            {
                PrivateData._uORB_Gryo_Pitch = biquadFilterApply(&GyroNotchFilter[GAXR], PrivateData._uORB_Gryo_Pitch);
                PrivateData._uORB_Gryo__Roll = biquadFilterApply(&GyroNotchFilter[GAYP], PrivateData._uORB_Gryo__Roll);
                PrivateData._uORB_Gryo___Yaw = biquadFilterApply(&GyroNotchFilter[GAZY], PrivateData._uORB_Gryo___Yaw);
            }
        }

        //========================= //=========================Dynamic Anlyisc
        {
            if (PrivateConfig.TargetFreqency >= 1000)
            {
            }
        }
        //========================= //=========================Accel Filter
        {
            if (PrivateData._uORB_ICM42605_AccelCountDown == 1)
            {
                float AccVibeFloorX = pt1FilterApply(&VibeFloorLPF[AAXN], (PrivateData._uORB_ICM42605_A_X / ICM42605_ACCEL_LSB));
                float AccVibeFloorY = pt1FilterApply(&VibeFloorLPF[AAYN], (PrivateData._uORB_ICM42605_A_Y / ICM42605_ACCEL_LSB));
                float AccVibeFloorZ = pt1FilterApply(&VibeFloorLPF[AAZN], (PrivateData._uORB_ICM42605_A_Z / ICM42605_ACCEL_LSB));
                PrivateData._uORB_Accel_VIBE_X = pt1FilterApply(&VibeLPF[AAXN], (pow((((float)PrivateData._uORB_ICM42605_A_X / ICM42605_ACCEL_LSB) - AccVibeFloorX), 2)));
                PrivateData._uORB_Accel_VIBE_Y = pt1FilterApply(&VibeLPF[AAYN], (pow((((float)PrivateData._uORB_ICM42605_A_Y / ICM42605_ACCEL_LSB) - AccVibeFloorY), 2)));
                PrivateData._uORB_Accel_VIBE_Z = pt1FilterApply(&VibeLPF[AAZN], (pow((((float)PrivateData._uORB_ICM42605_A_Z / ICM42605_ACCEL_LSB) - AccVibeFloorZ), 2)));

                //
                if (PrivateConfig.AccelFilterCutOff)
                {
                    switch (PrivateConfig.AccelFilterType)
                    {
                    case FilterLPFPT1:
                        PrivateData._uORB_ICM42605_ADF_X = pt1FilterApply(&AccelFilterLPF[AAXN], ((float)PrivateData._uORB_ICM42605_A_X / ICM42605_ACCEL_LSB));
                        PrivateData._uORB_ICM42605_ADF_Y = pt1FilterApply(&AccelFilterLPF[AAYN], ((float)PrivateData._uORB_ICM42605_A_Y / ICM42605_ACCEL_LSB));
                        PrivateData._uORB_ICM42605_ADF_Z = pt1FilterApply(&AccelFilterLPF[AAZN], ((float)PrivateData._uORB_ICM42605_A_Z / ICM42605_ACCEL_LSB));

                        break;
                    case FilterLPFBiquad:
                        PrivateData._uORB_ICM42605_ADF_X = biquadFilterApply(&AccelFilterBLPF[AAXN], ((float)PrivateData._uORB_ICM42605_A_X / ICM42605_ACCEL_LSB));
                        PrivateData._uORB_ICM42605_ADF_Y = biquadFilterApply(&AccelFilterBLPF[AAYN], ((float)PrivateData._uORB_ICM42605_A_Y / ICM42605_ACCEL_LSB));
                        PrivateData._uORB_ICM42605_ADF_Z = biquadFilterApply(&AccelFilterBLPF[AAZN], ((float)PrivateData._uORB_ICM42605_A_Z / ICM42605_ACCEL_LSB));
                        break;
                    }
                    if (PrivateConfig.AccelFilterNotchCutOff)
                    {
                        PrivateData._uORB_ICM42605_ADF_X = biquadFilterApply(&AccelNotchFilter[AAXN], PrivateData._uORB_ICM42605_ADF_X);
                        PrivateData._uORB_ICM42605_ADF_Y = biquadFilterApply(&AccelNotchFilter[AAYN], PrivateData._uORB_ICM42605_ADF_Y);
                        PrivateData._uORB_ICM42605_ADF_Z = biquadFilterApply(&AccelNotchFilter[AAZN], PrivateData._uORB_ICM42605_ADF_Z);
                    }
                }
                else
                {
                    PrivateData._uORB_ICM42605_ADF_X = PrivateData._uORB_ICM42605_ADF_X / ICM42605_ACCEL_LSB;
                    PrivateData._uORB_ICM42605_ADF_Y = PrivateData._uORB_ICM42605_ADF_X / ICM42605_ACCEL_LSB;
                    PrivateData._uORB_ICM42605_ADF_Z = PrivateData._uORB_ICM42605_ADF_X / ICM42605_ACCEL_LSB;
                }
                //
                if (abs(PrivateData._uORB_ICM42605_ADF_X) > ACC_CLIPPING_THRESHOLD_G ||
                    abs(PrivateData._uORB_ICM42605_ADF_Y) > ACC_CLIPPING_THRESHOLD_G ||
                    abs(PrivateData._uORB_ICM42605_ADF_Z) > ACC_CLIPPING_THRESHOLD_G)
                    PrivateData._uORB_ICM42605_ACC_Clipped = true;
                else
                    PrivateData._uORB_ICM42605_ACC_Clipped = false;
                //

                PrivateData._uORB_ICM42605_A_Vector = sqrtf((PrivateData._uORB_ICM42605_ADF_X * PrivateData._uORB_ICM42605_ADF_X) +
                                                            (PrivateData._uORB_ICM42605_ADF_Y * PrivateData._uORB_ICM42605_ADF_Y) +
                                                            (PrivateData._uORB_ICM42605_ADF_Z * PrivateData._uORB_ICM42605_ADF_Z));
                PrivateData._uORB_Accel_Pitch = -1 * atan2((float)PrivateData._uORB_ICM42605_ADF_X, PrivateData._uORB_ICM42605_ADF_Z) * 180.f / PI;
                PrivateData._uORB_Accel__Roll = atan2((float)PrivateData._uORB_ICM42605_ADF_Y, PrivateData._uORB_ICM42605_ADF_Z) * 180.f / PI;
            }
        }
        //========================= //=========================AHRS Update
        {

            if ((PrivateData._uORB_ICM42605_A_Vector > (PrivateData._uORB_ICM42605_A_Static_Vector - MAX_ACC_NEARNESS)) &&
                (PrivateData._uORB_ICM42605_A_Vector < (PrivateData._uORB_ICM42605_A_Static_Vector + MAX_ACC_NEARNESS)))
                PrivateData.ICMMixTraditionBeta = PrivateConfig.GyroToAccelBeta;
            else
                PrivateData.ICMMixTraditionBeta = 0.f;
            if (!AHRSEnable)
                AHRSSys->MadgwickAHRSIMUApply(PrivateData._uORB_Gryo__Roll, PrivateData._uORB_Gryo_Pitch, PrivateData._uORB_Gryo___Yaw,
                                              (PrivateData._uORB_ICM42605_ADF_X),
                                              (PrivateData._uORB_ICM42605_ADF_Y),
                                              (PrivateData._uORB_ICM42605_ADF_Z),
                                              ((float)PrivateData._uORB_ICM42605_IMUUpdateTime * 1e-6f));
            else
                AHRSSys->MadgwickAHRSApply(PrivateData._uORB_Gryo__Roll, PrivateData._uORB_Gryo_Pitch, PrivateData._uORB_Gryo___Yaw,
                                           (PrivateData._uORB_ICM42605_ADF_X),
                                           (PrivateData._uORB_ICM42605_ADF_Y),
                                           (PrivateData._uORB_ICM42605_ADF_Z),
                                           (_Tmp_AHRS_MAG_X),
                                           (_Tmp_AHRS_MAG_Y),
                                           (_Tmp_AHRS_MAG_Z),
                                           ((float)PrivateData._uORB_ICM42605_IMUUpdateTime * 1e-6f));

            AHRSSys->MadgwickSetAccelWeight(PrivateData.ICMMixTraditionBeta);
            AHRSSys->MadgwickAHRSGetQ(PrivateData._uORB_Raw_QuaternionQ[0],
                                      PrivateData._uORB_Raw_QuaternionQ[1],
                                      PrivateData._uORB_Raw_QuaternionQ[2],
                                      PrivateData._uORB_Raw_QuaternionQ[3]);
            AHRSSys->MadgwickComputeAngles(PrivateData._uORB_Real__Roll, PrivateData._uORB_Real_Pitch, PrivateData._uORB_Real___Yaw);

            PrivateData._uORB_Real__Roll *= 180.f / PI;
            PrivateData._uORB_Real_Pitch *= 180.f / PI;
            PrivateData._uORB_Real___Yaw *= 180.f / PI;
            PrivateData._uORB_Real___Yaw = PrivateData._uORB_Real___Yaw > 0 ? 360 - PrivateData._uORB_Real___Yaw : PrivateData._uORB_Real___Yaw;
            PrivateData._uORB_Real___Yaw = PrivateData._uORB_Real___Yaw < 0 ? -1 * PrivateData._uORB_Real___Yaw : PrivateData._uORB_Real___Yaw;
            //
            PrivateData._uORB_Real__Roll += PrivateData._flag_ICM42605_A_TR_Cali;
            PrivateData._uORB_Real_Pitch += PrivateData._flag_ICM42605_A_TP_Cali;

        }
        //========================= //=========================Navigation update
        {
            if (PrivateData._uORB_ICM42605_AccelCountDown == 1)
            {
                PrivateData._uORB_ICM42605_Quaternion = Eigen::AngleAxisd(((PrivateData._uORB_Real__Roll - PrivateData._flag_ICM42605_A_TR_Cali) * (PI / 180.f)), Eigen::Vector3d::UnitZ()) *
                                                        Eigen::AngleAxisd(((PrivateData._uORB_Real_Pitch - PrivateData._flag_ICM42605_A_TP_Cali) * (PI / 180.f)), Eigen::Vector3d::UnitY()) *
                                                        Eigen::AngleAxisd((0 * (PI / 180.f)), Eigen::Vector3d::UnitX());

                PrivateData._uORB_ICM42605_RotationMatrix = PrivateData._uORB_ICM42605_Quaternion.normalized().toRotationMatrix();
                Eigen::Matrix<double, 1, 3> AccelRaw;
                AccelRaw << PrivateData._uORB_ICM42605_ADF_Z,
                    PrivateData._uORB_ICM42605_ADF_Y,
                    PrivateData._uORB_ICM42605_ADF_X;

                Eigen::Matrix<double, 1, 3> AccelStatic = AccelRaw * PrivateData._uORB_ICM42605_RotationMatrix;
                PrivateData._uORB_ICM42605_A_Static_Z = AccelStatic[0] - PrivateData._uORB_ICM42605_A_Static_Vector;
                PrivateData._uORB_ICM42605_A_Static_X = -1.f * AccelStatic[1];
                PrivateData._uORB_ICM42605_A_Static_Y = -1.f * AccelStatic[2];
                PrivateData._uORB_Acceleration_X = ((float)PrivateData._uORB_ICM42605_A_Static_X) * GravityAccel * 100.f;
                PrivateData._uORB_Acceleration_Y = ((float)PrivateData._uORB_ICM42605_A_Static_Y) * GravityAccel * 100.f;
                PrivateData._uORB_Acceleration_Z = ((float)PrivateData._uORB_ICM42605_A_Static_Z) * GravityAccel * 100.f;
            }
        }
        return PrivateData;
    }

    inline void ResetICMMixAngle()
    {
        AHRSSys->MadgwickResetToAccel();
    }

    inline ~RPiICM42605()
    {
        AHRSSys.reset();
        _s_spiClose(ICM42605_fd);
    };

private:
    inline void IMUSensorsDeviceInit()
    {
        // double OutputSpeedCal = (ICM_250HZ_LPF_SPEED / (float)PrivateConfig.TargetFreqency) - 1.f;
        if (PrivateConfig.ICMType == ICMTypeSPI)
        {
            ICM42605_fd = _s_spiOpen(PrivateConfig.ICMSPIChannel, PrivateConfig.ICM42605_SPI_Freq, 0);
            if (ICM42605_fd < 0)
                throw std::invalid_argument("[SPI] ICM device can't open");
            uint8_t ICM42605_SPI_Config_WHOAMI[2] = {0xf5, 0x00};
            _s_spiXfer(ICM42605_fd, ICM42605_SPI_Config_WHOAMI, ICM42605_SPI_Config_WHOAMI, PrivateConfig.ICM42605_SPI_Freq, 2);
            PrivateData.DeviceType = ICM42605_SPI_Config_WHOAMI[1];
            std::cout << "Device Type: " << std::dec << static_cast<int>(PrivateData.DeviceType) << std::endl;

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
            _s_spiXfer(ICM42605_fd, ICM42605_SPI_Config_CONF5, ICM42605_SPI_Config_CONF5, PrivateConfig.ICM42605_SPI_Freq, 2);
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
            uint8_t ICM42605_SPI_Config_CONF9[2] = {0x5f, 0x63};
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF9, PrivateConfig.ICM42605_SPI_Freq, 2);
            usleep(500);
            uint8_t ICM42605_SPI_Config_CONF10[2] = {0x5f, 0x63};
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF10, PrivateConfig.ICM42605_SPI_Freq, 2); // Enable the accel and gyro to the FIFO
            usleep(500);

            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF3, PrivateConfig.ICM42605_SPI_Freq, 2); // bank0
            usleep(500);
            uint8_t ICM42605_SPI_Config_CONF11[2] = {0x14, 0x36};
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF11, PrivateConfig.ICM42605_SPI_Freq, 2);
            usleep(500);

            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF3, PrivateConfig.ICM42605_SPI_Freq, 2); // bank0
            usleep(500);
            uint8_t ICM42605_SPI_Config_CON12[2] = {0xe5, 0x00};
            _s_spiXfer(ICM42605_fd, ICM42605_SPI_Config_CON12, ICM42605_SPI_Config_CON12, PrivateConfig.ICM42605_SPI_Freq, 2);
            usleep(500);
            ICM42605_SPI_Config_CON12[1] |= (1 << 2); // FIFO_THS_INT1_ENABLE
            uint8_t ICM42605_SPI_Config_CONF13[2] = {0x65, ICM42605_SPI_Config_CON12[1]};
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF11, PrivateConfig.ICM42605_SPI_Freq, 2);
            usleep(500);

            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF3, PrivateConfig.ICM42605_SPI_Freq, 2); // bank0
            usleep(500);
            uint8_t ICM42605_SPI_Config_Accel[2] = {0x50, 0x0f};
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_Accel, PrivateConfig.ICM42605_SPI_Freq, 2); // 16g || 500hz lp
            usleep(500);

            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF3, PrivateConfig.ICM42605_SPI_Freq, 2); // bank0
            usleep(500);
            uint8_t ICM42605_SPI_Config_Gyro[2] = {0x4f, 0x0f};
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_Gyro, PrivateConfig.ICM42605_SPI_Freq, 2); // 2000dps || 2khz
            usleep(500);

            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_CONF3, PrivateConfig.ICM42605_SPI_Freq, 2); // bank0
            usleep(500);
            uint8_t ICM42605_SPI_Config_Tem[2] = {0x4e, 0x0f};
            _s_spiWrite(ICM42605_fd, ICM42605_SPI_Config_Tem, PrivateConfig.ICM42605_SPI_Freq, 2); // Accel on in LNM ||  Gyro on in LNM
            usleep(500);
        }
        else if (PrivateConfig.ICMType == ICMTypeI2C)
        {
        }
    }

    inline void ICMSensorsDataRead()
    {
        if (PrivateConfig.ICMType == ICMTypeI2C)
        {
            //
        }
        else if (PrivateConfig.ICMType == ICMTypeSPI)
        {
            PrivateData._uORB_ICM42605_IMUUpdateTime = GetTimestamp() - LastUpdate;
            LastUpdate = GetTimestamp();

            uint8_t Tmp_ICM42605_SPI_Buffer[8] = {0};
            uint8_t Tmp_ICM42605_SPI_Bufferout[8] = {0};
            Tmp_ICM42605_SPI_Buffer[0] = 0x9f;
            if (PrivateData._uORB_ICM42605_AccelCountDown >= (PrivateConfig.TargetFreqency / PrivateConfig.AccTargetFreqency))
            {
                _s_spiXfer(ICM42605_fd, Tmp_ICM42605_SPI_Buffer, Tmp_ICM42605_SPI_Bufferout, PrivateConfig.ICM42605_SPI_Freq, 8);
                int Tmp_AX = (short)((int)Tmp_ICM42605_SPI_Bufferout[1] << 8 | (int)Tmp_ICM42605_SPI_Bufferout[2]);
                int Tmp_AY = (short)((int)Tmp_ICM42605_SPI_Bufferout[3] << 8 | (int)Tmp_ICM42605_SPI_Bufferout[4]);
                int Tmp_AZ = (short)((int)Tmp_ICM42605_SPI_Bufferout[5] << 8 | (int)Tmp_ICM42605_SPI_Bufferout[6]);

                // Step 1: rotate Yaw
                int Tmp_A2X = Tmp_AX * cos(DEG2RAD((PrivateConfig.ICM_Flip___Yaw))) + Tmp_AY * sin(DEG2RAD((PrivateConfig.ICM_Flip___Yaw)));
                int Tmp_A2Y = Tmp_AY * cos(DEG2RAD((PrivateConfig.ICM_Flip___Yaw))) + Tmp_AX * sin(DEG2RAD((180 + PrivateConfig.ICM_Flip___Yaw)));

                // Step 2: rotate Pitch
                int Tmp_A3X = Tmp_A2X * cos(DEG2RAD(PrivateConfig.ICM_Flip_Pitch)) + Tmp_AZ * sin(DEG2RAD((PrivateConfig.ICM_Flip_Pitch)));
                int Tmp_A3Z = Tmp_AZ * cos(DEG2RAD((PrivateConfig.ICM_Flip_Pitch))) + Tmp_A2X * sin(DEG2RAD((180 + PrivateConfig.ICM_Flip_Pitch)));
                // Step 3: rotate Roll
                PrivateData._uORB_ICM42605_A_Y = Tmp_A2Y * cos(DEG2RAD((PrivateConfig.ICM_Flip__Roll))) + Tmp_A3Z * sin(DEG2RAD((180 + PrivateConfig.ICM_Flip__Roll)));
                PrivateData._uORB_ICM42605_A_Z = Tmp_A3Z * cos(DEG2RAD((PrivateConfig.ICM_Flip__Roll))) + Tmp_A2Y * sin(DEG2RAD((PrivateConfig.ICM_Flip__Roll)));
                PrivateData._uORB_ICM42605_A_X = Tmp_A3X;

                //
                PrivateData._uORB_ICM42605_AccelCountDown = 0;
            }
            PrivateData._uORB_ICM42605_AccelCountDown++;
            {
                uint8_t Tmp_ICM42605_SPI_GBuffer[8] = {0};
                uint8_t Tmp_ICM42605_SPI_GBufferout[8] = {0};
                Tmp_ICM42605_SPI_GBuffer[0] = 0xa5;
                _s_spiXfer(ICM42605_fd, Tmp_ICM42605_SPI_GBuffer, Tmp_ICM42605_SPI_GBufferout, PrivateConfig.ICM42605_SPI_Freq, 8);
                int Tmp_GX = (short)((int)Tmp_ICM42605_SPI_GBufferout[1] << 8 | (int)Tmp_ICM42605_SPI_GBufferout[2]);
                int Tmp_GY = (short)((int)Tmp_ICM42605_SPI_GBufferout[3] << 8 | (int)Tmp_ICM42605_SPI_GBufferout[4]);
                int Tmp_GZ = (short)((int)Tmp_ICM42605_SPI_GBufferout[5] << 8 | (int)Tmp_ICM42605_SPI_GBufferout[6]);
                // Step 1: rotate Yaw
                int Tmp_G2X = Tmp_GX * cos(DEG2RAD((PrivateConfig.ICM_Flip___Yaw))) + Tmp_GY * sin(DEG2RAD((PrivateConfig.ICM_Flip___Yaw)));
                int Tmp_G2Y = Tmp_GY * cos(DEG2RAD((PrivateConfig.ICM_Flip___Yaw))) + Tmp_GX * sin(DEG2RAD((180 + PrivateConfig.ICM_Flip___Yaw)));
                // Step 2: rotate Pitch
                int Tmp_G3X = Tmp_G2X * cos(DEG2RAD(PrivateConfig.ICM_Flip_Pitch)) + Tmp_GZ * sin(DEG2RAD((PrivateConfig.ICM_Flip_Pitch)));
                int Tmp_G3Z = Tmp_GZ * cos(DEG2RAD((PrivateConfig.ICM_Flip_Pitch))) + Tmp_G2X * sin(DEG2RAD((180 + PrivateConfig.ICM_Flip_Pitch)));
                // Step 3: rotate Roll
                PrivateData._uORB_ICM42605_G_Y = Tmp_G2Y * cos(DEG2RAD((PrivateConfig.ICM_Flip__Roll))) + Tmp_G3Z * sin(DEG2RAD((180 + PrivateConfig.ICM_Flip__Roll)));
                PrivateData._uORB_ICM42605_G_Z = Tmp_G3Z * cos(DEG2RAD((PrivateConfig.ICM_Flip__Roll))) + Tmp_G2Y * sin(DEG2RAD((PrivateConfig.ICM_Flip__Roll)));
                PrivateData._uORB_ICM42605_G_X = Tmp_G3X;
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
    float ICM42605_Accel_LSB = ICM42605_ACCEL_LSB; //+-16g
    int _Tmp_AHRS_MAG_X = 0;
    int _Tmp_AHRS_MAG_Y = 0;
    int _Tmp_AHRS_MAG_Z = 0;
    bool AHRSEnable = false;

    ICM4Data PrivateData;
    ICM4Config PrivateConfig;

    pt1Filter_t VibeLPF[3];
    pt1Filter_t VibeFloorLPF[3];
    pt1Filter_t GryoFilterLPF[3];
    pt1Filter_t GryoFilterLPFST2[3];
    pt1Filter_t AccelFilterLPF[3];
    biquadFilter_t GryoFilterBLPFST2[3];
    biquadFilter_t GryoFilterBLPF[3];
    biquadFilter_t AccelFilterBLPF[3];
    biquadFilter_t GryoFilterDynamicNotch[3];
    biquadFilter_t GryoFilterDynamicFreq[3];
    biquadFilter_t GyroNotchFilter[3];
    biquadFilter_t AccelNotchFilter[3];
    std::unique_ptr<MadgwickAHRS> AHRSSys;
};