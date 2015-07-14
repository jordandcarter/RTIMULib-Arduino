////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014-2015, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "RTIMUSettings.h"

#if defined(MPU9150_68) || defined(MPU9150_69)
#include "RTIMUMPU9150.h"
#endif

#if defined(MPU9250_68) || defined(MPU9250_69)
#include "RTIMUMPU9250.h"
#endif

#if defined(LSM9DS0_6a) || defined(LSM9DS0_6b)
#include "RTIMULSM9DS0.h"
#endif

#if defined(GD20HM303D_6a) || defined(GD20HM303D_6b)
#include "RTIMUGD20HM303D.h"
#endif

#if defined(GD20M303DLHC_6a) || defined(GD20M303DLHC_6b)
#include "RTIMUGD20M303DLHC.h"
#endif

#if defined(GD20HM303DLHC_6a) || defined(GD20HM303DLHC_6b)
#include "RTIMUGD20HM303DLHC.h"
#endif

#if defined(BNO055_28) || defined(BNO055_29)
#include "RTIMUBNO055.h"
#endif

#if defined(BMP180)
#include "RTPressureBMP180.h"
#endif

#if defined(LPS25H_5c) || defined (LPS25H_5d)
#include "RTPressureLPS25H.h"
#endif

#if defined(MS5611_76) || defined (MS5611_77)
#include "RTPressureMS5611.h"
#endif

#define RATE_TIMER_INTERVAL 2

RTIMUSettings::RTIMUSettings()
{
    //  preset general defaults

    m_imuType = -1;
    m_I2CSlaveAddress = 0;

#ifdef MPU9150_68
    //  MPU9150 defaults

    m_MPU9150GyroAccelSampleRate = 50;
    m_MPU9150CompassSampleRate = 25;
    m_MPU9150GyroAccelLpf = MPU9150_LPF_20;
    m_MPU9150GyroFsr = MPU9150_GYROFSR_1000;
    m_MPU9150AccelFsr = MPU9150_ACCELFSR_8;
    m_imuType = RTIMU_TYPE_MPU9150;
    m_I2CSlaveAddress = MPU9150_ADDRESS0;
#endif

#ifdef MPU9150_69
    //  MPU9150 defaults

    m_MPU9150GyroAccelSampleRate = 50;
    m_MPU9150CompassSampleRate = 25;
    m_MPU9150GyroAccelLpf = MPU9150_LPF_20;
    m_MPU9150GyroFsr = MPU9150_GYROFSR_1000;
    m_MPU9150AccelFsr = MPU9150_ACCELFSR_8;
    m_imuType = RTIMU_TYPE_MPU9150;
    m_I2CSlaveAddress = MPU9150_ADDRESS1;
#endif

#ifdef MPU9250_68
    //  MPU9250 defaults

    m_MPU9250GyroAccelSampleRate = 80;
    m_MPU9250CompassSampleRate = 40;
    m_MPU9250GyroLpf = MPU9250_GYRO_LPF_41;
    m_MPU9250AccelLpf = MPU9250_ACCEL_LPF_41;
    m_MPU9250GyroFsr = MPU9250_GYROFSR_1000;
    m_MPU9250AccelFsr = MPU9250_ACCELFSR_8;
    m_I2CSlaveAddress = MPU9250_ADDRESS0;
#endif

#ifdef MPU9250_69
    //  MPU9250 defaults

    m_MPU9250GyroAccelSampleRate = 80;
    m_MPU9250CompassSampleRate = 40;
    m_MPU9250GyroLpf = MPU9250_GYRO_LPF_41;
    m_MPU9250AccelLpf = MPU9250_ACCEL_LPF_41;
    m_MPU9250GyroFsr = MPU9250_GYROFSR_1000;
    m_MPU9250AccelFsr = MPU9250_ACCELFSR_8;
    m_I2CSlaveAddress = MPU9250_ADDRESS1;
#endif

#ifdef LSM9DS0_6a
    //  LSM9DS0 defaults

    m_LSM9DS0GyroSampleRate = LSM9DS0_GYRO_SAMPLERATE_95;
    m_LSM9DS0GyroBW = LSM9DS0_GYRO_BANDWIDTH_1;
    m_LSM9DS0GyroHpf = LSM9DS0_GYRO_HPF_4;
    m_LSM9DS0GyroFsr = LSM9DS0_GYRO_FSR_500;

    m_LSM9DS0AccelSampleRate = LSM9DS0_ACCEL_SAMPLERATE_50;
    m_LSM9DS0AccelFsr = LSM9DS0_ACCEL_FSR_8;
    m_LSM9DS0AccelLpf = LSM9DS0_ACCEL_LPF_50;

    m_LSM9DS0CompassSampleRate = LSM9DS0_COMPASS_SAMPLERATE_50;
    m_LSM9DS0CompassFsr = LSM9DS0_COMPASS_FSR_2;

    m_imuType = RTIMU_TYPE_LSM9DS0;
    m_I2CSlaveAddress = LSM9DS0_GYRO_ADDRESS0;

#endif

#ifdef LSM9DS0_6b
    //  LSM9DS0 defaults

    m_LSM9DS0GyroSampleRate = LSM9DS0_GYRO_SAMPLERATE_95;
    m_LSM9DS0GyroBW = LSM9DS0_GYRO_BANDWIDTH_1;
    m_LSM9DS0GyroHpf = LSM9DS0_GYRO_HPF_4;
    m_LSM9DS0GyroFsr = LSM9DS0_GYRO_FSR_500;

    m_LSM9DS0AccelSampleRate = LSM9DS0_ACCEL_SAMPLERATE_50;
    m_LSM9DS0AccelFsr = LSM9DS0_ACCEL_FSR_8;
    m_LSM9DS0AccelLpf = LSM9DS0_ACCEL_LPF_50;

    m_LSM9DS0CompassSampleRate = LSM9DS0_COMPASS_SAMPLERATE_50;
    m_LSM9DS0CompassFsr = LSM9DS0_COMPASS_FSR_2;

    m_imuType = RTIMU_TYPE_LSM9DS0;
    m_I2CSlaveAddress = LSM9DS0_GYRO_ADDRESS1;

#endif

#ifdef GD20HM303D_6a
    //  GD20HM303D defaults

    m_GD20HM303DGyroSampleRate = L3GD20H_SAMPLERATE_50;
    m_GD20HM303DGyroBW = L3GD20H_BANDWIDTH_1;
    m_GD20HM303DGyroHpf = L3GD20H_HPF_4;
    m_GD20HM303DGyroFsr = L3GD20H_FSR_500;

    m_GD20HM303DAccelSampleRate = LSM303D_ACCEL_SAMPLERATE_50;
    m_GD20HM303DAccelFsr = LSM303D_ACCEL_FSR_8;
    m_GD20HM303DAccelLpf = LSM303D_ACCEL_LPF_50;

    m_GD20HM303DCompassSampleRate = LSM303D_COMPASS_SAMPLERATE_50;
    m_GD20HM303DCompassFsr = LSM303D_COMPASS_FSR_2;

    m_imuType = RTIMU_TYPE_GD20HM303D;
    m_I2CSlaveAddress = L3GD20H_ADDRESS0;
#endif

#ifdef GD20HM303D_6b
    //  GD20HM303D defaults

    m_GD20HM303DGyroSampleRate = L3GD20H_SAMPLERATE_50;
    m_GD20HM303DGyroBW = L3GD20H_BANDWIDTH_1;
    m_GD20HM303DGyroHpf = L3GD20H_HPF_4;
    m_GD20HM303DGyroFsr = L3GD20H_FSR_500;

    m_GD20HM303DAccelSampleRate = LSM303D_ACCEL_SAMPLERATE_50;
    m_GD20HM303DAccelFsr = LSM303D_ACCEL_FSR_8;
    m_GD20HM303DAccelLpf = LSM303D_ACCEL_LPF_50;

    m_GD20HM303DCompassSampleRate = LSM303D_COMPASS_SAMPLERATE_50;
    m_GD20HM303DCompassFsr = LSM303D_COMPASS_FSR_2;

    m_imuType = RTIMU_TYPE_GD20HM303D;
    m_I2CSlaveAddress = L3GD20H_ADDRESS1;
#endif

#ifdef GD20M303DLHC_6a
    //  GD20M303DLHC defaults

    m_GD20M303DLHCGyroSampleRate = L3GD20_SAMPLERATE_95;
    m_GD20M303DLHCGyroBW = L3GD20_BANDWIDTH_1;
    m_GD20M303DLHCGyroHpf = L3GD20_HPF_4;
    m_GD20M303DLHCGyroFsr = L3GD20_FSR_500;

    m_GD20M303DLHCAccelSampleRate = LSM303DLHC_ACCEL_SAMPLERATE_50;
    m_GD20M303DLHCAccelFsr = LSM303DLHC_ACCEL_FSR_8;

    m_GD20M303DLHCCompassSampleRate = LSM303DLHC_COMPASS_SAMPLERATE_30;
    m_GD20M303DLHCCompassFsr = LSM303DLHC_COMPASS_FSR_1_3;

    m_imuType = RTIMU_TYPE_GD20M303DLHC;
    m_I2CSlaveAddress = L3GD20_ADDRESS0;
#endif

#ifdef GD20M303DLHC_6b
    //  GD20M303DLHC defaults

    m_GD20M303DLHCGyroSampleRate = L3GD20_SAMPLERATE_95;
    m_GD20M303DLHCGyroBW = L3GD20_BANDWIDTH_1;
    m_GD20M303DLHCGyroHpf = L3GD20_HPF_4;
    m_GD20M303DLHCGyroFsr = L3GD20_FSR_500;

    m_GD20M303DLHCAccelSampleRate = LSM303DLHC_ACCEL_SAMPLERATE_50;
    m_GD20M303DLHCAccelFsr = LSM303DLHC_ACCEL_FSR_8;

    m_GD20M303DLHCCompassSampleRate = LSM303DLHC_COMPASS_SAMPLERATE_30;
    m_GD20M303DLHCCompassFsr = LSM303DLHC_COMPASS_FSR_1_3;

    m_imuType = RTIMU_TYPE_GD20M303DLHC;
    m_I2CSlaveAddress = L3GD20_ADDRESS1;
#endif

#ifdef GD20HM303DLHC_6a
    //  GD20HM303DLHC defaults

    m_GD20HM303DLHCGyroSampleRate = L3GD20H_SAMPLERATE_50;
    m_GD20HM303DLHCGyroBW = L3GD20H_BANDWIDTH_1;
    m_GD20HM303DLHCGyroHpf = L3GD20H_HPF_4;
    m_GD20HM303DLHCGyroFsr = L3GD20H_FSR_500;

    m_GD20HM303DLHCAccelSampleRate = LSM303DLHC_ACCEL_SAMPLERATE_50;
    m_GD20HM303DLHCAccelFsr = LSM303DLHC_ACCEL_FSR_8;

    m_GD20HM303DLHCCompassSampleRate = LSM303DLHC_COMPASS_SAMPLERATE_30;
    m_GD20HM303DLHCCompassFsr = LSM303DLHC_COMPASS_FSR_1_3;

    m_imuType = RTIMU_TYPE_GD20HM303DLHC;
    m_I2CSlaveAddress = L3GD20H_ADDRESS0;
#endif

#ifdef GD20HM303DLHC_6b
    //  GD20M303DLHC defaults

    m_GD20HM303DLHCGyroSampleRate = L3GD20H_SAMPLERATE_50;
    m_GD20HM303DLHCGyroBW = L3GD20H_BANDWIDTH_1;
    m_GD20HM303DLHCGyroHpf = L3GD20H_HPF_4;
    m_GD20HM303DLHCGyroFsr = L3GD20H_FSR_500;

    m_GD20HM303DLHCAccelSampleRate = LSM303DLHC_ACCEL_SAMPLERATE_50;
    m_GD20HM303DLHCAccelFsr = LSM303DLHC_ACCEL_FSR_8;

    m_GD20HM303DLHCCompassSampleRate = LSM303DLHC_COMPASS_SAMPLERATE_30;
    m_GD20HM303DLHCCompassFsr = LSM303DLHC_COMPASS_FSR_1_3;

    m_imuType = RTIMU_TYPE_GD20HM303DLHC;
    m_I2CSlaveAddress = L3GD20H_ADDRESS1;
#endif

#ifdef BNO055_28
    m_imuType = RTIMU_TYPE_BNO055;
    m_I2CSlaveAddress = BNO055_ADDRESS0;
#endif

#ifdef BNO055_29
    m_imuType = RTIMU_TYPE_BNO055;
    m_I2CSlaveAddress = BNO055_ADDRESS1;
#endif

#ifdef BMP180
    m_pressureType = RTPRESSURE_TYPE_BMP180;
    m_I2CPressureAddress = BMP180_ADDRESS;
#endif

#ifdef LPS25H_5c
    m_pressureType = RTPRESSURE_TYPE_LPS25H;
    m_I2CPressureAddress = LPS25H_ADDRESS0;
#endif

#ifdef LPS25H_5d
    m_pressureType = RTPRESSURE_TYPE_LPS25H;
    m_I2CPressureAddress = LPS25H_ADDRESS1;
#endif

#ifdef MS5611_76
    m_pressureType = RTPRESSURE_TYPE_MS5611;
    m_I2CPressureAddress = MS5611_ADDRESS0;
#endif

#ifdef MS5611_77
    m_pressureType = RTPRESSURE_TYPE_MS5611;
    m_I2CPressureAddress = MS5611_ADDRESS1;
#endif

}
