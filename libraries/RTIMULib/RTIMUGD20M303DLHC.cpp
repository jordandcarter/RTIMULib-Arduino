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

#include "RTIMUGD20M303DLHC.h"
#include "RTIMUSettings.h"

#if defined(GD20M303DLHC_6a) || defined(GD20M303DLHC_6b)

RTIMUGD20M303DLHC::RTIMUGD20M303DLHC(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 100;
}

RTIMUGD20M303DLHC::~RTIMUGD20M303DLHC()
{
}

int RTIMUGD20M303DLHC::IMUInit()
{
    unsigned char result;

    //  configure IMU

    m_gyroSlaveAddr = m_settings->m_I2CSlaveAddress;
    m_accelSlaveAddr = LSM303DLHC_ACCEL_ADDRESS;
    m_compassSlaveAddr = LSM303DLHC_COMPASS_ADDRESS;

    setCalibrationData();

    //  Set up the gyro

    if (!I2CWrite(m_gyroSlaveAddr, L3GD20_CTRL5, 0x80))
        return -1;

    if (!I2CRead(m_gyroSlaveAddr, L3GD20_WHO_AM_I, 1, &result))
        return -2;

    if (result != L3GD20_ID) {
        return -3;
    }

    if (!setGyroSampleRate())
            return -4;

    if (!setGyroCTRL2())
            return -5;

    if (!setGyroCTRL4())
            return -6;

    //  Set up the accel

    if (!setAccelCTRL1())
        return -7;

    if (!setAccelCTRL4())
        return -8;

    //  Set up the compass

    if (!setCompassCRA())
        return -9;

    if (!setCompassCRB())
        return -10;

    if (!setCompassCRM())
        return -11;

    if (!setGyroCTRL5())
            return -12;

    gyroBiasInit();

    return true;
}

bool RTIMUGD20M303DLHC::setGyroSampleRate()
{
    unsigned char ctrl1;

    switch (m_settings->m_GD20M303DLHCGyroSampleRate) {
    case L3GD20_SAMPLERATE_95:
        ctrl1 = 0x0f;
        m_sampleRate = 95;
        break;

    case L3GD20_SAMPLERATE_190:
        ctrl1 = 0x4f;
        m_sampleRate = 190;
        break;

    case L3GD20_SAMPLERATE_380:
        ctrl1 = 0x8f;
        m_sampleRate = 380;
        break;

    case L3GD20_SAMPLERATE_760:
        ctrl1 = 0xcf;
        m_sampleRate = 760;
        break;

    default:
        return false;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;

    switch (m_settings->m_GD20M303DLHCGyroBW) {
    case L3GD20_BANDWIDTH_0:
        ctrl1 |= 0x00;
        break;

    case L3GD20_BANDWIDTH_1:
        ctrl1 |= 0x10;
        break;

    case L3GD20_BANDWIDTH_2:
        ctrl1 |= 0x20;
        break;

    case L3GD20_BANDWIDTH_3:
        ctrl1 |= 0x30;
        break;

    }

    return (I2CWrite(m_gyroSlaveAddr, L3GD20_CTRL1, ctrl1));
}

bool RTIMUGD20M303DLHC::setGyroCTRL2()
{
    if ((m_settings->m_GD20M303DLHCGyroHpf < L3GD20_HPF_0) || (m_settings->m_GD20M303DLHCGyroHpf > L3GD20_HPF_9)) {
        return false;
    }
    return I2CWrite(m_gyroSlaveAddr,  L3GD20_CTRL2, m_settings->m_GD20M303DLHCGyroHpf);
}

bool RTIMUGD20M303DLHC::setGyroCTRL4()
{
    unsigned char ctrl4;

    switch (m_settings->m_GD20M303DLHCGyroFsr) {
    case L3GD20_FSR_250:
        ctrl4 = 0x00;
        m_gyroScale = (RTFLOAT)0.00875 * RTMATH_DEGREE_TO_RAD;
        break;

    case L3GD20_FSR_500:
        ctrl4 = 0x10;
        m_gyroScale = (RTFLOAT)0.0175 * RTMATH_DEGREE_TO_RAD;
        break;

    case L3GD20_FSR_2000:
        ctrl4 = 0x20;
        m_gyroScale = (RTFLOAT)0.07 * RTMATH_DEGREE_TO_RAD;
        break;

    default:
        return false;
    }

    return I2CWrite(m_gyroSlaveAddr,  L3GD20_CTRL4, ctrl4);
}


bool RTIMUGD20M303DLHC::setGyroCTRL5()
{
    unsigned char ctrl5;

    //  Turn on hpf

    ctrl5 = 0x10;

#ifdef GD20M303DLHC_CACHE_MODE
    //  turn on fifo

    ctrl5 |= 0x40;
#endif

    return I2CWrite(m_gyroSlaveAddr,  L3GD20_CTRL5, ctrl5);
}


bool RTIMUGD20M303DLHC::setAccelCTRL1()
{
    unsigned char ctrl1;

    if ((m_settings->m_GD20M303DLHCAccelSampleRate < 1) || (m_settings->m_GD20M303DLHCAccelSampleRate > 7)) {
        return false;
    }

    ctrl1 = (m_settings->m_GD20M303DLHCAccelSampleRate << 4) | 0x07;

    return I2CWrite(m_accelSlaveAddr,  LSM303DLHC_CTRL1_A, ctrl1);
}

bool RTIMUGD20M303DLHC::setAccelCTRL4()
{
    unsigned char ctrl4;

    switch (m_settings->m_GD20M303DLHCAccelFsr) {
    case LSM303DLHC_ACCEL_FSR_2:
        m_accelScale = (RTFLOAT)0.001 / (RTFLOAT)64;
        break;

    case LSM303DLHC_ACCEL_FSR_4:
        m_accelScale = (RTFLOAT)0.002 / (RTFLOAT)64;
        break;

    case LSM303DLHC_ACCEL_FSR_8:
        m_accelScale = (RTFLOAT)0.004 / (RTFLOAT)64;
        break;

    case LSM303DLHC_ACCEL_FSR_16:
        m_accelScale = (RTFLOAT)0.012 / (RTFLOAT)64;
        break;

    default:
        return false;
    }

    ctrl4 = (m_settings->m_GD20M303DLHCAccelFsr << 4);

    return I2CWrite(m_accelSlaveAddr,  LSM303DLHC_CTRL2_A, ctrl4);
}


bool RTIMUGD20M303DLHC::setCompassCRA()
{
    unsigned char cra;

    if ((m_settings->m_GD20M303DLHCCompassSampleRate < 0) || (m_settings->m_GD20M303DLHCCompassSampleRate > 7)) {
        return false;
    }

    cra = (m_settings->m_GD20M303DLHCCompassSampleRate << 2);

    return I2CWrite(m_compassSlaveAddr,  LSM303DLHC_CRA_M, cra);
}

bool RTIMUGD20M303DLHC::setCompassCRB()
{
    unsigned char crb;

    //  convert FSR to uT

    switch (m_settings->m_GD20M303DLHCCompassFsr) {
    case LSM303DLHC_COMPASS_FSR_1_3:
        crb = 0x20;
        m_compassScaleXY = (RTFLOAT)100 / (RTFLOAT)1100;
        m_compassScaleZ = (RTFLOAT)100 / (RTFLOAT)980;
        break;

    case LSM303DLHC_COMPASS_FSR_1_9:
        crb = 0x40;
        m_compassScaleXY = (RTFLOAT)100 / (RTFLOAT)855;
        m_compassScaleZ = (RTFLOAT)100 / (RTFLOAT)760;
       break;

    case LSM303DLHC_COMPASS_FSR_2_5:
        crb = 0x60;
        m_compassScaleXY = (RTFLOAT)100 / (RTFLOAT)670;
        m_compassScaleZ = (RTFLOAT)100 / (RTFLOAT)600;
        break;

    case LSM303DLHC_COMPASS_FSR_4:
        crb = 0x80;
        m_compassScaleXY = (RTFLOAT)100 / (RTFLOAT)450;
        m_compassScaleZ = (RTFLOAT)100 / (RTFLOAT)400;
        break;

    case LSM303DLHC_COMPASS_FSR_4_7:
        crb = 0xa0;
        m_compassScaleXY = (RTFLOAT)100 / (RTFLOAT)400;
        m_compassScaleZ = (RTFLOAT)100 / (RTFLOAT)355;
        break;

    case LSM303DLHC_COMPASS_FSR_5_6:
        crb = 0xc0;
        m_compassScaleXY = (RTFLOAT)100 / (RTFLOAT)330;
        m_compassScaleZ = (RTFLOAT)100 / (RTFLOAT)295;
        break;

    case LSM303DLHC_COMPASS_FSR_8_1:
        crb = 0xe0;
        m_compassScaleXY = (RTFLOAT)100 / (RTFLOAT)230;
        m_compassScaleZ = (RTFLOAT)100 / (RTFLOAT)205;
        break;

    default:
        return false;
    }

    return I2CWrite(m_compassSlaveAddr,  LSM303DLHC_CRB_M, crb);
}

bool RTIMUGD20M303DLHC::setCompassCRM()
{
     return I2CWrite(m_compassSlaveAddr,  LSM303DLHC_CRM_M, 0x00);
}


int RTIMUGD20M303DLHC::IMUGetPollInterval()
{
    return (400 / m_sampleRate);
}

bool RTIMUGD20M303DLHC::IMURead()
{
    unsigned char status;
    unsigned char gyroData[6];
    unsigned char accelData[6];
    unsigned char compassData[6];

    if (!I2CRead(m_gyroSlaveAddr, L3GD20_STATUS, 1, &status))
        return false;

    if ((status & 0x8) == 0)
        return false;

    if (!I2CRead(m_gyroSlaveAddr, 0x80 | L3GD20_OUT_X_L, 6, gyroData))
        return false;

    m_timestamp = millis();

    if (!I2CRead(m_accelSlaveAddr, 0x80 | LSM303DLHC_OUT_X_L_A, 6, accelData))
        return false;

    if (!I2CRead(m_compassSlaveAddr, 0x80 | LSM303DLHC_OUT_X_H_M, 6, compassData))
        return false;

    RTMath::convertToVector(gyroData, m_gyro, m_gyroScale, false);
    RTMath::convertToVector(accelData, m_accel, m_accelScale, false);

    m_compass.setX((RTFLOAT)((int16_t)(((uint16_t)compassData[0] << 8) | (uint16_t)compassData[1])) * m_compassScaleXY);
    m_compass.setY((RTFLOAT)((int16_t)(((uint16_t)compassData[2] << 8) | (uint16_t)compassData[3])) * m_compassScaleXY);
    m_compass.setZ((RTFLOAT)((int16_t)(((uint16_t)compassData[4] << 8) | (uint16_t)compassData[5])) * m_compassScaleZ);

    //  sort out gyro axes

    m_gyro.setY(-m_gyro.y());
    m_gyro.setZ(-m_gyro.z());

    //  sort out accel data;

    m_accel.setX(-m_accel.x());

    //  sort out compass axes

    RTFLOAT temp;

    temp = m_compass.z();
    m_compass.setZ(-m_compass.y());
    m_compass.setY(-temp);

    //  now do standard processing

    handleGyroBias();
    calibrateAverageCompass();

    return true;
}
#endif