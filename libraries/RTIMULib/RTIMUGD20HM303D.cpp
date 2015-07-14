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

#include "RTIMUGD20HM303D.h"
#include "RTIMUSettings.h"

#if defined(GD20HM303D_6a) || defined(GD20HM303D_6b)

RTIMUGD20HM303D::RTIMUGD20HM303D(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 100;
}

RTIMUGD20HM303D::~RTIMUGD20HM303D()
{
}

int RTIMUGD20HM303D::IMUInit()
{
    unsigned char result;

    //  configure IMU

    m_gyroSlaveAddr = m_settings->m_I2CSlaveAddress;
    if (m_gyroSlaveAddr == L3GD20H_ADDRESS0)
        m_accelCompassSlaveAddr = LSM303D_ADDRESS0;
    else
        m_accelCompassSlaveAddr = LSM303D_ADDRESS1;

    setCalibrationData();

    //  Set up the gyro

    if (!I2CWrite(m_gyroSlaveAddr, L3GD20H_LOW_ODR, 0x04))
        return -1;

    if (!I2CWrite(m_gyroSlaveAddr, L3GD20H_CTRL5, 0x80))
        return -2;

    if (!I2CRead(m_gyroSlaveAddr, L3GD20H_WHO_AM_I, 1, &result))
        return -3;

    if (result != L3GD20H_ID) {
        return -4;
    }

    if (!setGyroSampleRate())
            return -5;

    if (!setGyroCTRL2())
            return -6;

    if (!setGyroCTRL4())
            return -7;

    //  Set up the accel/compass

    if (!I2CRead(m_accelCompassSlaveAddr, LSM303D_WHO_AM_I, 1, &result))
        return -8;

    if (result != LSM303D_ID) {
        return -9;
    }

    if (!setAccelCTRL1())
        return -10;

    if (!setAccelCTRL2())
        return -11;

    if (!setCompassCTRL5())
        return -12;

    if (!setCompassCTRL6())
        return -13;

    if (!setCompassCTRL7())
        return -14;

    if (!setGyroCTRL5())
            return -16;

    gyroBiasInit();

    return true;
}

bool RTIMUGD20HM303D::setGyroSampleRate()
{
    unsigned char ctrl1;
    unsigned char lowOdr = 0;

    switch (m_settings->m_GD20HM303DGyroSampleRate) {
    case L3GD20H_SAMPLERATE_12_5:
        ctrl1 = 0x0f;
        lowOdr = 1;
        m_sampleRate = 13;
        break;

    case L3GD20H_SAMPLERATE_25:
        ctrl1 = 0x4f;
        lowOdr = 1;
        m_sampleRate = 25;
        break;

    case L3GD20H_SAMPLERATE_50:
        ctrl1 = 0x8f;
        lowOdr = 1;
        m_sampleRate = 50;
        break;

    case L3GD20H_SAMPLERATE_100:
        ctrl1 = 0x0f;
        m_sampleRate = 100;
        break;

    case L3GD20H_SAMPLERATE_200:
        ctrl1 = 0x4f;
        m_sampleRate = 200;
        break;

    case L3GD20H_SAMPLERATE_400:
        ctrl1 = 0x8f;
        m_sampleRate = 400;
        break;

    case L3GD20H_SAMPLERATE_800:
        ctrl1 = 0xcf;
        m_sampleRate = 800;
        break;

    default:
        return false;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;

    switch (m_settings->m_GD20HM303DGyroBW) {
    case L3GD20H_BANDWIDTH_0:
        ctrl1 |= 0x00;
        break;

    case L3GD20H_BANDWIDTH_1:
        ctrl1 |= 0x10;
        break;

    case L3GD20H_BANDWIDTH_2:
        ctrl1 |= 0x20;
        break;

    case L3GD20H_BANDWIDTH_3:
        ctrl1 |= 0x30;
        break;

    }

    if (!I2CWrite(m_gyroSlaveAddr, L3GD20H_LOW_ODR, lowOdr))
        return false;

    return (I2CWrite(m_gyroSlaveAddr, L3GD20H_CTRL1, ctrl1));
}

bool RTIMUGD20HM303D::setGyroCTRL2()
{
    if ((m_settings->m_GD20HM303DGyroHpf < L3GD20H_HPF_0) || (m_settings->m_GD20HM303DGyroHpf > L3GD20H_HPF_9)) {
        return false;
    }
    return I2CWrite(m_gyroSlaveAddr,  L3GD20H_CTRL2, m_settings->m_GD20HM303DGyroHpf);
}

bool RTIMUGD20HM303D::setGyroCTRL4()
{
    unsigned char ctrl4;

    switch (m_settings->m_GD20HM303DGyroFsr) {
    case L3GD20H_FSR_245:
        ctrl4 = 0x00;
        m_gyroScale = (RTFLOAT)0.00875 * RTMATH_DEGREE_TO_RAD;
        break;

    case L3GD20H_FSR_500:
        ctrl4 = 0x10;
        m_gyroScale = (RTFLOAT)0.0175 * RTMATH_DEGREE_TO_RAD;
        break;

    case L3GD20H_FSR_2000:
        ctrl4 = 0x20;
        m_gyroScale = (RTFLOAT)0.07 * RTMATH_DEGREE_TO_RAD;
        break;

    default:
        return false;
    }

    return I2CWrite(m_gyroSlaveAddr,  L3GD20H_CTRL4, ctrl4);
}


bool RTIMUGD20HM303D::setGyroCTRL5()
{
    unsigned char ctrl5;

    //  Turn on hpf

    ctrl5 = 0x10;

#ifdef GD20HM303D_CACHE_MODE
    //  turn on fifo

    ctrl5 |= 0x40;
#endif

    return I2CWrite(m_gyroSlaveAddr,  L3GD20H_CTRL5, ctrl5);
}


bool RTIMUGD20HM303D::setAccelCTRL1()
{
    unsigned char ctrl1;

    if ((m_settings->m_GD20HM303DAccelSampleRate < 0) || (m_settings->m_GD20HM303DAccelSampleRate > 10)) {
        return false;
    }

    ctrl1 = (m_settings->m_GD20HM303DAccelSampleRate << 4) | 0x07;

    return I2CWrite(m_accelCompassSlaveAddr,  LSM303D_CTRL1, ctrl1);
}

bool RTIMUGD20HM303D::setAccelCTRL2()
{
    unsigned char ctrl2;

    if ((m_settings->m_GD20HM303DAccelLpf < 0) || (m_settings->m_GD20HM303DAccelLpf > 3)) {
        return false;
    }

    switch (m_settings->m_GD20HM303DAccelFsr) {
    case LSM303D_ACCEL_FSR_2:
        m_accelScale = (RTFLOAT)0.000061;
        break;

    case LSM303D_ACCEL_FSR_4:
        m_accelScale = (RTFLOAT)0.000122;
        break;

    case LSM303D_ACCEL_FSR_6:
        m_accelScale = (RTFLOAT)0.000183;
        break;

    case LSM303D_ACCEL_FSR_8:
        m_accelScale = (RTFLOAT)0.000244;
        break;

    case LSM303D_ACCEL_FSR_16:
        m_accelScale = (RTFLOAT)0.000732;
        break;

    default:
        return false;
    }

    ctrl2 = (m_settings->m_GD20HM303DAccelLpf << 6) | (m_settings->m_GD20HM303DAccelFsr << 3);

    return I2CWrite(m_accelCompassSlaveAddr,  LSM303D_CTRL2, ctrl2);
}


bool RTIMUGD20HM303D::setCompassCTRL5()
{
    unsigned char ctrl5;

    if ((m_settings->m_GD20HM303DCompassSampleRate < 0) || (m_settings->m_GD20HM303DCompassSampleRate > 5)) {
        return false;
    }

    ctrl5 = (m_settings->m_GD20HM303DCompassSampleRate << 2);

#ifdef GD20HM303D_CACHE_MODE
    //  enable fifo

    ctrl5 |= 0x40;
#endif

    return I2CWrite(m_accelCompassSlaveAddr,  LSM303D_CTRL5, ctrl5);
}

bool RTIMUGD20HM303D::setCompassCTRL6()
{
    unsigned char ctrl6;

    //  convert FSR to uT

    switch (m_settings->m_GD20HM303DCompassFsr) {
    case LSM303D_COMPASS_FSR_2:
        ctrl6 = 0;
        m_compassScale = (RTFLOAT)0.008;
        break;

    case LSM303D_COMPASS_FSR_4:
        ctrl6 = 0x20;
        m_compassScale = (RTFLOAT)0.016;
        break;

    case LSM303D_COMPASS_FSR_8:
        ctrl6 = 0x40;
        m_compassScale = (RTFLOAT)0.032;
        break;

    case LSM303D_COMPASS_FSR_12:
        ctrl6 = 0x60;
        m_compassScale = (RTFLOAT)0.0479;
        break;

    default:
        return false;
    }

    return I2CWrite(m_accelCompassSlaveAddr,  LSM303D_CTRL6, ctrl6);
}

bool RTIMUGD20HM303D::setCompassCTRL7()
{
     return I2CWrite(m_accelCompassSlaveAddr,  LSM303D_CTRL7, 0x60);
}


int RTIMUGD20HM303D::IMUGetPollInterval()
{
    return (400 / m_sampleRate);
}

bool RTIMUGD20HM303D::IMURead()
{
    unsigned char status;
    unsigned char gyroData[6];
    unsigned char accelData[6];
    unsigned char compassData[6];

    if (!I2CRead(m_gyroSlaveAddr, L3GD20H_STATUS, 1, &status))
        return false;

    if ((status & 0x8) == 0)
        return false;

    if (!I2CRead(m_gyroSlaveAddr, 0x80 | L3GD20H_OUT_X_L, 6, gyroData))
        return false;

    m_timestamp = millis();

    if (!I2CRead(m_accelCompassSlaveAddr, 0x80 | LSM303D_OUT_X_L_A, 6, accelData))
        return false;

    if (!I2CRead(m_accelCompassSlaveAddr, 0x80 | LSM303D_OUT_X_L_M, 6, compassData))
        return false;

    RTMath::convertToVector(gyroData, m_gyro, m_gyroScale, false);
    RTMath::convertToVector(accelData, m_accel, m_accelScale, false);
    RTMath::convertToVector(compassData, m_compass, m_compassScale, false);

    //  sort out gyro axes

    m_gyro.setY(-m_gyro.y());
    m_gyro.setZ(-m_gyro.z());

    //  sort out accel data;

    m_accel.setX(-m_accel.x());

    //  sort out compass axes

    m_compass.setY(-m_compass.y());
    m_compass.setZ(-m_compass.z());

    //  now do standard processing

    handleGyroBias();
    calibrateAverageCompass();

    return true;
}
#endif