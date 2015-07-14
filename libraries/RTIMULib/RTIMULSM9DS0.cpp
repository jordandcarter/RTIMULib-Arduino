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

#include "RTIMULSM9DS0.h"
#include "RTIMUSettings.h"

#if defined(LSM9DS0_6a) || defined(LSM9DS0_6b)


RTIMULSM9DS0::RTIMULSM9DS0(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 100;
}

RTIMULSM9DS0::~RTIMULSM9DS0()
{
}

int RTIMULSM9DS0::IMUInit()
{
    unsigned char result;

    //  configure IMU

    m_gyroSlaveAddr = m_settings->m_I2CSlaveAddress;
    if (m_gyroSlaveAddr == LSM9DS0_GYRO_ADDRESS0)
        m_accelCompassSlaveAddr = LSM9DS0_ACCELMAG_ADDRESS0;
    else
        m_accelCompassSlaveAddr = LSM9DS0_ACCELMAG_ADDRESS1;

    setCalibrationData();

    //  Set up the gyro

    if (!I2Cdev::writeByte(m_gyroSlaveAddr, LSM9DS0_GYRO_CTRL5, 0x80))
        return -1;

    if (!I2Cdev::readByte(m_gyroSlaveAddr, LSM9DS0_GYRO_WHO_AM_I, &result))
        return -2;

    if (result != LSM9DS0_GYRO_ID) {
        return -3;
    }

    if (!setGyroSampleRate())
            return -4;

    if (!setGyroCTRL2())
            return -5;

    if (!setGyroCTRL4())
            return -6;

    //  Set up the accel

    if (!I2Cdev::readByte(m_accelCompassSlaveAddr, LSM9DS0_WHO_AM_I, &result))
        return -7;

    if (result != LSM9DS0_ACCELMAG_ID) {
        return -8;
    }

    if (!setAccelCTRL1())
        return 9;

    if (!setAccelCTRL2())
        return -10;

    if (!setCompassCTRL5())
        return 11;

    if (!setCompassCTRL6())
        return -12;

    if (!setCompassCTRL7())
        return -13;

    if (!setGyroCTRL5())
            return -14;

    gyroBiasInit();
    return 1;
}

bool RTIMULSM9DS0::setGyroSampleRate()
{
    unsigned char ctrl1;

    switch (m_settings->m_LSM9DS0GyroSampleRate) {
    case LSM9DS0_GYRO_SAMPLERATE_95:
        ctrl1 = 0x0f;
        m_sampleRate = 95;
        break;

    case LSM9DS0_GYRO_SAMPLERATE_190:
        ctrl1 = 0x4f;
        m_sampleRate = 190;
        break;

    case LSM9DS0_GYRO_SAMPLERATE_380:
        ctrl1 = 0x8f;
        m_sampleRate = 380;
        break;

    case LSM9DS0_GYRO_SAMPLERATE_760:
        ctrl1 = 0xcf;
        m_sampleRate = 760;
        break;

    default:
         return false;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;

    switch (m_settings->m_LSM9DS0GyroBW) {
    case LSM9DS0_GYRO_BANDWIDTH_0:
        ctrl1 |= 0x00;
        break;

    case LSM9DS0_GYRO_BANDWIDTH_1:
        ctrl1 |= 0x10;
        break;

    case LSM9DS0_GYRO_BANDWIDTH_2:
        ctrl1 |= 0x20;
        break;

    case LSM9DS0_GYRO_BANDWIDTH_3:
        ctrl1 |= 0x30;
        break;

    }

    return (I2Cdev::writeByte(m_gyroSlaveAddr, LSM9DS0_GYRO_CTRL1, ctrl1));
}

bool RTIMULSM9DS0::setGyroCTRL2()
{
    if ((m_settings->m_LSM9DS0GyroHpf < LSM9DS0_GYRO_HPF_0) || (m_settings->m_LSM9DS0GyroHpf > LSM9DS0_GYRO_HPF_9)) {
        return false;
    }
    return I2Cdev::writeByte(m_gyroSlaveAddr,  LSM9DS0_GYRO_CTRL2, m_settings->m_LSM9DS0GyroHpf);
}

bool RTIMULSM9DS0::setGyroCTRL4()
{
    unsigned char ctrl4;

    switch (m_settings->m_LSM9DS0GyroFsr) {
    case LSM9DS0_GYRO_FSR_250:
        ctrl4 = 0x00;
        m_gyroScale = (RTFLOAT)0.00875 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM9DS0_GYRO_FSR_500:
        ctrl4 = 0x10;
        m_gyroScale = (RTFLOAT)0.0175 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM9DS0_GYRO_FSR_2000:
        ctrl4 = 0x20;
        m_gyroScale = (RTFLOAT)0.07 * RTMATH_DEGREE_TO_RAD;
        break;

    default:
        return false;
    }

    return I2Cdev::writeByte(m_gyroSlaveAddr,  LSM9DS0_GYRO_CTRL4, ctrl4);
}


bool RTIMULSM9DS0::setGyroCTRL5()
{
    unsigned char ctrl5;

    //  Turn on hpf

    ctrl5 = 0x10;

    return I2Cdev::writeByte(m_gyroSlaveAddr,  LSM9DS0_GYRO_CTRL5, ctrl5);
}


bool RTIMULSM9DS0::setAccelCTRL1()
{
    unsigned char ctrl1;

    if ((m_settings->m_LSM9DS0AccelSampleRate < 0) || (m_settings->m_LSM9DS0AccelSampleRate > 10)) {
        return false;
    }

    ctrl1 = (m_settings->m_LSM9DS0AccelSampleRate << 4) | 0x07;

    return I2Cdev::writeByte(m_accelCompassSlaveAddr,  LSM9DS0_CTRL1, ctrl1);
}

bool RTIMULSM9DS0::setAccelCTRL2()
{
    unsigned char ctrl2;

    if ((m_settings->m_LSM9DS0AccelLpf < 0) || (m_settings->m_LSM9DS0AccelLpf > 3)) {
        return false;
    }

    switch (m_settings->m_LSM9DS0AccelFsr) {
    case LSM9DS0_ACCEL_FSR_2:
        m_accelScale = (RTFLOAT)0.000061;
        break;

    case LSM9DS0_ACCEL_FSR_4:
        m_accelScale = (RTFLOAT)0.000122;
        break;

    case LSM9DS0_ACCEL_FSR_6:
        m_accelScale = (RTFLOAT)0.000183;
        break;

    case LSM9DS0_ACCEL_FSR_8:
        m_accelScale = (RTFLOAT)0.000244;
        break;

    case LSM9DS0_ACCEL_FSR_16:
        m_accelScale = (RTFLOAT)0.000732;
        break;

    default:
        return false;
    }

    ctrl2 = (m_settings->m_LSM9DS0AccelLpf << 6) | (m_settings->m_LSM9DS0AccelFsr << 3);

    return I2Cdev::writeByte(m_accelCompassSlaveAddr,  LSM9DS0_CTRL2, ctrl2);
}


bool RTIMULSM9DS0::setCompassCTRL5()
{
    unsigned char ctrl5;

    if ((m_settings->m_LSM9DS0CompassSampleRate < 0) || (m_settings->m_LSM9DS0CompassSampleRate > 5)) {
        return false;
    }

    ctrl5 = (m_settings->m_LSM9DS0CompassSampleRate << 2);

    return I2Cdev::writeByte(m_accelCompassSlaveAddr,  LSM9DS0_CTRL5, ctrl5);
}

bool RTIMULSM9DS0::setCompassCTRL6()
{
    unsigned char ctrl6;

    //  convert FSR to uT

    switch (m_settings->m_LSM9DS0CompassFsr) {
    case LSM9DS0_COMPASS_FSR_2:
        ctrl6 = 0;
        m_compassScale = (RTFLOAT)0.008;
        break;

    case LSM9DS0_COMPASS_FSR_4:
        ctrl6 = 0x20;
        m_compassScale = (RTFLOAT)0.016;
        break;

    case LSM9DS0_COMPASS_FSR_8:
        ctrl6 = 0x40;
        m_compassScale = (RTFLOAT)0.032;
        break;

    case LSM9DS0_COMPASS_FSR_12:
        ctrl6 = 0x60;
        m_compassScale = (RTFLOAT)0.0479;
        break;

    default:
        return false;
    }

    return I2Cdev::writeByte(m_accelCompassSlaveAddr,  LSM9DS0_CTRL6, ctrl6);
}

bool RTIMULSM9DS0::setCompassCTRL7()
{
     return I2Cdev::writeByte(m_accelCompassSlaveAddr,  LSM9DS0_CTRL7, 0x60);
}

int RTIMULSM9DS0::IMUGetPollInterval()
{
    return (400 / m_sampleRate);
}

bool RTIMULSM9DS0::IMURead()
{
    unsigned char status;
    unsigned char gyroData[6];
    unsigned char accelData[6];
    unsigned char compassData[6];

    if (!I2Cdev::readByte(m_gyroSlaveAddr, LSM9DS0_GYRO_STATUS, &status))
        return false;

    if ((status & 0x8) == 0)
        return false;

    if (!I2Cdev::readBytes(m_gyroSlaveAddr, 0x80 | LSM9DS0_GYRO_OUT_X_L, 6, gyroData))
        return false;

    m_timestamp = millis();

    if (!I2Cdev::readBytes(m_accelCompassSlaveAddr, 0x80 | LSM9DS0_OUT_X_L_A, 6, accelData))
        return false;

    if (!I2Cdev::readBytes(m_accelCompassSlaveAddr, 0x80 | LSM9DS0_OUT_X_L_M, 6, compassData))
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

    //  now do standard processing

    handleGyroBias();
    calibrateAverageCompass();

    return true;
}
#endif
