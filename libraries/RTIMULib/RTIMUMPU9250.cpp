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

#include "RTIMUMPU9250.h"
#include "RTIMUSettings.h"

#if defined(MPU9250_68) || defined(MPU9250_69)

RTIMUMPU9250::RTIMUMPU9250(RTIMUSettings *settings) : RTIMU(settings)
{

}

RTIMUMPU9250::~RTIMUMPU9250()
{
}


bool RTIMUMPU9250::setSampleRate(int rate)
{
    if ((rate < MPU9250_SAMPLERATE_MIN) || (rate > MPU9250_SAMPLERATE_MAX)) {
        return false;
    }
    m_sampleRate = rate;
    m_sampleInterval = (unsigned long)1000 / m_sampleRate;
    if (m_sampleInterval == 0)
        m_sampleInterval = 1;
    return true;
}

bool RTIMUMPU9250::setGyroLpf(unsigned char lpf)
{
    switch (lpf) {
    case MPU9250_GYRO_LPF_8800:
    case MPU9250_GYRO_LPF_3600:
    case MPU9250_GYRO_LPF_250:
    case MPU9250_GYRO_LPF_184:
    case MPU9250_GYRO_LPF_92:
    case MPU9250_GYRO_LPF_41:
    case MPU9250_GYRO_LPF_20:
    case MPU9250_GYRO_LPF_10:
    case MPU9250_GYRO_LPF_5:
        m_gyroLpf = lpf;
        return true;

    default:
        return false;
    }
}

bool RTIMUMPU9250::setAccelLpf(unsigned char lpf)
{
    switch (lpf) {
    case MPU9250_ACCEL_LPF_1130:
    case MPU9250_ACCEL_LPF_460:
    case MPU9250_ACCEL_LPF_184:
    case MPU9250_ACCEL_LPF_92:
    case MPU9250_ACCEL_LPF_41:
    case MPU9250_ACCEL_LPF_20:
    case MPU9250_ACCEL_LPF_10:
    case MPU9250_ACCEL_LPF_5:
        m_accelLpf = lpf;
        return true;

    default:
        return false;
    }
}

bool RTIMUMPU9250::setCompassRate(int rate)
{
    if ((rate < MPU9250_COMPASSRATE_MIN) || (rate > MPU9250_COMPASSRATE_MAX)) {
        return false;
    }
    m_compassRate = rate;
    return true;
}

bool RTIMUMPU9250::setGyroFsr(unsigned char fsr)
{
    switch (fsr) {
    case MPU9250_GYROFSR_250:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (131.0 * 180.0);
        return true;

    case MPU9250_GYROFSR_500:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (62.5 * 180.0);
        return true;

    case MPU9250_GYROFSR_1000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (32.8 * 180.0);
        return true;

    case MPU9250_GYROFSR_2000:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (16.4 * 180.0);
        return true;

    default:
        return false;
    }
}

bool RTIMUMPU9250::setAccelFsr(unsigned char fsr)
{
    switch (fsr) {
    case MPU9250_ACCELFSR_2:
        m_accelFsr = fsr;
        m_accelScale = 1.0/16384.0;
        return true;

    case MPU9250_ACCELFSR_4:
        m_accelFsr = fsr;
        m_accelScale = 1.0/8192.0;
        return true;

    case MPU9250_ACCELFSR_8:
        m_accelFsr = fsr;
        m_accelScale = 1.0/4096.0;
        return true;

    case MPU9250_ACCELFSR_16:
        m_accelFsr = fsr;
        m_accelScale = 1.0/2048.0;
        return true;

    default:
        return false;
    }
}


int RTIMUMPU9250::IMUInit()
{
    unsigned char result;
    unsigned char asa[3];

    m_firstTime = true;

#ifdef MPU9250_CACHE_MODE
    m_cacheIn = m_cacheOut = m_cacheCount = 0;
#endif
    //  configure IMU

    m_slaveAddr = m_settings->m_I2CSlaveAddress;

    setSampleRate(m_settings->m_MPU9250GyroAccelSampleRate);
    setCompassRate(m_settings->m_MPU9250CompassSampleRate);
    setGyroLpf(m_settings->m_MPU9250GyroLpf);
    setAccelLpf(m_settings->m_MPU9250AccelLpf);
    setGyroFsr(m_settings->m_MPU9250GyroFsr);
    setAccelFsr(m_settings->m_MPU9250AccelFsr);

    setCalibrationData();

    //  reset the MPU9250

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_PWR_MGMT_1, 0x80))
        return -1;

    delay(100);

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_PWR_MGMT_1, 0x00))
        return -4;

    if (!I2Cdev::readByte(m_slaveAddr, MPU9250_WHO_AM_I, &result))
        return -5;

    if (result != MPU9250_ID) {
         return -6;
    }

    //  now configure the various components

    if (!setGyroConfig())
        return -7;

    if (!setAccelConfig())
        return -8;

    if (!setSampleRate())
        return -9;

    //  now configure compass

    if (!bypassOn())
        return -11;

    // get fuse ROM data

    if (!I2Cdev::writeByte(AK8963_ADDRESS, AK8963_CNTL, 0)) {
        bypassOff();
        return -12;
    }

    if (!I2Cdev::writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0f)) {
        bypassOff();
        return -13;
    }

    if (!I2Cdev::readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, asa)) {
        bypassOff();
        return -14;
    }

    //  convert asa to usable scale factor

    m_compassAdjust[0] = ((float)asa[0] - 128.0) / 256.0 + 1.0f;
    m_compassAdjust[1] = ((float)asa[1] - 128.0) / 256.0 + 1.0f;
    m_compassAdjust[2] = ((float)asa[2] - 128.0) / 256.0 + 1.0f;

    if (!I2Cdev::writeByte(AK8963_ADDRESS, AK8963_CNTL, 0)) {
        bypassOff();
        return -15;
    }

    if (!bypassOff())
        return -16;

    //  now set up MPU9250 to talk to the compass chip

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_I2C_MST_CTRL, 0x40))
        return -17;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_I2C_SLV0_ADDR, 0x80 | AK8963_ADDRESS))
        return -18;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_I2C_SLV0_REG, AK8963_ST1))
        return -19;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_I2C_SLV0_CTRL, 0x88))
        return -20;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_I2C_SLV1_ADDR, AK8963_ADDRESS))
        return -21;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_I2C_SLV1_REG, AK8963_CNTL))
        return -22;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_I2C_SLV1_CTRL, 0x81))
        return -23;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_I2C_SLV1_DO, 0x1))
        return -24;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_I2C_MST_DELAY_CTRL, 0x3))
        return -25;

    if (!setCompassRate())
        return -27;

    //  enable the sensors

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_PWR_MGMT_1, 1))
        return -28;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_PWR_MGMT_2, 0))
         return -29;

    //  select the data to go into the FIFO and enable

    if (!resetFifo())
        return -30;

    gyroBiasInit();
    return 1;
}

bool RTIMUMPU9250::resetFifo()
{
    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_INT_ENABLE, 0))
        return false;
    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_FIFO_EN, 0))
        return false;
    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_USER_CTRL, 0))
        return false;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_USER_CTRL, 0x04))
        return false;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_USER_CTRL, 0x60))
        return false;

    delay(50);

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_INT_ENABLE, 1))
        return false;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_FIFO_EN, 0x78))
        return false;

    return true;
}

bool RTIMUMPU9250::bypassOn()
{
    unsigned char userControl;

    if (!I2Cdev::readByte(m_slaveAddr, MPU9250_USER_CTRL, &userControl))
        return false;

    userControl &= ~0x20;
    userControl |= 2;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_USER_CTRL, userControl))
        return false;

    delay(50);

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_INT_PIN_CFG, 0x82))
        return false;

    delay(50);
    return true;
}


bool RTIMUMPU9250::bypassOff()
{
    unsigned char userControl;

    if (!I2Cdev::readByte(m_slaveAddr, MPU9250_USER_CTRL, &userControl))
        return false;

    userControl |= 0x20;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_USER_CTRL, userControl))
        return false;

    delay(50);

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_INT_PIN_CFG, 0x80))
         return false;

    delay(50);
    return true;
}

bool RTIMUMPU9250::setGyroConfig()
{
    unsigned char gyroConfig = m_gyroFsr + ((m_gyroLpf >> 3) & 3);
    unsigned char gyroLpf = m_gyroLpf & 7;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_GYRO_CONFIG, gyroConfig))
         return false;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_GYRO_LPF, gyroLpf))
         return false;
    return true;
}

bool RTIMUMPU9250::setAccelConfig()
{
    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_ACCEL_CONFIG, m_accelFsr))
         return false;

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_ACCEL_LPF, m_accelLpf))
         return false;
    return true;
}

bool RTIMUMPU9250::setSampleRate()
{
    if (m_sampleRate > 1000)
        return true;                                        // SMPRT not used above 1000Hz

    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_SMPRT_DIV, (unsigned char) (1000 / m_sampleRate - 1)))
        return false;

    return true;
}


bool RTIMUMPU9250::setCompassRate()
{
    int rate;

    rate = m_sampleRate / m_compassRate - 1;

    if (rate > 31)
        rate = 31;
    if (!I2Cdev::writeByte(m_slaveAddr, MPU9250_I2C_SLV4_CTRL, rate))
         return false;
    return true;
}

int RTIMUMPU9250::IMUGetPollInterval()
{
    return (400 / m_sampleRate);
}

bool RTIMUMPU9250::IMURead()
{
    unsigned char fifoCount[2];
    unsigned int count;
    unsigned char fifoData[12];
    unsigned char compassData[8];

    if (!I2Cdev::readBytes(m_slaveAddr, MPU9250_FIFO_COUNT_H, 2, fifoCount))
         return false;

    count = ((unsigned int)fifoCount[0] << 8) + fifoCount[1];

    if (count == 1024) {
        resetFifo();
        m_timestamp += m_sampleInterval * (1024 / MPU9250_FIFO_CHUNK_SIZE + 1); // try to fix timestamp
        return false;
    }

    if (count > MPU9250_FIFO_CHUNK_SIZE * 40) {
        // more than 40 samples behind - going too slowly so discard some samples but maintain timestamp correctly
        while (count >= MPU9250_FIFO_CHUNK_SIZE * 10) {
            if (!I2Cdev::readBytes(m_slaveAddr, MPU9250_FIFO_R_W, MPU9250_FIFO_CHUNK_SIZE, fifoData))
                return false;
            count -= MPU9250_FIFO_CHUNK_SIZE;
            m_timestamp += m_sampleInterval;
        }
    }

    if (count < MPU9250_FIFO_CHUNK_SIZE)
        return false;

    if (!I2Cdev::readBytes(m_slaveAddr, MPU9250_FIFO_R_W, MPU9250_FIFO_CHUNK_SIZE, fifoData))
        return false;

    if (!I2Cdev::readBytes(m_slaveAddr, MPU9250_EXT_SENS_DATA_00, 8, compassData))
        return false;

    RTMath::convertToVector(fifoData, m_accel, m_accelScale, true);
    RTMath::convertToVector(fifoData + 6, m_gyro, m_gyroScale, true);
    RTMath::convertToVector(compassData + 1, m_compass, 0.6f, false);

    //  sort out gyro axes

    m_gyro.setY(-m_gyro.y());
    m_gyro.setZ(-m_gyro.z());

    //  sort out accel data;

    m_accel.setX(-m_accel.x());

    //  use the fuse data adjustments for compass

    m_compass.setX(m_compass.x() * m_compassAdjust[0]);
    m_compass.setY(m_compass.y() * m_compassAdjust[1]);
    m_compass.setZ(m_compass.z() * m_compassAdjust[2]);

    //  sort out compass axes

    float temp;

    temp = m_compass.x();
    m_compass.setX(m_compass.y());
    m_compass.setY(-temp);

    //  now do standard processing

    handleGyroBias();
    calibrateAverageCompass();

    if (m_firstTime)
        m_timestamp = millis();
    else
        m_timestamp += m_sampleInterval;

    m_firstTime = false;

    return true;
}
#endif
