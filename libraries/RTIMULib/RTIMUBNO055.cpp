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

//  Based on the Adafruit BNO055 driver:

/***************************************************************************
  This is a library for the BNO055 orientation sensor
  Designed specifically to work with the Adafruit BNO055 Breakout.
  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products
  These sensors use I2C to communicate, 2 pins are required to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by KTOWN for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ***************************************************************************/

#include "RTIMUBNO055.h"
#include "RTIMUSettings.h"
#if defined(BNO055_28) || defined(BNO055_29)

RTIMUBNO055::RTIMUBNO055(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 100;
    m_sampleInterval = (unsigned long)1000 / m_sampleRate;
}

RTIMUBNO055::~RTIMUBNO055()
{
}

int RTIMUBNO055::IMUInit()
{
    unsigned char result;

    m_slaveAddr = m_settings->m_I2CSlaveAddress;
    m_lastReadTime = millis();

    if (!I2Cdev::readByte(m_slaveAddr, BNO055_WHO_AM_I, &result))
        return -1;

    if (result != BNO055_ID) {
        return -2;
    }

    if (!I2Cdev::writeByte(m_slaveAddr, BNO055_OPER_MODE, BNO055_OPER_MODE_CONFIG))
        return -3;

    delay(50);

    if (!I2Cdev::writeByte(m_slaveAddr, BNO055_SYS_TRIGGER, 0x20))
        return -4;

    delay(50);

    while (1) {
        if (!I2Cdev::readByte(m_slaveAddr, BNO055_WHO_AM_I, &result))
            continue;
        if (result == BNO055_ID)
            break;
        delay(50);
    }

    delay(50);

    if (!I2Cdev::writeByte(m_slaveAddr, BNO055_PWR_MODE, BNO055_PWR_MODE_NORMAL))
        return -5;

    delay(50);

    if (!I2Cdev::writeByte(m_slaveAddr, BNO055_PAGE_ID, 0))
        return -6;

    delay(50);

    if (!I2Cdev::writeByte(m_slaveAddr, BNO055_SYS_TRIGGER, 0x00))
        return -7;

    delay(50);

    if (!I2Cdev::writeByte(m_slaveAddr, BNO055_UNIT_SEL, 0x87))
        return -8;

    delay(50);

    if (!I2Cdev::writeByte(m_slaveAddr, BNO055_OPER_MODE, BNO055_OPER_MODE_NDOF))
        return -9;

    delay(50);

    return 1;
}

int RTIMUBNO055::IMUGetPollInterval()
{
    return (7);
}

bool RTIMUBNO055::IMURead()
{
    unsigned char buffer[24];

    if ((millis() - m_lastReadTime) < m_sampleInterval)
        return false;                                       // too soon

    m_lastReadTime = millis();
    if (!I2Cdev::readBytes(m_slaveAddr, BNO055_ACCEL_DATA, 24, buffer))
        return false;

    int16_t x, y, z;

    // process accel data

    x = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
    y = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
    z = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);

    m_accel.setX((RTFLOAT)y / 1000.0);
    m_accel.setY((RTFLOAT)x / 1000.0);
    m_accel.setZ((RTFLOAT)z / 1000.0);

    // process mag data

    x = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);
    y = (((uint16_t)buffer[9]) << 8) | ((uint16_t)buffer[8]);
    z = (((uint16_t)buffer[11]) << 8) | ((uint16_t)buffer[10]);

    m_compass.setX(-(RTFLOAT)y / 16.0);
    m_compass.setY(-(RTFLOAT)x / 16.0);
    m_compass.setZ(-(RTFLOAT)z / 16.0);

    // process gyro data

    x = (((uint16_t)buffer[13]) << 8) | ((uint16_t)buffer[12]);
    y = (((uint16_t)buffer[15]) << 8) | ((uint16_t)buffer[14]);
    z = (((uint16_t)buffer[17]) << 8) | ((uint16_t)buffer[16]);

    m_gyro.setX(-(RTFLOAT)y / 900.0);
    m_gyro.setY(-(RTFLOAT)x / 900.0);
    m_gyro.setZ(-(RTFLOAT)z / 900.0);

    // process euler angles

    x = (((uint16_t)buffer[19]) << 8) | ((uint16_t)buffer[18]);
    y = (((uint16_t)buffer[21]) << 8) | ((uint16_t)buffer[20]);
    z = (((uint16_t)buffer[23]) << 8) | ((uint16_t)buffer[22]);

    //  put in structure and do axis remap

    m_fusionPose.setX((RTFLOAT)y / 900.0);
    m_fusionPose.setY((RTFLOAT)z / 900.0);
    m_fusionPose.setZ((RTFLOAT)x / 900.0);

    m_fusionQPose.fromEuler(m_fusionPose);

    m_timestamp = millis();
    return true;
}
#endif