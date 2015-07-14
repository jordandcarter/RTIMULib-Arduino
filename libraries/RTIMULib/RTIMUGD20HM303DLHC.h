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

#ifndef _RTIMUGD20HM303DLHC_H
#define	_RTIMUGD20HM303DLHC_H

#include "RTIMU.h"

//  I2C Slave Addresses

#define L3GD20H_ADDRESS0        0x6a
#define L3GD20H_ADDRESS1        0x6b
#define L3GD20H_ID              0xd7

#define LSM303DLHC_ACCEL_ADDRESS    0x19
#define LSM303DLHC_COMPASS_ADDRESS  0x1e

//  L3GD20H Register map

#define L3GD20H_WHO_AM_I        0x0f
#define L3GD20H_CTRL1           0x20
#define L3GD20H_CTRL2           0x21
#define L3GD20H_CTRL3           0x22
#define L3GD20H_CTRL4           0x23
#define L3GD20H_CTRL5           0x24
#define L3GD20H_OUT_TEMP        0x26
#define L3GD20H_STATUS          0x27
#define L3GD20H_OUT_X_L         0x28
#define L3GD20H_OUT_X_H         0x29
#define L3GD20H_OUT_Y_L         0x2a
#define L3GD20H_OUT_Y_H         0x2b
#define L3GD20H_OUT_Z_L         0x2c
#define L3GD20H_OUT_Z_H         0x2d
#define L3GD20H_FIFO_CTRL       0x2e
#define L3GD20H_FIFO_SRC        0x2f
#define L3GD20H_IG_CFG          0x30
#define L3GD20H_IG_SRC          0x31
#define L3GD20H_IG_THS_XH       0x32
#define L3GD20H_IG_THS_XL       0x33
#define L3GD20H_IG_THS_YH       0x34
#define L3GD20H_IG_THS_YL       0x35
#define L3GD20H_IG_THS_ZH       0x36
#define L3GD20H_IG_THS_ZL       0x37
#define L3GD20H_IG_DURATION     0x38
#define L3GD20H_LOW_ODR         0x39

//  Gyro sample rate defines

#define L3GD20H_SAMPLERATE_12_5 0
#define L3GD20H_SAMPLERATE_25   1
#define L3GD20H_SAMPLERATE_50   2
#define L3GD20H_SAMPLERATE_100  3
#define L3GD20H_SAMPLERATE_200  4
#define L3GD20H_SAMPLERATE_400  5
#define L3GD20H_SAMPLERATE_800  6

//  Gyro banwidth defines

#define L3GD20H_BANDWIDTH_0     0
#define L3GD20H_BANDWIDTH_1     1
#define L3GD20H_BANDWIDTH_2     2
#define L3GD20H_BANDWIDTH_3     3

//  Gyro FSR defines

#define L3GD20H_FSR_245         0
#define L3GD20H_FSR_500         1
#define L3GD20H_FSR_2000        2

//  Gyro high pass filter defines

#define L3GD20H_HPF_0           0
#define L3GD20H_HPF_1           1
#define L3GD20H_HPF_2           2
#define L3GD20H_HPF_3           3
#define L3GD20H_HPF_4           4
#define L3GD20H_HPF_5           5
#define L3GD20H_HPF_6           6
#define L3GD20H_HPF_7           7
#define L3GD20H_HPF_8           8
#define L3GD20H_HPF_9           9

//  LSM303DLHC Accel Register Map

#define LSM303DLHC_CTRL1_A         0x20
#define LSM303DLHC_CTRL2_A         0x21
#define LSM303DLHC_CTRL3_A         0x22
#define LSM303DLHC_CTRL4_A         0x23
#define LSM303DLHC_CTRL5_A         0x24
#define LSM303DLHC_CTRL6_A         0x25
#define LSM303DLHC_REF_A           0x26
#define LSM303DLHC_STATUS_A        0x27
#define LSM303DLHC_OUT_X_L_A       0x28
#define LSM303DLHC_OUT_X_H_A       0x29
#define LSM303DLHC_OUT_Y_L_A       0x2a
#define LSM303DLHC_OUT_Y_H_A       0x2b
#define LSM303DLHC_OUT_Z_L_A       0x2c
#define LSM303DLHC_OUT_Z_H_A       0x2d
#define LSM303DLHC_FIFO_CTRL_A     0x2e
#define LSM303DLHC_FIFO_SRC_A      0x2f

//  LSM303DLHC Compass Register Map

#define LSM303DLHC_CRA_M            0x00
#define LSM303DLHC_CRB_M            0x01
#define LSM303DLHC_CRM_M            0x02
#define LSM303DLHC_OUT_X_H_M        0x03
#define LSM303DLHC_OUT_X_L_M        0x04
#define LSM303DLHC_OUT_Y_H_M        0x05
#define LSM303DLHC_OUT_Y_L_M        0x06
#define LSM303DLHC_OUT_Z_H_M        0x07
#define LSM303DLHC_OUT_Z_L_M        0x08
#define LSM303DLHC_STATUS_M         0x09
#define LSM303DLHC_TEMP_OUT_L_M     0x31
#define LSM303DLHC_TEMP_OUT_H_M     0x32

//  Accel sample rate defines

#define LSM303DLHC_ACCEL_SAMPLERATE_1       1
#define LSM303DLHC_ACCEL_SAMPLERATE_10      2
#define LSM303DLHC_ACCEL_SAMPLERATE_25      3
#define LSM303DLHC_ACCEL_SAMPLERATE_50      4
#define LSM303DLHC_ACCEL_SAMPLERATE_100     5
#define LSM303DLHC_ACCEL_SAMPLERATE_200     6
#define LSM303DLHC_ACCEL_SAMPLERATE_400     7

//  Accel FSR

#define LSM303DLHC_ACCEL_FSR_2     0
#define LSM303DLHC_ACCEL_FSR_4     1
#define LSM303DLHC_ACCEL_FSR_8     2
#define LSM303DLHC_ACCEL_FSR_16    3

//  Compass sample rate defines

#define LSM303DLHC_COMPASS_SAMPLERATE_0_75      0
#define LSM303DLHC_COMPASS_SAMPLERATE_1_5       1
#define LSM303DLHC_COMPASS_SAMPLERATE_3         2
#define LSM303DLHC_COMPASS_SAMPLERATE_7_5       3
#define LSM303DLHC_COMPASS_SAMPLERATE_15        4
#define LSM303DLHC_COMPASS_SAMPLERATE_30        5
#define LSM303DLHC_COMPASS_SAMPLERATE_75        6
#define LSM303DLHC_COMPASS_SAMPLERATE_220       7

//  Compass FSR

#define LSM303DLHC_COMPASS_FSR_1_3      1
#define LSM303DLHC_COMPASS_FSR_1_9      2
#define LSM303DLHC_COMPASS_FSR_2_5      3
#define LSM303DLHC_COMPASS_FSR_4        4
#define LSM303DLHC_COMPASS_FSR_4_7      5
#define LSM303DLHC_COMPASS_FSR_5_6      6
#define LSM303DLHC_COMPASS_FSR_8_1      7

class RTIMUGD20HM303DLHC : public RTIMU
{
public:
    RTIMUGD20HM303DLHC(RTIMUSettings *settings);
    ~RTIMUGD20HM303DLHC();

    virtual const char *IMUName() { return "L3GD20H + LSM303DLHC"; }
    virtual int IMUType() { return RTIMU_TYPE_GD20HM303DLHC; }
    virtual int IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();

private:
    bool setGyroSampleRate();
    bool setGyroCTRL2();
    bool setGyroCTRL4();
    bool setGyroCTRL5();
    bool setAccelCTRL1();
    bool setAccelCTRL4();
    bool setCompassCRA();
    bool setCompassCRB();
    bool setCompassCRM();

    unsigned char m_gyroSlaveAddr;                          // I2C address of L3GD20
    unsigned char m_accelSlaveAddr;                         // I2C address of LSM303DLHC accel
    unsigned char m_compassSlaveAddr;                       // I2C address of LSM303DLHC compass

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;
    RTFLOAT m_compassScaleXY;
    RTFLOAT m_compassScaleZ;
};

#endif // _RTIMUGD20HM303DLHC_H
