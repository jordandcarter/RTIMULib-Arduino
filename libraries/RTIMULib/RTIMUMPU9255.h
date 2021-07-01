////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
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

//  The MPU-9255 and SPI driver code is based on code generously supplied by
//  staslock@gmail.com (www.clickdrive.io)


#ifndef _RTIMUMPU9255_H
#define	_RTIMUMPU9255_H

#include "RTIMU.h"


#define MPU9255_ADDRESS0            0x68
#define MPU9255_ADDRESS1            0x69
#define MPU9255_ID                  0x71

#define AK8963_ADDRESS              0x0c

//  Register map

#define MPU9255_SMPRT_DIV           0x19
#define MPU9255_GYRO_LPF            0x1a
#define MPU9255_GYRO_CONFIG         0x1b
#define MPU9255_ACCEL_CONFIG        0x1c
#define MPU9255_ACCEL_LPF           0x1d
#define MPU9255_FIFO_EN             0x23
#define MPU9255_I2C_MST_CTRL        0x24
#define MPU9255_I2C_SLV0_ADDR       0x25
#define MPU9255_I2C_SLV0_REG        0x26
#define MPU9255_I2C_SLV0_CTRL       0x27
#define MPU9255_I2C_SLV1_ADDR       0x28
#define MPU9255_I2C_SLV1_REG        0x29
#define MPU9255_I2C_SLV1_CTRL       0x2a
#define MPU9255_I2C_SLV2_ADDR       0x2b
#define MPU9255_I2C_SLV2_REG        0x2c
#define MPU9255_I2C_SLV2_CTRL       0x2d
#define MPU9255_I2C_SLV4_CTRL       0x34
#define MPU9255_INT_PIN_CFG         0x37
#define MPU9255_INT_ENABLE          0x38
#define MPU9255_INT_STATUS          0x3a
#define MPU9255_ACCEL_XOUT_H        0x3b
#define MPU9255_GYRO_XOUT_H         0x43
#define MPU9255_EXT_SENS_DATA_00    0x49
#define MPU9255_I2C_SLV1_DO         0x64
#define MPU9255_I2C_MST_DELAY_CTRL  0x67
#define MPU9255_USER_CTRL           0x6a
#define MPU9255_PWR_MGMT_1          0x6b
#define MPU9255_PWR_MGMT_2          0x6c
#define MPU9255_FIFO_COUNT_H        0x72
#define MPU9255_FIFO_R_W            0x74
#define MPU9255_WHO_AM_I            0x75

//  sample rate defines (applies to gyros and accels, not mags)

#define MPU9255_SAMPLERATE_MIN      5                       // 5 samples per second is the lowest
#define MPU9255_SAMPLERATE_MAX      32000                   // 32000 samples per second is the absolute maximum
//  compass rate defines

#define MPU9255_COMPASSRATE_MIN     1                       // 1 samples per second is the lowest
#define MPU9255_COMPASSRATE_MAX     100                     // 100 samples per second is maximum

//  Gyro LPF options

#define MPU9255_GYRO_LPF_8800       0x11                    // 8800Hz, 0.64mS delay
#define MPU9255_GYRO_LPF_3600       0x10                    // 3600Hz, 0.11mS delay
#define MPU9255_GYRO_LPF_250        0x00                    // 250Hz, 0.97mS delay
#define MPU9255_GYRO_LPF_184        0x01                    // 184Hz, 2.9mS delay
#define MPU9255_GYRO_LPF_92         0x02                    // 92Hz, 3.9mS delay
#define MPU9255_GYRO_LPF_41         0x03                    // 41Hz, 5.9mS delay
#define MPU9255_GYRO_LPF_20         0x04                    // 20Hz, 9.9mS delay
#define MPU9255_GYRO_LPF_10         0x05                    // 10Hz, 17.85mS delay
#define MPU9255_GYRO_LPF_5          0x06                    // 5Hz, 33.48mS delay
//  Gyro FSR options

#define MPU9255_GYROFSR_250         0                       // +/- 250 degrees per second
#define MPU9255_GYROFSR_500         8                       // +/- 500 degrees per second
#define MPU9255_GYROFSR_1000        0x10                    // +/- 1000 degrees per second
#define MPU9255_GYROFSR_2000        0x18                    // +/- 2000 degrees per second
//  Accel FSR options

#define MPU9255_ACCELFSR_2          0                       // +/- 2g
#define MPU9255_ACCELFSR_4          8                       // +/- 4g
#define MPU9255_ACCELFSR_8          0x10                    // +/- 8g
#define MPU9255_ACCELFSR_16         0x18                    // +/- 16g
//  Accel LPF options

#define MPU9255_ACCEL_LPF_1130      0x08                    // 1130Hz, 0.75mS delay
#define MPU9255_ACCEL_LPF_460       0x00                    // 460Hz, 1.94mS delay
#define MPU9255_ACCEL_LPF_184       0x01                    // 184Hz, 5.80mS delay
#define MPU9255_ACCEL_LPF_92        0x02                    // 92Hz, 7.80mS delay
#define MPU9255_ACCEL_LPF_41        0x03                    // 41Hz, 11.80mS delay
#define MPU9255_ACCEL_LPF_20        0x04                    // 20Hz, 19.80mS delay
#define MPU9255_ACCEL_LPF_10        0x05                    // 10Hz, 35.70mS delay
#define MPU9255_ACCEL_LPF_5         0x06                    // 5Hz, 66.96mS delay
//  AK8963 compass registers

#define AK8963_DEVICEID             0x48                    // the device ID
#define AK8963_ST1                  0x02                    // status 1
#define AK8963_CNTL                 0x0a                    // control reg
#define AK8963_ASAX                 0x10                    // start of the fuse ROM data

//  FIFO transfer size

#define MPU9255_FIFO_CHUNK_SIZE     12                      // gyro and accels take 12 bytes

class RTIMUMPU9255 : public RTIMU
{
public:
    RTIMUMPU9255(RTIMUSettings *settings);
    ~RTIMUMPU9255();

    bool setGyroLpf(unsigned char lpf);
    bool setAccelLpf(unsigned char lpf);
    bool setSampleRate(int rate);
    bool setCompassRate(int rate);
    bool setGyroFsr(unsigned char fsr);
    bool setAccelFsr(unsigned char fsr);

    virtual const char *IMUName() { return "MPU-9255"; }
    virtual int IMUType() { return RTIMU_TYPE_MPU9255; }
    virtual int IMUInit();
    virtual bool IMURead();
    virtual int IMUGetPollInterval();



private:
    bool setGyroConfig();
    bool setAccelConfig();
    bool setSampleRate();
    int compassSetup();
    bool setCompassRate();
    bool resetFifo();
    bool bypassOn();
    bool bypassOff();

    bool m_firstTime;                                       // if first sample

    unsigned char m_slaveAddr;                              // I2C address of MPU9150
    unsigned char m_bus;                                    // I2C bus (usually 1 for Raspberry Pi for example)

    unsigned char m_gyroLpf;                                // gyro low pass filter setting
    unsigned char m_accelLpf;                               // accel low pass filter setting
    int m_compassRate;                                      // compass sample rate in Hz
    unsigned char m_gyroFsr;
    unsigned char m_accelFsr;

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;

    RTFLOAT m_compassAdjust[3];                             // the compass fuse ROM values converted for use

};

#endif // _RTIMUMPU9255_H
