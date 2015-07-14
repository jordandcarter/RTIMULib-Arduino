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


#ifndef _RTIMUBNO055_H
#define	_RTIMUBNO055_H

#include "RTIMU.h"

//  I2C Slave Addresses

#define BNO055_ADDRESS0             0x28
#define BNO055_ADDRESS1             0x29
#define BNO055_ID                   0xa0

//  Register map

#define BNO055_WHO_AM_I             0x00
#define BNO055_PAGE_ID              0x07
#define BNO055_ACCEL_DATA           0x08
#define BNO055_MAG_DATA             0x0e
#define BNO055_GYRO_DATA            0x14
#define BNO055_FUSED_EULER          0x1a
#define BNO055_FUSED_QUAT           0x20
#define BNO055_UNIT_SEL             0x3b
#define BNO055_OPER_MODE            0x3d
#define BNO055_PWR_MODE             0x3e
#define BNO055_SYS_TRIGGER          0x3f
#define BNO055_AXIS_MAP_CONFIG      0x41
#define BNO055_AXIS_MAP_SIGN        0x42

//  Operation modes

#define BNO055_OPER_MODE_CONFIG     0x00
#define BNO055_OPER_MODE_NDOF       0x0c

//  Power modes

#define BNO055_PWR_MODE_NORMAL      0x00

class RTIMUBNO055 : public RTIMU
{
public:
    RTIMUBNO055(RTIMUSettings *settings);
    ~RTIMUBNO055();

    inline const RTVector3& getFusionPose() { return m_fusionPose; }
    inline const RTQuaternion& getFusionQPose() { return m_fusionQPose; }

    virtual const char *IMUName() { return "BNO055"; }
    virtual int IMUType() { return RTIMU_TYPE_BNO055; }
    virtual int IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();

private:
    unsigned char m_slaveAddr;                              // I2C address of BNO055

    uint64_t m_lastReadTime;

    RTQuaternion m_fusionQPose;
    RTVector3 m_fusionPose;
};

#endif // _RTIMUBNO055_H
