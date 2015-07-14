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

#ifndef _RTIMU_H
#define	_RTIMU_H

#include "RTMath.h"
#include "RTIMULibDefs.h"
#include "I2Cdev.h"

#define I2CWrite(x, y, z) I2Cdev::writeByte(x, y, z)
#define I2CRead(w, x, y, z) I2Cdev::readBytes(w, x, y, z)

class RTIMUSettings;

class RTIMU
{
public:
    //  IMUs should always be created with the following call

    static RTIMU *createIMU(RTIMUSettings *settings);

    //  Constructor/destructor

    RTIMU(RTIMUSettings *settings);
    virtual ~RTIMU();

    //  These functions must be provided by sub classes

    virtual const char *IMUName() = 0;                      // the name of the IMU
    virtual int IMUType() = 0;                              // the type code of the IMU
    virtual int IMUInit() = 0;                              // set up the IMU
    virtual int IMUGetPollInterval() = 0;                   // returns the recommended poll interval in mS
    virtual bool IMURead() = 0;                             // get a sample

    //  This one wanted a similar name but isn't pure virtual

    virtual bool IMUCompassCalValid() { return m_calibrationValid; }

    //  setCalibrationMode() turns off use of cal data so that raw data can be accumulated
    //  to derive calibration data

    void setCalibrationMode(bool enable) { m_calibrationMode = enable; }

    //  setCalibrationData configured the cal data and also enables use if valid

    void setCalibrationData();

    //  getCalibrationValid() returns true if the calibration data is being used

    bool getCalibrationValid() { return !m_calibrationMode && m_calibrationValid; }

    // returns true if enough samples for valid data

    virtual bool IMUGyroBiasValid();

    inline const RTVector3& getGyro() { return m_gyro; }            // gets gyro rates in radians/sec
    inline const RTVector3& getAccel() { return m_accel; }          // get accel data in gs
    inline const RTVector3& getCompass() { return m_compass; }      // gets compass data in uT
    inline unsigned long getTimestamp() { return m_timestamp; }     // and the timestamp for it

protected:
    void gyroBiasInit();                                    // sets up gyro bias calculation
    void handleGyroBias();                                  // adjust gyro for bias
    void calibrateAverageCompass();                         // calibrate and smooth compass
    bool m_calibrationMode;                                 // true if cal mode so don't use cal data!
    bool m_calibrationValid;                                // tru if call data is valid and can be used

    RTVector3 m_gyro;                                       // the gyro readings
    RTVector3 m_accel;                                      // the accel readings
    RTVector3 m_compass;                                    // the compass readings
    unsigned long m_timestamp;                              // the timestamp

    RTIMUSettings *m_settings;                              // the settings object pointer

    int m_sampleRate;                                       // samples per second
    uint64_t m_sampleInterval;                              // interval between samples in microseonds

    RTFLOAT m_gyroAlpha;                                    // gyro bias learning rate
    int m_gyroSampleCount;                                  // number of gyro samples used
    bool m_gyroBiasValid;                                   // true if the recorded gyro bias is valid
    RTVector3 m_gyroBias;                                   // the recorded gyro bias

    RTVector3 m_previousAccel;                              // previous step accel for gyro learning

    RTFLOAT m_compassCalOffset[3];
    RTFLOAT m_compassCalScale[3];
    RTVector3 m_compassAverage;                             // a running average to smooth the mag outputs

    static RTFLOAT m_axisRotation[9];                         // axis rotation matrix

 };

#endif // _RTIMU_H
