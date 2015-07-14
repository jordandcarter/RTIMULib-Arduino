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

#ifndef _RTMATH_H_
#define _RTMATH_H_

#include <math.h>
#include <stdint.h>

//  The fundamental float type

#ifdef RTMATH_USE_DOUBLE
typedef double RTFLOAT;
#else
typedef float RTFLOAT;
#endif

//  Useful constants

#define	RTMATH_PI                   3.1415926535
#define	RTMATH_DEGREE_TO_RAD        (M_PI / 180.0)
#define	RTMATH_RAD_TO_DEGREE        (180.0 / M_PI)

class RTVector3;

#ifndef RTARDULINK_MODE
class RTQuaternion;
#endif

class RTMath
{
public:
#ifndef RTARDULINK_MODE
    // convenient display routines

    static void display(const char *label, RTVector3& vec);
    static void displayDegrees(const char *label, RTVector3& vec);
    static void displayRollPitchYaw(const char *label, RTVector3& vec);
    static void display(const char *label, RTQuaternion& quat);

    //  poseFromAccelMag generates pose Euler angles from measured settings

    static RTVector3 poseFromAccelMag(const RTVector3& accel, const RTVector3& mag);

    //  Takes signed 16 bit data from a char array and converts it to a vector of scaled RTFLOATs

    static void convertToVector(unsigned char *rawData, RTVector3& vec, RTFLOAT scale, bool bigEndian);

#endif // #ifndef RTARDULINK_MODE
};


class RTVector3
{
public:
    RTVector3();
    RTVector3(RTFLOAT x, RTFLOAT y, RTFLOAT z);

    const RTVector3&  operator +=(RTVector3& vec);
    const RTVector3&  operator -=(RTVector3& vec);

    RTVector3& operator =(const RTVector3& vec);

    RTFLOAT squareLength();
    void zero();

    inline RTFLOAT x() const { return m_data[0]; }
    inline RTFLOAT y() const { return m_data[1]; }
    inline RTFLOAT z() const { return m_data[2]; }
    inline RTFLOAT data(const int i) const { return m_data[i]; }

    inline void setX(const RTFLOAT val) { m_data[0] = val; }
    inline void setY(const RTFLOAT val) { m_data[1] = val; }
    inline void setZ(const RTFLOAT val) { m_data[2] = val; }
    inline void setData(const int i, RTFLOAT val) { m_data[i] = val; }

    #ifndef RTARDULINK_MODE
    RTFLOAT length();
    void normalize();

    const char *display();
    const char *displayDegrees();

    static RTFLOAT dotProduct(const RTVector3& a, const RTVector3& b);
    static void crossProduct(const RTVector3& a, const RTVector3& b, RTVector3& d);

    void accelToEuler(RTVector3& rollPitchYaw) const;
    void accelToQuaternion(RTQuaternion& qPose) const;
#endif // #ifndef RTARDULINK_MODE

private:
    RTFLOAT m_data[3];
};

#ifndef RTARDULINK_MODE
class RTQuaternion
{
public:
    RTQuaternion();
    RTQuaternion(RTFLOAT scalar, RTFLOAT x, RTFLOAT y, RTFLOAT z);

    RTQuaternion& operator +=(const RTQuaternion& quat);
    RTQuaternion& operator -=(const RTQuaternion& quat);
    RTQuaternion& operator *=(const RTQuaternion& qb);
    RTQuaternion& operator *=(const RTFLOAT val);
    RTQuaternion& operator -=(const RTFLOAT val);

    RTQuaternion& operator =(const RTQuaternion& quat);
    const RTQuaternion operator *(const RTQuaternion& qb) const;
    const RTQuaternion operator *(const RTFLOAT val) const;
    const RTQuaternion operator -(const RTQuaternion& qb) const;
    const RTQuaternion operator -(const RTFLOAT val) const;

    void normalize();
    void toEuler(RTVector3& vec);
    void fromEuler(RTVector3& vec);
    RTQuaternion conjugate() const;
    void toAngleVector(RTFLOAT& angle, RTVector3& vec);
    void fromAngleVector(const RTFLOAT& angle, const RTVector3& vec);

    void zero();
    const char *display();

    inline RTFLOAT scalar() const { return m_data[0]; }
    inline RTFLOAT x() const { return m_data[1]; }
    inline RTFLOAT y() const { return m_data[2]; }
    inline RTFLOAT z() const { return m_data[3]; }
    inline RTFLOAT data(const int i) const { return m_data[i]; }

    inline void setScalar(const RTFLOAT val) { m_data[0] = val; }
    inline void setX(const RTFLOAT val) { m_data[1] = val; }
    inline void setY(const RTFLOAT val) { m_data[2] = val; }
    inline void setZ(const RTFLOAT val) { m_data[3] = val; }
    inline void setData(const int i, RTFLOAT val) { m_data[i] = val; }

private:
    RTFLOAT m_data[4];
};
#endif // #ifndef RTARDULINK_MODE

#endif /* _RTMATH_H_ */
