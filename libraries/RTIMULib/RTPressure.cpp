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


#include "RTPressure.h"

#if defined(BMP180)
#include "RTPressureBMP180.h"
#endif
#if defined(LPS25H_5c) || defined(LPS25H_5d)
#include "RTPressureLPS25H.h"
#endif
#if defined(MS5611_76) || defined(MS5611_77)
#include "RTPressureMS5611.h"
#endif

RTPressure *RTPressure::createPressure(RTIMUSettings *settings)
{
#if defined(BMP180)
    return new RTPressureBMP180(settings);
#endif
#if defined(LPS25H_5c) || defined(LPS25H_5d)
    return new RTPressureLPS25H(settings);
#endif
#if defined(MS5611_76) || defined(MS5611_77)
    return new RTPressureMS5611(settings);
#endif
    return 0;
}


RTPressure::RTPressure(RTIMUSettings *settings)
{
    m_settings = settings;
}

RTPressure::~RTPressure()
{
}
