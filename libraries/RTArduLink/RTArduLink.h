///////////////////////////////////////////////////////////
//
//  This file is part of RTArduLink
//
//  Copyright (c) 2014-2015 richards-tech
//
//  Permission is hereby granted, free of charge,
//  to any person obtaining a copy of
//  this software and associated documentation files
//  (the "Software"), to deal in the Software without
//  restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and
//  to permit persons to whom the Software is furnished
//  to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice
//  shall be included in all copies or substantial portions
//  of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF
//  ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
//  TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
//  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
//  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
//  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.

#ifndef _RTARDULINK_H
#define _RTARDULINK_H

#include "RTArduLinkDefs.h"
#include "RTArduLinkHAL.h"

#define	RTARDULINK_HOST_PORT                0               // host port is always 0
#define	RTARDULINK_DAISY_PORT               1               // daisy chain port is always 1

typedef struct
{
    int index;                                              // port index
    bool inUse;                                             // true if in use
    RTARDULINK_RXFRAME RXFrame;                             // structure to maintain receive frame state
    RTARDULINK_FRAME RXFrameBuffer;                         // used to assemble received frames
    RTARDULINKHAL_PORT portHAL;                             // the actual hardware port interface
} RTARDULINK_PORT;

class RTArduLink
{
public:
    RTArduLink();
    virtual ~RTArduLink();

    void begin(const char *identitySuffix);                 // should be called in setup() code
    void background();                                      // should be called once per loop()
    void sendDebugMessage(const char *debugMesssage);       // sends a debug message to the host port
    void sendMessage(unsigned char messageType, unsigned char messageParam,
        unsigned char *data, int length);                   // sends a message to the host port

protected:
//  These are functions that can be overridden

    virtual void processCustomMessage(unsigned char messageType, 
        unsigned char messageParam, unsigned char *data, int dataLength) {}

    RTARDULINK_PORT m_ports[RTARDULINKHAL_MAX_PORTS];       // port array
    RTARDULINK_PORT *m_hostPort;                            // a link to the entry for the host port


private:
    void processReceivedMessage(RTARDULINK_PORT *port);     // process a completed message
    void processHostMessage();                              // special case for stuff received from the host port
    void sendFrame(RTARDULINK_PORT *portInfo, RTARDULINK_FRAME *frame, int length);	// send a frame to the host. length is length of data field

    const char *m_identitySuffix;                           // what to add to the EEPROM identity string

};

#endif // _RTARDULINK_H

