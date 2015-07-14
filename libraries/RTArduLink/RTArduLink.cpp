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

#include "RTArduLink.h"
#include "RTArduLinkHAL.h"
#include "RTArduLinkUtils.h"

#include <string.h>

RTArduLink::RTArduLink()
{
}

RTArduLink::~RTArduLink()
{
}

void RTArduLink::begin(const char *identitySuffix)
{
    RTARDULINK_PORT *portInfo;

    if (!RTArduLinkHALEEPROMValid())
        RTArduLinkHALEEPROMDefault();

    m_identitySuffix = identitySuffix;

    // now set up host and subsystem ports based on EEPROM configuration

    for (int i = 0; i < RTARDULINKHAL_MAX_PORTS; i++) {
        portInfo = m_ports + i;
        portInfo->index = i;
        portInfo->inUse = RTArduLinkHALConfigurePort(&(portInfo->portHAL), i);
        RTArduLinkRXFrameInit(&(portInfo->RXFrame), &(portInfo->RXFrameBuffer));
    }
    m_hostPort = m_ports;
}


void RTArduLink::background()
{
    unsigned char index;
    RTARDULINK_PORT *portInfo;

    for (index = 0; index < RTARDULINKHAL_MAX_PORTS; index++) {
        portInfo = m_ports + index;
        if (!portInfo->inUse)
            continue;

        while (RTArduLinkHALPortAvailable(&(portInfo->portHAL))) {
            if (!RTArduLinkReassemble(&(portInfo->RXFrame), RTArduLinkHALPortRead(&(portInfo->portHAL)))) {
                sendDebugMessage("Reassembly error");
            } else {
                if (portInfo->RXFrame.complete) {
                    processReceivedMessage(portInfo);
                    RTArduLinkRXFrameInit(&(portInfo->RXFrame), &(portInfo->RXFrameBuffer));
                }
            }
        }
    }
}

void RTArduLink::processReceivedMessage(RTARDULINK_PORT *portInfo)
{
    RTARDULINK_MESSAGE *message;                            // a pointer to the message part of the frame
    unsigned int address;

    message = &(portInfo->RXFrameBuffer.message);           // get the message pointer
    address = RTArduLinkConvertUC2ToUInt(message->messageAddress);

    switch (portInfo->index) {
        case  RTARDULINK_HOST_PORT:
            processHostMessage();                           // came from this upstream link
            return;

        case RTARDULINK_DAISY_PORT:                         // came from dasiy chain port
            if (address != RTARDULINK_HOST_PORT)            // true if it came from a daisy chained subsystem, not a directly connected subsystem
                RTArduLinkConvertIntToUC2(address + RTARDULINKHAL_MAX_PORTS, message->messageAddress);
             else
                RTArduLinkConvertIntToUC2(RTARDULINK_DAISY_PORT, message->messageAddress);
            break;

        default:
            RTArduLinkConvertIntToUC2(address + portInfo->index, message->messageAddress);
            break;
    }

    // if get here, need to forward to host port

    sendFrame(m_hostPort, &(portInfo->RXFrameBuffer), portInfo->RXFrameBuffer.messageLength);
}


void RTArduLink::processHostMessage()
{
    RTARDULINK_MESSAGE *message;                            // a pointer to the message part of the frame
    int identityLength;
    int suffixLength;
    unsigned int address;

    message = &(m_hostPort->RXFrameBuffer.message);         // get the message pointer
    address = RTArduLinkConvertUC2ToUInt(message->messageAddress);

    if (address == RTARDULINK_BROADCAST_ADDRESS) {          // need to forward to downstream ports also
        for (int i = RTARDULINK_HOST_PORT + 1; i < RTARDULINKHAL_MAX_PORTS; i++) {
            if (m_ports[i].inUse)
                sendFrame(m_ports + i, &(m_hostPort->RXFrameBuffer), m_hostPort->RXFrameBuffer.messageLength);
        }
    }
    if ((address == RTARDULINK_MY_ADDRESS) || (address == RTARDULINK_BROADCAST_ADDRESS)) {  // it's for me
        switch (message->messageType)
        {
            case RTARDULINK_MESSAGE_POLL:
            case RTARDULINK_MESSAGE_ECHO:
                RTArduLinkConvertIntToUC2(RTARDULINK_MY_ADDRESS, message->messageAddress);
                sendFrame(m_hostPort, &(m_hostPort->RXFrameBuffer), m_hostPort->RXFrameBuffer.messageLength);   // just send the frame back as received
                break;

            case RTARDULINK_MESSAGE_IDENTITY:
                identityLength = strlen(RTArduLinkHALConfig.identity);
                suffixLength = strlen(m_identitySuffix);

                memcpy(message->data, RTArduLinkHALConfig.identity, identityLength + 1);    // copy in identity

                if ((identityLength + suffixLength) < RTARDULINK_DATA_MAX_LEN - 1) {
                    memcpy(message->data + identityLength, m_identitySuffix, suffixLength + 1); // copy in suffix
                } else {
                    suffixLength = 0;
                }
                RTArduLinkConvertIntToUC2(RTARDULINK_MY_ADDRESS, message->messageAddress);
                message->data[RTARDULINK_DATA_MAX_LEN - 1] = 0;     // make sure zero terminated if it was truncated
                sendFrame(m_hostPort, &(m_hostPort->RXFrameBuffer), RTARDULINK_MESSAGE_HEADER_LEN + identityLength + suffixLength + 1);
                break;

            default:
                if (message->messageType < RTARDULINK_MESSAGE_CUSTOM) {	// illegal code
                    message->data[0] = RTARDULINK_RESPONSE_ILLEGAL_COMMAND;
                    message->data[1] = message->messageType;        // this is the offending code
                    message->messageType = RTARDULINK_MESSAGE_ERROR;
                    RTArduLinkConvertIntToUC2(RTARDULINK_MY_ADDRESS, message->messageAddress);
                    sendFrame(m_hostPort, &(m_hostPort->RXFrameBuffer), RTARDULINK_MESSAGE_HEADER_LEN + 2);
                    break;
                }
                processCustomMessage(message->messageType, message->messageParam, message->data,
                        m_hostPort->RXFrameBuffer.messageLength - RTARDULINK_MESSAGE_HEADER_LEN);	// see if anyone wants to process it
                break;
        }
        return;
    }

    if (address >= RTARDULINKHAL_MAX_PORTS) {               // need to pass it to the first subsystem
        if (!m_ports[RTARDULINK_DAISY_PORT].inUse)
            return;                                         // there is no daisy chain port
        RTArduLinkConvertIntToUC2(address - RTARDULINKHAL_MAX_PORTS, message->messageAddress); // adjust the address
        sendFrame(m_ports +RTARDULINK_DAISY_PORT, &(m_hostPort->RXFrameBuffer), m_hostPort->RXFrameBuffer.messageLength);
        return;
    }

    // if get here, needs to go to a local subsystem port

    if (m_ports[address].inUse) {
        RTArduLinkConvertIntToUC2(0, message->messageAddress);     // indicates that the target should process it
        sendFrame(m_ports + address, &(m_hostPort->RXFrameBuffer), m_hostPort->RXFrameBuffer.messageLength);
    }
}

void RTArduLink::sendDebugMessage(const char *debugMessage)
{
    RTARDULINK_FRAME frame;
    int stringLength;

    stringLength = strlen(debugMessage);
    if (stringLength >= RTARDULINK_DATA_MAX_LEN)
        stringLength = RTARDULINK_DATA_MAX_LEN-1;
    memcpy(frame.message.data, debugMessage, stringLength);
    frame.message.data[stringLength] = 0;
    frame.message.messageType = RTARDULINK_MESSAGE_DEBUG;
    RTArduLinkConvertIntToUC2(RTARDULINK_MY_ADDRESS, frame.message.messageAddress);
    sendFrame(m_hostPort, &frame, RTARDULINK_MESSAGE_HEADER_LEN + stringLength + 1);
}

void RTArduLink::sendMessage(unsigned char messageType, unsigned char messageParam, unsigned char *data, int length)
{
    RTARDULINK_FRAME frame;
  
    RTArduLinkConvertIntToUC2(RTARDULINK_MY_ADDRESS, frame.message.messageAddress);
    frame.message.messageType = messageType;
    frame.message.messageParam = messageParam;

    if (length > RTARDULINK_DATA_MAX_LEN)
        length = RTARDULINK_DATA_MAX_LEN;
    memcpy(frame.message.data, data, length);

    sendFrame(m_hostPort, &frame, length + RTARDULINK_MESSAGE_HEADER_LEN);
}

void RTArduLink::sendFrame(RTARDULINK_PORT *portInfo, RTARDULINK_FRAME *frame, int length)
{
    frame->sync0 = RTARDULINK_MESSAGE_SYNC0;
    frame->sync1 = RTARDULINK_MESSAGE_SYNC1;
    frame->messageLength = length;                          // set length
    RTArduLinkSetChecksum(frame);                           // compute checksum
    RTArduLinkHALPortWrite(&(portInfo->portHAL), (unsigned char *)frame, frame->messageLength + RTARDULINK_FRAME_HEADER_LEN);
}

