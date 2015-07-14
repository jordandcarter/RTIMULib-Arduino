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

#include <string.h>
#include "RTArduLinkHAL.h"

//----------------------------------------------------------
//
//  Arduino HAL


//  The global config structure

RTARDULINKHAL_EEPROM RTArduLinkHALConfig;

bool RTArduLinkHALAddHardwarePort(RTARDULINKHAL_PORT *port, long portSpeed, unsigned char hardwarePort);

//  Port speed map array

unsigned long RTArduLinkHALSpeedMap[] = {0, 9600, 19200, 38400, 57600, 115200};


bool RTArduLinkHALConfigurePort(RTARDULINKHAL_PORT *port, int portIndex)
{
    if (RTArduLinkHALConfig.portSpeed[portIndex] == RTARDULINK_PORT_SPEED_OFF)
        return false;                                       // port is not enabled

    return RTArduLinkHALAddHardwarePort(port, RTArduLinkHALSpeedMap[RTArduLinkHALConfig.portSpeed[portIndex]],
                    RTArduLinkHALConfig.hardwarePort[portIndex]);
}

int RTArduLinkHALPortAvailable(RTARDULINKHAL_PORT *port)
{
    return port->serialPort->available();
}

unsigned char RTArduLinkHALPortRead(RTARDULINKHAL_PORT *port)
{
    return port->serialPort->read();
}

void RTArduLinkHALPortWrite(RTARDULINKHAL_PORT *port, unsigned char *data, unsigned char length)
{
    port->serialPort->write(data, length);
}


bool RTArduLinkHALAddHardwarePort(RTARDULINKHAL_PORT *port, long portSpeed, unsigned char hardwarePort)
{
    HardwareSerial *hardPort;

    switch (hardwarePort) {
        case 0:
#if defined(USBCON)
            /* Leonardo support */
            hardPort = &Serial1;
#else
            hardPort = &Serial;
#endif
            break;

        case 1:
#if defined(UBRR1H)
            hardPort = &Serial1;
#else
            return false;
#endif
            break;

        case 2:
#if defined(UBRR2H)
            hardPort = &Serial2;
#else
            return false;
#endif
            break;

        case 3:
#if defined(UBRR3H)
            hardPort = &Serial3;
#else
            return false;
#endif
            break;

        default:
            return false;
    }

    port->serialPort = hardPort;
    hardPort->begin(portSpeed);                             // start the port
    return true;
}


bool RTArduLinkHALEEPROMValid()
{
    RTArduLinkHALEEPROMRead();                              // see what it really is
    return (RTArduLinkHALConfig.sig0 == RTARDULINKHAL_SIG0) && 
        (RTArduLinkHALConfig.sig1 == RTARDULINKHAL_SIG1);
}

void RTArduLinkHALEEPROMDisplay()
{
    Serial.println();

    if ((RTArduLinkHALConfig.sig0 != RTARDULINKHAL_SIG0) || 
        (RTArduLinkHALConfig.sig1 != RTARDULINKHAL_SIG1)) {
        Serial.println("Invalid config");
        return;
    }
    Serial.print("Identity: ");
    Serial.println(RTArduLinkHALConfig.identity);

    for (int i = 0; i < RTARDULINKHAL_MAX_PORTS; i++)
        RTArduLinkHALEEPROMDisplayPort(i, true);
}

void RTArduLinkHALEEPROMDisplayPort(int index, bool suppress)
{
    if (suppress && (RTArduLinkHALConfig.portSpeed[index] == RTARDULINK_PORT_SPEED_OFF))
        return;
    Serial.print("Port index ");
    Serial.print(index);
    Serial.print(" speed=");
    Serial.print(RTArduLinkHALConfig.portSpeed[index]);
    Serial.print(", ");
    Serial.print("hardware port number=");
    Serial.println(RTArduLinkHALConfig.hardwarePort[index]);
}


void RTArduLinkHALEEPROMDefault()
{
    RTArduLinkHALConfig.sig0 = RTARDULINKHAL_SIG0;       // set to valid signature
    RTArduLinkHALConfig.sig1 = RTARDULINKHAL_SIG1;                        
    strcpy(RTArduLinkHALConfig.identity, "RTArduLink_Arduino");

    RTArduLinkHALConfig.portSpeed[0] = RTARDULINK_PORT_SPEED_115200;
    for (int i = 1; i < RTARDULINKHAL_MAX_PORTS; i++)
        RTArduLinkHALConfig.portSpeed[i] = RTARDULINK_PORT_SPEED_OFF;

    for (int i = 0; i < RTARDULINKHAL_MAX_PORTS; i++)
        RTArduLinkHALConfig.hardwarePort[i] = i;

    RTArduLinkHALEEPROMWrite();
}

void RTArduLinkHALEEPROMRead()
{
    unsigned char *data;

    data = (unsigned char *)&RTArduLinkHALConfig;

    for (int i = 0; i < (int)sizeof(RTARDULINKHAL_EEPROM); i++)
        *data++ = EEPROM.read(i + RTARDULINKHAL_EEPROM_OFFSET);
}

void RTArduLinkHALEEPROMWrite()
{
    unsigned char *data;

    data = (unsigned char *)&RTArduLinkHALConfig;

    for (int i = 0; i < (int)sizeof(RTARDULINKHAL_EEPROM); i++)
        EEPROM.write(i + RTARDULINKHAL_EEPROM_OFFSET, *data++);
}
