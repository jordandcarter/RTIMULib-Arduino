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

#ifndef _RTARDULINKHAL_H
#define _RTARDULINKHAL_H


//----------------------------------------------------------
//  Target-specific includes
//
//
//  Arduino HAL

#include <RTArduLinkDefs.h>
#include <HardwareSerial.h>
#include <EEPROM.h>

#define	RTARDULINKHAL_MAX_SUBSYSTEM_PORTS	3               // maximum number of subsystem ports
#define RTARDULINKHAL_MAX_PORTS             (RTARDULINKHAL_MAX_SUBSYSTEM_PORTS + 1)	// max total ports (including host)
#define RTARDULINKHAL_EEPROM_OFFSET         256             // where the config starts in EEPROM

//  RTARDULINKHAL_PORT should be modified as appropriate for the target.
//  There is one copy of this per port. It contains all state needed about
//  a serial port.

typedef struct
{
    HardwareSerial *serialPort;                             // the serial port structure
} RTARDULINKHAL_PORT;

//  RTARDULINKHAL_EEPROM is the target-specific structure used to
//  store configs in EEPROM

//  Signature bytes indicating valid config

#define RTARDULINKHAL_SIG0                  0x38          
#define RTARDULINKHAL_SIG1                  0xc1           

typedef struct
{
    unsigned char sig0;                                     // signature byte 0
    unsigned char sig1;                                     // signature byte 1
    char identity[RTARDULINK_DATA_MAX_LEN];                 // identity string
    unsigned char portSpeed[RTARDULINKHAL_MAX_PORTS];       // port speed codes
    unsigned char hardwarePort[RTARDULINKHAL_MAX_PORTS];    // port number for hardware serial
} RTARDULINKHAL_EEPROM;

//  The global config structure

extern RTARDULINKHAL_EEPROM RTArduLinkHALConfig;



//----------------------------------------------------------
//
// These functions must be provided the RTArduLinkHAL for all implementations

//  RTArduLinkHALConfigurePort() activates the specified port configuration specified by portIndex in port structure port.

    bool RTArduLinkHALConfigurePort(RTARDULINKHAL_PORT *port, int portIndex);


//  RTArduLinkHALPortAvailable() returns the number of bytes availabel on the specified port.

    int RTArduLinkHALPortAvailable(RTARDULINKHAL_PORT *port);


//  RTArduLinkHALPortRead() returns the next available byte from a port. Always check available bytes first

    unsigned char RTArduLinkHALPortRead(RTARDULINKHAL_PORT *port);


//  RTArduLinkHALPortWrite() writes length bytes of the block pointed to by data to the specified port.

    void RTArduLinkHALPortWrite(RTARDULINKHAL_PORT *port, unsigned char *data, unsigned char length);


//  RTArduLinkHALEEPROMValid() returns true if the EEPROM contains a valid configuration,
//  false otherwise.

    bool RTArduLinkHALEEPROMValid();                        // returns true if a valid config


//  RTArduLinkHALEEPROMDisplay() displays the current configuration

    void RTArduLinkHALEEPROMDisplay();                      // display the config


//  RTArduLinkHALEEPROMDisplayPort() displays the configuration for a single port
//  If suppress is true, nothing is displayed if the port is not enabled. If false
//  the port's data will be displayed regardless.

    void RTArduLinkHALEEPROMDisplayPort(int port, bool suppress);   // display the port config


//  RTArduLinkHALEEPROMDefault() writes a default config to EEPROM

    void RTArduLinkHALEEPROMDefault();                      // write and load default settings


//  RTArduLinkHALEEPROMRead() loads the EEPROM config into the RTArduLinkHALConfig
//  global structure.

    void RTArduLinkHALEEPROMRead();                         // to load the config


//  RTArduLinkHALEEPROMWrite() writes the config in the RTArduLinkHALConfig
//  global structure back to EEPROM.

    void RTArduLinkHALEEPROMWrite();                        // to write the config


#endif // _RTARDULINKHAL_H

