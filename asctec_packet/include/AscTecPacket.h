/* Data packet parsing code for use with AscTec firmware

    Aleksandr Kushleyev: Optimizations and full rewrite
    Nathan Michael: Original version

    BSD license.
    --------------------------------------------------------------------
    Copyright (c) 2011 Aleksandr Kushleyev, Nathan Michael
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    3. The name of the author may not be used to endorse or promote products
      derived from this software without specific prior written permission.

      THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
      IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
      OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
      IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
      INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
      NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
      THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef ASCTEC_PACKET_H
#define ASCTEC_PACKET_H

#include <stdio.h>
#include <stdint.h>

#define ASCTEC_PACKET_HEADER_BYTES 6
#define ASCTEC_PACKET_OVERHEAD_BYTES 11
#define ASCTEC_PACKET_MAX_SIZE      250
#define ASCTEC_PACKET_POS_LENGTH    3
#define ASCTEC_PACKET_POS_DESC      5
#define ASCTEC_PACKET_POS_PAYLOAD   6
#define ASCTEC_PACKET_MAX_DESC      8


#define ASCTEC_ERROR_NONE              0
#define ASCTEC_ERROR_UNKNOWN          -1
#define ASCTEC_ERROR_BAD_START_STRING -2
#define ASCTEC_ERROR_BAD_LENGTH       -3
#define ASCTEC_ERROR_BAD_DESCRIPTOR   -4
#define ASCTEC_ERROR_BAD_CHECKSUM     -5
#define ASCTEC_ERROR_BAD_STOP_STRING  -6


const char ASCTEC_PACKET_START_STRING[4] = ">*>";
const char ASCTEC_PACKET_STOP_STRING[4]  = "<#<";

typedef struct
{
  uint8_t buffer[ASCTEC_PACKET_MAX_SIZE];  //buffer for incoming data
  uint8_t lenReceived;                     //number of bytes successfully received
  uint8_t lenExpected;                     //expected length of packet
  uint8_t * bp;                            //current write pointer in the buffer
} AscTecPacket;


//initialize the packet data structure
void AscTecPacketInit(AscTecPacket * packet);

//process one character and returns an error/success code
int16_t AscTecPacketProcessChar(uint8_t c, AscTecPacket * packet);

//get the lenght of data structure that was sent in the packet
uint16_t AscTecPacketGetLength(AscTecPacket * packet);

//get the packet descriptor
uint8_t AscTecPacketGetDescriptor(AscTecPacket * packet);

//get the packet payload
void * AscTecPacketGetPayload(AscTecPacket * packet);

//get the payload checksum
uint16_t AscTecPacketGetPayloadChecksum(AscTecPacket * packet);

//update the checksum
inline uint16_t AscTecPacketCRCUpdate(uint16_t crc, uint8_t data);

//calculate the checksum
inline uint16_t AscTecPacketCRC16(void* data, uint16_t cnt);

//return the error message
char* AscTecPacketPrintError(int16_t code);


#endif //ASCTEC_PACKET_H

