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

#include "AscTecPacket.h"

//initialize the packet data structure
void AscTecPacketInit(AscTecPacket * packet)
{
  packet->lenReceived = 0;
  packet->lenExpected = 0;
  packet->bp          = NULL;
}


//get the lenght of data structure that was sent in the packet
uint16_t AscTecPacketGetLength(AscTecPacket * packet)
{
  return *(uint16_t*)&packet->buffer[ASCTEC_PACKET_POS_LENGTH];
}

//get the packet descriptor
uint8_t AscTecPacketGetDescriptor(AscTecPacket * packet)
{
  return packet->buffer[ASCTEC_PACKET_POS_DESC];
}

//get the packet payload
void * AscTecPacketGetPayload(AscTecPacket * packet)
{
  return &(packet->buffer[ASCTEC_PACKET_POS_PAYLOAD]);
}

//get the payload checksum
uint16_t AscTecPacketGetPayloadChecksum(AscTecPacket * packet)
{
  return *(uint16_t*)&packet->buffer[AscTecPacketGetLength(packet)
                                    +ASCTEC_PACKET_HEADER_BYTES];
}

//process one char for checksum
uint16_t AscTecPacketCRCUpdate(uint16_t crc, uint8_t data)
{
  data ^= (crc & 0xff);
  data ^= data << 4;
 
  return ((((uint16_t)data << 8) | ((crc>>8) & 0xff)) ^
          (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}

//calculate checksum for the full packet
uint16_t AscTecPacketCRC16(void* data, uint16_t cnt)
{
  uint16_t crc = 0xff;
  uint8_t *ptr = (uint8_t *)data;

  int i = 0;
  for (i = 0; i < cnt; i++)
    crc = AscTecPacketCRCUpdate(crc, *ptr++); 
  
  return crc;
}

//process one character and returns an error/success code
int16_t AscTecPacketProcessChar(uint8_t c, AscTecPacket * packet)
{
  int16_t ret = ASCTEC_ERROR_NONE;
  uint16_t checksum   = 0;

  switch (packet->lenReceived)
  {
    case 0:
      packet->bp = packet->buffer;    //reset the pointer for storing data

    case 1:
    case 2:
      if (c != ASCTEC_PACKET_START_STRING[packet->lenReceived])
      {
        packet->lenReceived = 0;
        ret = ASCTEC_ERROR_BAD_START_STRING;
        break;
      }

      packet->lenReceived++;          //increment the received byte length
      *(packet->bp)++ = c;            //store the value into the buffer
      break;

    case 3:   //get the length
    case 4:
      packet->lenReceived++;          //increment the received byte length
      *(packet->bp)++ = c;
      break;

    case 5:   //check the length and get and check the descriptor
      packet->lenExpected = AscTecPacketGetLength(packet) +
                            ASCTEC_PACKET_OVERHEAD_BYTES;

      //check the length
      if (packet->lenExpected > ASCTEC_PACKET_MAX_SIZE)
      {      
        packet->lenReceived = 0;
        ret = ASCTEC_ERROR_BAD_LENGTH;
        break;
      }

      packet->lenReceived++;          //increment the received byte length
      *(packet->bp)++ =c;             //store the descriptor
      
      //check the descriptor
      if (AscTecPacketGetDescriptor(packet) > ASCTEC_PACKET_MAX_SIZE)
      {      
        packet->lenReceived = 0;      //reset the length if error
        ret = ASCTEC_ERROR_BAD_DESCRIPTOR;
      }
      break;

    default:    //read off the payload, checksum and the stop string
      packet->lenReceived++;          //increment the received byte length
      *(packet->bp)++ = c;            //store the payload byte

      if (packet->lenReceived < packet->lenExpected)
        break;  //have not received enough yet
      
      //calculate the checksum
      checksum = AscTecPacketCRC16(AscTecPacketGetPayload(packet),
                                   AscTecPacketGetLength(packet));

      //verify the checksum
      if (checksum != AscTecPacketGetPayloadChecksum(packet))
      {      
        packet->lenReceived = 0;
        ret = ASCTEC_ERROR_BAD_CHECKSUM;
        break;
      }

      //skip verifying the stop string
      /*
      //verify the stop string
      if (strncmp(&packet->buffer(packet->lenReceived-2),
                  ASCTEC_PACKET_STOP_STRING, 3) != 0)
      {
        packet->lenReceived = 0;
        ret = ASCTEC_ERROR_BAD_STOP_STRING;
        break;
      }
      */
      
      //set the return value to total number of chars received
      ret = packet->lenReceived;

      //reset the char counter so that we start over next time around
      packet->lenReceived = 0;
  }

  return ret;
}


//return the error message
char* AscTecPacketPrintError(int16_t code)
{
  char * message;

  switch (code)
  {
    case ASCTEC_ERROR_NONE:
      message = (char*)"no error";
      break;

    case ASCTEC_ERROR_UNKNOWN:
      message = (char*)"unknown error";
      break;

    case ASCTEC_ERROR_BAD_START_STRING:
      message = (char*)"bad start string";
      break;

    case ASCTEC_ERROR_BAD_LENGTH:
      message = (char*)"bad length";
      break;

    case ASCTEC_ERROR_BAD_DESCRIPTOR:
      message = (char*)"bad descriptor";
      break;

    case ASCTEC_ERROR_BAD_CHECKSUM:
      message = (char*)"bad checksum";
      break;

    case ASCTEC_ERROR_BAD_STOP_STRING:
      message = (char*)"bad stop string";
      break;

    default:
      message = (char*)"unknown error code";
      break;
  }

  return message;
}


