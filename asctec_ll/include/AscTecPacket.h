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

