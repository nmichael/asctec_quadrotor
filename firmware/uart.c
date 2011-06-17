#include "LPC214x.h"
#include "system.h"
#include "main.h"
#include "uart.h"
#include "irq.h"
#include "hardware.h"
#include "gpsmath.h"
#include "SSP.h"

unsigned char packets;
unsigned char DataOutputsPerSecond;
unsigned int uart_cnt;

unsigned char data_requested=0;
extern int ZeroDepth;

unsigned short current_chksum;
unsigned char chksum_to_check=0;
unsigned char chksum_trigger=1;

unsigned char transmission_running=0;
unsigned char trigger_transmission=0;

unsigned char send_buffer[16];
unsigned char *tx_buff;
unsigned char UART_syncstate=0;
unsigned char UART1_syncstate=0;
unsigned int UART_rxcount=0;
unsigned char *UART_rxptr;
unsigned int UART1_rxcount=0;
unsigned char *UART1_rxptr;

unsigned char UART_CalibDoneFlag = 0;

static volatile unsigned char rb_busy=0;

/*

//globals for NMEA parser
	double latitudeDeg;
	double longitudeDeg;
	char gprmc_string[5]="GPRMC";
	static unsigned char gpsState=GPS_IDLE;
	static unsigned char gpsCnt=0;
	static unsigned char gpsFieldStart=1;
	static unsigned char gpsFieldCnt=0;
	static unsigned char gpsInitString[5];
	static unsigned char gpsValue[20];
//<- globals
*/
unsigned char startstring[]={'>','*','>'};
unsigned char stopstring[]={'<','#','<'};

void parse_POSLLH(unsigned char, unsigned char);
void parse_POSUTM(unsigned char, unsigned char);
void parse_VELNED(unsigned char, unsigned char);
void parse_STATUS(unsigned char, unsigned char);
void parse_NAVSOL(unsigned char, unsigned char);

void check_chksum(void)
{
//	unsigned short local_chksum;
	/*if(chksum_to_check==PD_FLIGHTPARAMS)
	{

	}
*/
	chksum_to_check=0;
	chksum_trigger=0;
}

inline void parse_VELNED(unsigned char c, unsigned char reset)
{
	static unsigned char cnt=0;
	static int VE, VN, heading;
	static unsigned int sacc;

	if(reset) cnt=0;
	else
	{
		if(cnt==0) VN=c;
		else if(cnt==1) VN+=c<<8;
		else if(cnt==2) VN+=c<<16;
		else if(cnt==3) VN+=c<<24;
		else if(cnt==4) VE=c;
		else if(cnt==5) VE+=c<<8;
		else if(cnt==6) VE+=c<<16;
		else if(cnt==7) VE+=c<<24;
		else if(cnt==20) heading=c;
		else if(cnt==21) heading+=c<<8;
		else if(cnt==22) heading+=c<<16;
		else if(cnt==23) heading+=c<<24;
		else if(cnt==24) sacc=c;
		else if(cnt==25) sacc+=c<<8;
		else if(cnt==26) sacc+=c<<16;
		else if(cnt==27)
		{
			sacc+=c<<24;
			GPS_Data.speed_x=VE*10;	//convert to mm/s
			GPS_Data.speed_y=VN*10; //convert to mm/s
			GPS_Data.heading=heading/100;	//convert to deg * 1000
			GPS_Data.speed_accuracy=sacc*10;	//convert to mm/s
			gpsDataOkTrigger=1;
		}
		cnt++;
	}
}
inline void parse_POSLLH(unsigned char c, unsigned char reset)
{
	static unsigned char cnt=0;
	static int lat, lon, height;
	static unsigned int hacc, vacc;

	if(reset) cnt=0;
	else
	{
		if(cnt==0) lon=c;
		else if(cnt==1) lon+=c<<8;
		else if(cnt==2) lon+=c<<16;
		else if(cnt==3) lon+=c<<24;
		else if(cnt==4) lat=c;
		else if(cnt==5) lat+=c<<8;
		else if(cnt==6) lat+=c<<16;
		else if(cnt==7) lat+=c<<24;
		else if(cnt==12) height=c;
		else if(cnt==13) height+=c<<8;
		else if(cnt==14) height+=c<<16;
		else if(cnt==15) height+=c<<24;
		else if(cnt==16) hacc=c;
		else if(cnt==17) hacc+=c<<8;
		else if(cnt==18) hacc+=c<<16;
		else if(cnt==19) hacc+=c<<24;
		else if(cnt==20) vacc=c;
		else if(cnt==21) vacc+=c<<8;
		else if(cnt==22) vacc+=c<<16;
		else if(cnt==23)
		{
			vacc+=c<<24;
			GPS_Data.latitude=lat;
			GPS_Data.longitude=lon;
			GPS_Data.height=height;
			GPS_Data.horizontal_accuracy=hacc;
			GPS_Data.vertical_accuracy=vacc;
		}
		cnt++;
	}
}
inline void parse_POSUTM(unsigned char c, unsigned char reset)
{
	static unsigned char cnt=0;
	static int E, N;

	if(reset) cnt=0;
	else
	{
		if(cnt==0) E=c;
		else if(cnt==1) E+=c<<8;
		else if(cnt==2) E+=c<<16;
		else if(cnt==3) E+=c<<24;
		else if(cnt==4) N=c;
		else if(cnt==5) N+=c<<8;
		else if(cnt==6) N+=c<<16;
		else if(cnt==7)
		{
			N+=c<<24;
//			GPS_Data.x=E;
//			GPS_Data.y=N;
		}
		cnt++;
	}
}

inline void parse_NAVSOL(unsigned char c, unsigned char reset)
{
	static unsigned char cnt=0;
	static unsigned int tow;
	static unsigned short week;

	if(reset) cnt=0;
	else
	{
		if(cnt==0) tow=c;
		else if(cnt==1)	tow+=c<<8;
		else if(cnt==2)	tow+=c<<16;
		else if(cnt==3) tow+=c<<24;
		else if(cnt==8) week=c;
		else if(cnt==9)
		{
			week+=c<<8;
			GPS_Time.time_of_week=tow;
			GPS_Time.week=week;
		}
		else if(cnt==43)
		{
			GPS_Data.numSV=c;
		}
		cnt++;
	}
}

inline void parse_STATUS(unsigned char c, unsigned char reset)
{
	static unsigned char cnt=0;
	static unsigned char GPSfix, flags, diffs;

	if(reset) cnt=0;
	else
	{
		if(cnt==0) GPSfix=c;
		else if(cnt==1) flags=c;
		else if(cnt==2)
		{
			diffs=c;
			GPS_Data.status=GPSfix|(flags<<8)|(diffs<<16);
		}
		cnt++;
	}
}

void uart1ISR(void) __irq
{
	static unsigned char state;
	static unsigned char current_packet;
	unsigned char c;
	static unsigned char cnt, length;
  IENABLE;
  unsigned iir = U1IIR;
  // Handle UART interrupt
  switch ((iir >> 1) & 0x7)
    {
      case 1:

        break;
      case 2:
		c=U1RBR;

		//UARTWriteChar(c);

#ifndef INDOOR_GPS	//run GPS statemachine

        //parse UBX (U0RBR);

	//SSP_trans_cnt++;
		switch (state)
		{
			case 0:
				if(c==0xB5)
				{
					state=1;
				}
			break;
			case 1:
				if(c==0x62)
				{
					state=2;
				}
				else state=0;
			break;
			case 2:
				if(c==0x01)	//NAV message
				{
				 	state=3;
				}
				else state=0;
			break;
			case 3:
				current_packet=c;
				cnt=0;
				state=4;
			break;
			case 4:
				if(!cnt) length=c;
				if(++cnt==2)
				{
					cnt=0;
					state=5;
				}
			break;
			case 5:	//Four bytes ITOW
				if(++cnt==4)
				{
					cnt=0;
					state=6;
					if(current_packet==0x02) parse_POSLLH(0,1);
					//else if(current_packet==0x08) parse_POSUTM(0,1);
					else if(current_packet==0x03) parse_STATUS(0,1);
					else if(current_packet==0x12) parse_VELNED(0,1);
					else if(current_packet==0x06) parse_NAVSOL(0,1);
				}
			break;
			case 6:
				if(current_packet==0x02)
				{
					parse_POSLLH(c,0);
				}
		/*		else if(current_packet==0x08	//POSUTM currently not used
				{
					parse_POSUTM(c,0);
				}
			*/	else if(current_packet==0x03)
				{
					parse_STATUS(c,0);
				}
				else if(current_packet==0x12)
				{
					parse_VELNED(c,0);
				}
				else if(current_packet==0x06)
				{
					parse_NAVSOL(c,0);
				}
				else state=0;

				if(++cnt==length-4)
				{
					state=0;
				}
			break;
		}

#else	//run optical tracking statemachine
		switch (state)
		{
			case 0:
				if(c=='>') state=1;
			break;
			case 1:
				if(c=='*') state=2;
				else state=0;
			break;
			case 2:
				if(c=='>')	//Startstring received
				{
					UART1_rxcount=sizeof(OF_Data);
					UART1_rxptr=(unsigned char *)&OF_Data_e;
				 	state=3;
				}
				else state=0;
			break;
			case 3:
				UART1_rxcount--;
				*UART1_rxptr=c;
				UART1_rxptr++;
				if (UART1_rxcount==0)
	        	{
	             	state=0;
	             	OF_data_updated=0;
	        	}
			break;
			default:
			state=0;
			break;
		}
#endif

        break;
      case 3:
        // RLS interrupt
        break;
      case 6:
        // CTI interrupt
        break;
   }
  IDISABLE;
  VICVectAddr = 0;		/* Acknowledge Interrupt */
}



void uart0ISR(void) __irq
{
  unsigned char t;
  unsigned char UART_rxdata;

  // Read IIR to clear interrupt and find out the cause
  IENABLE;
  unsigned iir = U0IIR;
  // Handle UART interrupt
  switch ((iir >> 1) & 0x7)
    {
      case 1:
        // THRE interrupt
		if(!(IOPIN0&(1<<CTS_RADIO)))
		{
			trigger_transmission=0;
			 if (ringbuffer(RBREAD, &t, 1))
		     {
		       transmission_running=1;
		       UARTWriteChar(t);
		     }
		     else
		     {
		       transmission_running=0;
		     }
		}
		else
		{
			trigger_transmission=1;
		}

		break;

      case 2:
        // RDA interrupt
        //receive handler
        UART_rxdata = U0RBR;

        switch (UART_syncstate)
        {
        case 0:
			if (UART_rxdata=='>')
				UART_syncstate = 1;
			else UART_syncstate = 0;
			break;
        case 1:
			if (UART_rxdata=='*')
				UART_syncstate = 2;
			else UART_syncstate = 0;
			break;
        case 2:
			if (UART_rxdata=='>')
				UART_syncstate = 3;
			else UART_syncstate = 0;
			break;
        case 3:
			UART_syncstate=0;
			break;
        default:
        	UART_syncstate=0;
		}
        break;
      case 3:
        // RLS interrupt
        break;
      case 6:
        // CTI interrupt
        break;
  }
  IDISABLE;
  VICVectAddr = 0;		// Acknowledge Interrupt
 }

void UARTInitialize(unsigned int baud)
{
  unsigned int divisor = peripheralClockFrequency() / (16 * baud);

  //UART0
  U0LCR = 0x83; /* 8 bit, 1 stop bit, no parity, enable DLAB */
  U0DLL = divisor & 0xFF;
  U0DLM = (divisor >> 8) & 0xFF;
  U0LCR &= ~0x80; /* Disable DLAB */
  U0FCR = 1;
}

void UART1Initialize(unsigned int baud)
{
  unsigned int divisor = peripheralClockFrequency() / (16 * baud);
//UART1
  U1LCR = 0x83; /* 8 bit, 1 stop bit, no parity, enable DLAB */
  U1DLL = divisor & 0xFF;
  U1DLM = (divisor >> 8) & 0xFF;
  U1LCR &= ~0x80; /* Disable DLAB */
  U1FCR = 1;
}


//Write to UART0
void UARTWriteChar(unsigned char ch)
{
  while ((U0LSR & 0x20) == 0);
  U0THR = ch;
}
//Write to UART1
void UART1WriteChar(unsigned char ch)
{
  while ((U1LSR & 0x20) == 0);
  U1THR = ch;
}

unsigned char UARTReadChar(void)
{
  while ((U0LSR & 0x01) == 0);
  return U0RBR;
}

unsigned char UART1ReadChar(void)
{
  while ((U1LSR & 0x01) == 0);
  return U1RBR;
}

void __putchar(int ch)
{
  if (ch == '\n')
    UARTWriteChar('\r');
  UARTWriteChar(ch);
}

void UART_send(char *buffer, unsigned char length)
{
  unsigned char cnt=0;
  while (!(U0LSR & 0x20)); //wait until U0THR and U0TSR are both empty
  while(length--)
  {
    U0THR = buffer[cnt++];
    if(cnt>15)
    {
      while (!(U0LSR & 0x20)); //wait until U0THR is empty
    }
  }
}

void UART1_send(unsigned char *buffer, unsigned char length)
{
  unsigned char cnt=0;
  while(length--)
  {
    while (!(U0LSR & 0x20)); //wait until U0THR is empty
    U1THR = buffer[cnt++];
  }
}


void UART_send_ringbuffer(void)
{
  unsigned char t;
  if(!transmission_running)
  {
    if(ringbuffer(RBREAD, &t, 1))
    {
      transmission_running=1;
      UARTWriteChar(t);
    }
  }
}

void UART_SendPacket(void *data, unsigned short count, unsigned char packetdescriptor)
{
  unsigned short crc;
  int state;
      state=ringbuffer(RBWRITE, startstring, 3);
      state=ringbuffer(RBWRITE, (unsigned char *) &count, 2);
      state=ringbuffer(RBWRITE, &packetdescriptor, 1);
      state=ringbuffer(RBWRITE, data, count);
                crc=crc16(data,count);
      state=ringbuffer(RBWRITE, (unsigned char *) &crc, 2);
      state=ringbuffer(RBWRITE, stopstring, 3);
      UART_send_ringbuffer();
}

void mdv_output(unsigned int value) //output for Measurement Distribution Visualizer
{
    send_buffer[0]='<';
    send_buffer[1]='*';
    send_buffer[2]='>';
    send_buffer[3]=(value>>8)&0xFF;
    send_buffer[4]=value&0xFF;
    send_buffer[5]='<';
    send_buffer[6]='#';
    send_buffer[7]='>';
    ringbuffer(RBWRITE, send_buffer, 8);
    UART_send_ringbuffer();
}

unsigned short crc_update (unsigned short crc, unsigned char data)
     {
         data ^= (crc & 0xff);
         data ^= data << 4;

         return ((((unsigned short )data << 8) | ((crc>>8)&0xff)) ^ (unsigned char )(data >> 4)
                 ^ ((unsigned short )data << 3));
     }

 unsigned short crc16(void* data, unsigned short cnt)
 {
   unsigned short crc=0xff;
   unsigned char * ptr=(unsigned char *) data;
   int i;

   for (i=0;i<cnt;i++)
     {
       crc=crc_update(crc,*ptr);
       ptr++;
     }
   return crc;
 }

// no longer a ringbuffer! - now it's a FIFO
int ringbuffer(unsigned char rw, unsigned char *data, unsigned int count)	//returns 1 when write/read was successful, 0 elsewise
{
    static volatile unsigned char buffer[RINGBUFFERSIZE];
//	static volatile unsigned int pfirst=0, plast=0;	//Pointers to first and last to read byte
	static volatile unsigned int read_pointer, write_pointer;
	static volatile unsigned int content=0;
	unsigned int p=0;
    unsigned int p2=0;

	if(rw==RBWRITE)
	{
		if(count<RINGBUFFERSIZE-content)	//enough space in buffer?
		{
			while(p<count)
			{
				buffer[write_pointer++]=data[p++];
			}
            content+=count;
            return(1);
		}
	}
	else if(rw==RBREAD)
	{
		if(content>=count)
		{
			while(p2<count)
			{
				data[p2++]=buffer[read_pointer++];
			}
            content-=count;
            if(!content) //buffer empty
            {
            	write_pointer=0;
            	read_pointer=0;
            }
			return(1);
		}
	}
        else if(rw==RBFREE)
        {
          if(content) return 0;
          else return(RINGBUFFERSIZE-11);
        }

	return(0);
}

