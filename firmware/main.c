/**********************************************************
                  Header files
**********************************************************/
#include "LPC214x.h"
#include "stdio.h"
#include "main.h"
#include "system.h"
#include "uart.h"
#include "mymath.h"
#include "hardware.h"
#include "irq.h"
#include "i2c.h"
#include "gpsmath.h"
#include "adc.h"
#include "uart.h"
#include "ssp.h"
#include "LL_HL_comm.h"
#include "sdk.h"

/* *********************************************************
   Function declarations
   ********************************************************* */

void Initialize(void);
void feed(void);
void beeper(unsigned char);

/**********************************************************
                  Global Variables
**********************************************************/
struct HL_STATUS HL_Status;
struct IMU_RAWDATA IMU_RawData;
volatile unsigned int int_cnt=0, cnt=0, mainloop_cnt=0;
volatile unsigned char mainloop_trigger=0;
volatile unsigned int GPS_timeout=0;
volatile unsigned int filter_counter=0;

extern unsigned char data_requested;
extern int ZeroDepth;

volatile unsigned int trigger_cnt=0;
unsigned int logs_per_second=0, total_logs_per_second=0;

struct FILTER_DATA Filter_Data, Filter_Data_tmp;

unsigned int output_counter = 10;

struct IMU_CALCDATA IMU_CalcData, IMU_CalcData_tmp;
struct GPS_TIME GPS_Time;
struct SYSTEM_PERMANENT_DATA SYSTEM_Permanent_Data;

// Structs to send to the LL controller
struct WO_SDK_STRUCT WO_SDK;
struct WO_CTRL_INPUT WO_CTRL_Input;
struct WO_DIRECT_MOTOR_CONTROL WO_Direct_Motor_Control;

void timer0ISR(void) __irq
{
  T0IR = 0x01;      //Clear the timer 0 interrupt
  IENABLE;
  trigger_cnt++;
  if(trigger_cnt==ControllerCyclesPerSecond)
    {
      trigger_cnt=0;
      HL_Status.up_time++;
      HL_Status.cpu_load=mainloop_cnt;
      Filter_Data.cpu_load = mainloop_cnt;
      mainloop_cnt=0;
    }

  if(mainloop_trigger<10) mainloop_trigger++;

  IDISABLE;
  VICVectAddr = 0;		// Acknowledge Interrupt
}

/**********************************************************
                       MAIN
**********************************************************/
int	main (void) {
  static int vbat1, vbat2;
  int vbat;
  static int bat_cnt=0, bat_warning=1000;
  static char bat_warning_enabled=1;

#ifdef GPS_BEEP
  static unsigned int gps_beep_cnt;
#endif

  IDISABLE;

  init();
  LL_write_init();
  beeper(OFF);

  HL_Status.up_time=0;

  //printf("\n\nProgramm is running ... \n");
  //printf("Processor Clock Frequency: %d Hz\n", processorClockFrequency());
  //printf("Peripheral Clock Frequency: %d Hz\n", peripheralClockFrequency());

  IENABLE;

  LED(1,ON);

  while(1)
    {
      if(mainloop_trigger)
        {
          if(GPS_timeout<ControllerCyclesPerSecond) GPS_timeout++;
          else if(GPS_timeout==ControllerCyclesPerSecond)
            {
              GPS_timeout=ControllerCyclesPerSecond+1;
              GPS_Data.status=0;
              GPS_Data.numSV=0;
            }

          mainloop_cnt++;
          if(++bat_cnt==100) bat_cnt=0;

          //battery monitoring
          vbat1=(vbat1*29+(ADC0Read(VOLTAGE_1)*9872/579))/30;	//voltage in mV //*9872/579

          Filter_Data.battery_voltage = vbat1;

          HL_Status.battery_voltage_1=vbat1;
          HL_Status.battery_voltage_2=vbat2;

          vbat=vbat1;

          if(vbat<BATTERY_WARNING_VOLTAGE)	//decide if it's really an empty battery
            {
              if(bat_warning<ControllerCyclesPerSecond*2) bat_warning++;
              else bat_warning_enabled=1;
            }
          else
            {
              if(bat_warning>10) bat_warning-=5;
              else
                {
                  bat_warning_enabled=0;
                  beeper(OFF);//IOCLR1 = (1<<17);	//Beeper off
                }
            }
          if(bat_warning_enabled)
            {
              if(bat_cnt>((vbat-9000)/BAT_DIV)) beeper(ON);//IOSET1 = (1<<17);	//Beeper on
              else beeper(OFF);//IOCLR1 = (1<<17);		//Beeper off
            }

#ifdef GPS_BEEP
          //GPS_Beep
          if((GPS_Data.status&0xFF)!=3)	//no lock
            {
              gps_beep_cnt++;
              if(gps_beep_cnt>=1500) gps_beep_cnt=0;
              if(gps_beep_cnt<20) beeper(ON);	//IOSET1 = (1<<17);	//Beeper on
              else if(gps_beep_cnt==40) beeper(OFF);// IOCLR1 = (1<<17); //Beeper off
            }
#endif

          if(mainloop_trigger) mainloop_trigger--;
          mainloop();
        }
    }
  return 0;
}

void beeper (unsigned char offon)
{
  if(offon)	//beeper on
    {
      IOSET1 = (1<<17);
    }
  else
    {
      IOCLR1 = (1<<17);
    }
}

void mainloop(void)
{
  static unsigned char t;
#if 0
  static unsigned char led_cnt=0, led_state=1;

  led_cnt++;
  if((GPS_Data.status&0xFF)==0x03)
    {
      LED(0,OFF);
    }
  else
    {
      if(led_cnt==150)
        {
          LED(0,ON);
        }
      else if(led_cnt==200)
        {
          led_cnt=0;
          LED(0,OFF);
        }
    }
#endif

  if(trigger_transmission)
    {
      if(!(IOPIN0&(1<<CTS_RADIO)))
        {
          trigger_transmission=0;
          if(ringbuffer(RBREAD, &t, 1))
            {
              transmission_running=1;
              UARTWriteChar(t);
            }
        }
    }

  static int i = 0;
  static unsigned char *dataptr, *dataptr2;

  filter_counter++;
  if (filter_counter == output_counter)
    {
      filter_counter = 0;

      dataptr=(unsigned char *)&Filter_Data;
      dataptr2=(unsigned char *)&Filter_Data_tmp;
      for(i=0;i<sizeof(Filter_Data);i++)
        {
          *dataptr2=*dataptr;
          dataptr++;
          dataptr2++;
        }

      if((sizeof(Filter_Data_tmp))<ringbuffer(RBFREE, 0, 0))
        UART_SendPacket(&Filter_Data_tmp, sizeof(Filter_Data_tmp), PD_FILTERDATA);
    }

  HL2LL_write_cycle();	//write data to transmit buffer for immediate transfer to LL processor

#if 0
  if (gpsDataOkTrigger)
    {
      if(GPS_Data.horizontal_accuracy>12000) GPS_Data.status&=~0x03;
      if(GPS_timeout>50)//(GPS_Data.status&0xFF)!=0x03)
    	{
          if(led_state)
            {
              led_state=0;
              LED(1,OFF);
            }
          else
            {
              LED(1,ON);
              led_state=1;
            }
    	}
      GPS_timeout=0;
      HL_Status.latitude=GPS_Data.latitude;
      HL_Status.longitude=GPS_Data.longitude;

    }
#endif
}
