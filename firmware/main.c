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
struct STATUS_DATA Status_Data, Status_Data_tmp;

unsigned int status_counter = 0;
unsigned int output_counter = 10;

struct IMU_CALCDATA IMU_CalcData, IMU_CalcData_tmp;
struct GPS_TIME GPS_Time;
struct SYSTEM_PERMANENT_DATA SYSTEM_Permanent_Data;

// The CTRL_Input coming from the UART
struct CTRL_INPUT CTRL_Input, CTRL_Input_tmp;

// The PD_Input coming from the UART
struct PD_INPUT PD_Input, PD_Input_tmp;

#if 0
// Default values for DMC when not receiving commands
float kp_roll_safety = 2.213;
float kd_roll_safety = 0.1396;
float kp_pitch_safety = 2.213;
float kd_pitch_safety = 0.1396;
float kd_yaw_safety = 0.1396;

float roll_des_safety = 0;
float pitch_des_safety = 0;
float yaw_delta_safety = 0;
float thrust_safety = 0;
#endif

// Direct motor control gains
float kp_roll = 0;
float kd_roll = 0;
float kp_pitch = 0;
float kd_pitch = 0;
float kd_yaw = 0;

// The control values
float roll_des = 0;
float pitch_des = 0;
float yaw_delta_des = 0;
unsigned char thrust_des = 0;

float roll = 0;
float pitch = 0;
short yaw_delta = 0;
short thrust = 0;

//dmel change
float p_des = 0;
float q_des = 0;
float r_des = 0;
short roll_delta = 0;
short pitch_delta = 0;

long mypitch = 0;
long myroll = 0;
short myangvel_pitch = 0;
short myangvel_roll = 0;
short thrust_p = 0;
short thrust_q = 0;
short roll_q = 0;
short pitch_p = 0;

short backintlength = -1;
short backintpitch[40];
short backintroll[40];
short intcntr = 0;
short recflag = 0;
short newincpitch;
short newincroll;

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
#ifdef PELICAN
  	Filter_Data.cpu_load = mainloop_cnt;
#else
  	Status_Data.cpu_load = mainloop_cnt;
#endif

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

  //dmel change
for (int i = 0; i<40; i++)
{
	backintpitch[i]=0;
	backintroll[i]=0;
}


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

  printf("\n\nProgramm is running ... \n");
  printf("Processor Clock Frequency: %d Hz\n", processorClockFrequency());
  printf("Peripheral Clock Frequency: %d Hz\n", peripheralClockFrequency());

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

#ifdef PELICAN
        Filter_Data.battery_voltage = vbat1;
#else
        Status_Data.battery_voltage = vbat1;
#endif

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

float normalize_deg (float angle_diff)
{
	while(angle_diff>=180)
	{
		angle_diff-=360;
	}
	while(angle_diff<(-180))
	{
		angle_diff+=360;
	}
	return angle_diff;
}

void mainloop(void)
{
    static unsigned char led_cnt=0, led_state=1;
	static unsigned char t;

    led_cnt++;
#if 0
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
	static short chksum = 0;

	if (PD_Input_updated)
	{
		dataptr=(unsigned char *)&PD_Input;
		dataptr2=(unsigned char *)&PD_Input_tmp;
		for(i=0;i<sizeof(PD_Input);i++)
		{
			*dataptr=*dataptr2;
			dataptr++;
			dataptr2++;
		}
		PD_Input_updated=0;

		chksum = PD_Input.roll + PD_Input.pitch + PD_Input.yaw_delta + 0xAAAA;

		if (chksum == PD_Input.chksum)
		{
			kp_roll = PD_Input.kp_roll/1e3;
			kd_roll = PD_Input.kd_roll/1e3;
			kp_pitch = PD_Input.kp_pitch/1e3;
			kd_pitch = PD_Input.kd_pitch/1e3;
			kd_yaw = PD_Input.kd_yaw/1e3;

			roll_des = PD_Input.roll/1e2;

			// Reversing pitch, yaw to match convention
			// XYZ , x forward, y left, z up
			// Rot = Rz(yaw)*Rx(roll)*Ry(pitch)
			// Rot is rotation from body frame to world frame
			pitch_des = -PD_Input.pitch/1e2;
			yaw_delta_des = -PD_Input.yaw_delta/1e2;
			thrust_des = PD_Input.thrust;

			// dmel change
			p_des = PD_Input.p_des/10;
			q_des = -PD_Input.q_des/10;
			r_des = PD_Input.r_des/10;

			roll_delta = PD_Input.roll_delta;
			pitch_delta = PD_Input.pitch_delta;
			mypitch = PD_Input.vicon_pitch*8;
			myroll = PD_Input.vicon_roll*8;
			backintlength = PD_Input.vicon_bil;

			if (backintlength==-1)
			{
				thrust_p = PD_Input.vicon_roll;
				thrust_q = PD_Input.vicon_pitch;
			}
			else if (backintlength==-2)
			{
				roll_q = PD_Input.vicon_roll;
				pitch_p = PD_Input.vicon_pitch;
			}

			recflag = 1;
#if 0
			if((sizeof(PD_Input))<ringbuffer(RBFREE, 0, 0))
				UART_SendPacket(&PD_Input, sizeof(PD_Input), PD_PDINPUT);
#endif
		}

	}

	// Conversion from angle units (100 = 1 deg) to deg
	// Conversion from angvel units (0.0154 deg/s) to deg/s
	if (backintlength < 1)
	{
		//this uses imu angle estimates
		roll = 100.5 + kp_roll*(normalize_deg(roll_des + Filter_Data.angle_roll/1e2)) +
		kd_roll*(p_des + Filter_Data.angvel_roll*0.0154) + roll_delta +
		roll_q*(Filter_Data.angvel_pitch*0.000077);

		pitch = 100.5 - kp_pitch*(normalize_deg(pitch_des + Filter_Data.angle_pitch/1e2))
		-kd_pitch*(q_des+Filter_Data.angvel_pitch*0.0154)+pitch_delta -
		pitch_p*(Filter_Data.angvel_roll*0.000077);

		//divide it by 200, 0.0154/200 = 0.000077
		thrust = thrust_des - thrust_p*Filter_Data.angvel_roll*0.000077
		+ thrust_q*Filter_Data.angvel_pitch*0.000077;
	}
	else
	{
		//this uses vicon+gyro angle estimates
		roll = 100.5 + kp_roll*(normalize_deg(roll_des - myroll/8e2)) +
		kd_roll*(p_des + Filter_Data.angvel_roll*0.0154) + roll_delta;
		pitch = 100.5 - kp_pitch*(normalize_deg(pitch_des + mypitch/8e2))
		-kd_pitch*(q_des+Filter_Data.angvel_pitch*0.0154)+ pitch_delta;
		thrust = thrust_des;
	}

	if (thrust < 0)
		thrust = 0;
	else if (thrust > 200)
		thrust = 200;

	if (roll < 0)
		roll = 0;
	else if (roll > 200)
		roll = 200;

	if (pitch < 0)
		pitch = 0;
	else if (pitch > 200)
		pitch = 200;

	yaw_delta = 100.5 - yaw_delta_des + kd_yaw*(r_des + Filter_Data.angvel_yaw*0.0154);
	if (yaw_delta < 0)
		yaw_delta = 0;
	else if (yaw_delta > 200)
		yaw_delta = 200;

	WO_SDK.ctrl_enabled=1;
	WO_SDK.ctrl_mode=0x01;
	WO_Direct_Motor_Control.thrust = thrust;
	WO_Direct_Motor_Control.roll = roll;
	WO_Direct_Motor_Control.pitch = pitch;
	WO_Direct_Motor_Control.yaw = yaw_delta;

	// dmel change
#if 1
	//do we just assume it runs at 1000 Hz?
	//my pitch is an signed int
	//now lets us pitch delta to reset mypitch everytime it comes in
	myangvel_pitch = angvel_pitchnew;
	if (myangvel_pitch>0)
		newincpitch = (myangvel_pitch+39)/78;
	else
		newincpitch = (myangvel_pitch-39)/78;

	backintpitch[intcntr]=newincpitch;

#if VICON_ROLL
	myangvel_roll = -angvel_rollnew; //seems the roll is flipped
	if (myangvel_roll>0)
		newincroll = (myangvel_roll+39)/78;
	else
		newincroll = (myangvel_roll-39)/78;
	backintroll[intcntr]=newincroll;
#endif

	if(recflag)
	{
		for (i =0; i<backintlength; i++)
		{
			mypitch += backintpitch[i];
#if VICON_ROLL
			myroll += backintroll[i];
#endif
		}
		recflag=0;
	}
	else
	{
		mypitch += newincpitch;
#if VICON_ROLL
		myroll += newincroll;
#endif
	}

	if(mypitch>144000)
		mypitch-=288000;
	if(mypitch<-144000)
		mypitch+=288000;

#if VICON_ROLL
	if(myroll>144000)
		myroll-=288000;
	if(myroll<-144000)
		myroll+=288000;
#endif

	intcntr++;
	if (intcntr>=backintlength)
	{
		intcntr=0;
	}

#endif

#ifdef PELICAN
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
#else
	status_counter++;
	if (status_counter == ControllerCyclesPerSecond)
	{
		status_counter = 0;

		dataptr=(unsigned char *)&Status_Data;
		dataptr2=(unsigned char *)&Status_Data_tmp;
		for(i=0;i<sizeof(Status_Data);i++)
		{
			*dataptr2=*dataptr;
			dataptr++;
			dataptr2++;
		}

		if((sizeof(Status_Data_tmp))<ringbuffer(RBFREE, 0, 0))
			UART_SendPacket(&Status_Data_tmp, sizeof(Status_Data_tmp), PD_STATUSDATA);
	}
#endif

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
