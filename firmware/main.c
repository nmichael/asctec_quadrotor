// Custom firmware for High-level processor on Ascending Technologies quadrotors
// Modified version of the Ascending Technologies SDK main file

// N. Michael, UPenn

// License is the same as the Asctec SDK.

// To output data, define the OUTPUT macro at the top of main.h

// pi/180*0.01
#define DEG2RADCONV 0.00017453292519943296450153635834823262484860606492
// 180.0/pi*100.0
#define RAD2DEGCONV 5729.577951308232513838447630405426025390625
// 0.0154*pi/180
#define DEGSEC2RADSECCONV 0.00026878070480712676262186056064251715724822133780

// Distance between robot center and motor
#define LENGTH 0.171

// Thrust gain, k_T, when f_i = k_T*w_i^2
#define KTHRUST 9.7206e-08

// Propeller radius
#define RADIUS 0.099

// RPMSCALE = 1/35
#define RPMSCALE 0.028571428571428571428571428571428571428571428571429

/**********************************************************
                  Header files
**********************************************************/
#include <math.h>
#include <string.h>
#include "LPC214x.h"
#include "stdio.h"
#include "main.h"
#include "system.h"
#include "uart.h"
//#include "mymath.h"
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

extern unsigned char data_requested;
extern int ZeroDepth;

volatile unsigned int trigger_cnt=0;
unsigned int logs_per_second=0, total_logs_per_second=0;

struct IMU_CALCDATA IMU_CalcData, IMU_CalcData_tmp;
struct GPS_TIME GPS_Time;
struct SYSTEM_PERMANENT_DATA SYSTEM_Permanent_Data;

// The Ctrl_Input coming from the UART
struct CTRL_INPUT Ctrl_Input, Ctrl_Input_tmp;

// The filter data, this is what we out the uart, if enabled
struct OUTPUT_DATA Output_Data, Output_Data_tmp;

extern unsigned int Ctrl_Input_updated;

#ifdef OUTPUT
#define OUTPUT_RATE ControllerCyclesPerSecond/100
#endif

// Direct motor control gains
float kp_roll = 0;
float kd_roll = 0;
float kp_pitch = 0;
float kd_pitch = 0;
float kp_yaw = 0;
float kd_yaw = 0;

// The control values
float roll_des = 0;
float pitch_des = 0;
float yaw_des = 0;
float thrust_des = 0;

float p_des = 0;
float q_des = 0;
float r_des = 0;

short angle_roll = 0;
short angle_pitch = 0;
unsigned short angle_yaw = 0;

short angvel_roll = 0;
short angvel_pitch = 0;
short angvel_yaw = 0;

short acc_x = 0;
short acc_y = 0;
short acc_z = 0;

float z_correction = 0.0;
float r_correction = 0.0;
float p_correction = 0.0;

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

      mainloop_cnt=0;
    }

  if(mainloop_trigger<10) mainloop_trigger++;

  IDISABLE;
  VICVectAddr = 0;		// Acknowledge Interrupt
}

/**********************************************************
                       MAIN
**********************************************************/
int main (void)
{
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

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif
float normalize(float angle);
float normalize(float angle)
{
  while (angle > M_PI)
    angle -= M_PI;

  while (angle < -M_PI)
    angle += M_PI;

  return angle;
}

void mainloop(void)
{
  static unsigned char led_cnt=0;//, led_state=1;
  static unsigned char t;

  led_cnt++;
  // Disable GPS flashing LEDs for no reception
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

  static short chksum = 0;

  if (Ctrl_Input_updated)
    {
      memcpy(&Ctrl_Input, &Ctrl_Input_tmp, sizeof(Ctrl_Input));
      Ctrl_Input_updated = 0;

      chksum = Ctrl_Input.roll_des + Ctrl_Input.pitch_des + Ctrl_Input.yaw_des + 0xAAAA;

      if (chksum == Ctrl_Input.chksum)
        {
          kp_roll = 1e-3*Ctrl_Input.kp_roll;
          kd_roll = 1e-3*Ctrl_Input.kd_roll;

          kp_pitch = 1e-3*Ctrl_Input.kp_pitch;
          kd_pitch = 1e-3*Ctrl_Input.kd_pitch;

          kp_yaw = 1e-3*Ctrl_Input.kp_yaw;
          kd_yaw = 1e-3*Ctrl_Input.kd_yaw;

          roll_des = 1e-3*Ctrl_Input.roll_des;
          pitch_des = 1e-3*Ctrl_Input.pitch_des;
          yaw_des = 1e-3*Ctrl_Input.yaw_des;

          p_des = 1e-3*Ctrl_Input.p_des;
          q_des = 1e-3*Ctrl_Input.q_des;
          r_des = 1e-3*Ctrl_Input.r_des;

          thrust_des = 1e-3*Ctrl_Input.thrust_des;

          z_correction = 1e-12*Ctrl_Input.z_correction;
          r_correction = 1e-3*Ctrl_Input.r_correction;
          p_correction = 1e-3*Ctrl_Input.p_correction;
#if 0
          if((sizeof(Ctrl_Input))<ringbuffer(RBFREE, 0, 0))
            UART_SendPacket(&Ctrl_Input, sizeof(Ctrl_Input), PD_CTRLINPUT);
#endif
        }
    }

  // Switch signs for x-y-z, x forward, y left, z up
  float rad_r = normalize(-DEG2RADCONV*angle_roll + r_correction);
  float rad_p = normalize(DEG2RADCONV*angle_pitch + p_correction);
  float rad_y = normalize(-DEG2RADCONV*angle_yaw + M_PI);

  float Om1 = -DEGSEC2RADSECCONV*angvel_roll;
  float Om2 = DEGSEC2RADSECCONV*angvel_pitch;
  float Om3 = -DEGSEC2RADSECCONV*angvel_yaw;

  float uf = thrust_des;
  float uM1 = -kp_roll*(rad_r - roll_des) - kd_roll*(Om1 - p_des);
  float uM2 = -kp_pitch*(rad_p - pitch_des) - kd_pitch*(Om2 - q_des);
  float uM3 = -kp_yaw*(rad_y - yaw_des) - kd_yaw*(Om3 - r_des);

  // Note that the above ignores inertial cancellation

  // Define body forces in quadrotor frame (defined by AscTec)
  //        *3*                 *front*
  //   4           1
  //         2

  float km_inv = 1.0/(0.45*RADIUS*(KTHRUST - z_correction));
  float kfl_inv = 1.0/((KTHRUST - z_correction)*LENGTH);

  float fb1_kF = (uf*LENGTH - 2.0*uM1)*kfl_inv - uM3*km_inv;
  float fb2_kF = (uf*LENGTH + 2.0*uM2)*kfl_inv + uM3*km_inv;
  float fb3_kF = (uf*LENGTH - 2.0*uM2)*kfl_inv + uM3*km_inv;
  float fb4_kF = (uf*LENGTH + 2.0*uM1)*kfl_inv - uM3*km_inv;

  float w1des = 1035.0;
  float w2des = 1035.0;
  float w3des = 1035.0;
  float w4des = 1035.0;

  if (fb1_kF > 0.0)
    w1des = 0.5*sqrt(fb1_kF);
  if (fb2_kF > 0.0)
    w2des = 0.5*sqrt(fb2_kF);
  if (fb3_kF > 0.0)
    w3des = 0.5*sqrt(fb3_kF);
  if (fb4_kF > 0.0)
    w4des = 0.5*sqrt(fb4_kF);

  // Prevent the motors from stalling
  if (w1des < 1035.0) w1des = 1035.0;
  if (w2des < 1035.0) w2des = 1035.0;
  if (w3des < 1035.0) w3des = 1035.0;
  if (w4des < 1035.0) w4des = 1035.0;

  // Convert from body frame to LL frame
  float u1 = RPMSCALE*(w1des - 1000.0) + 0.5;
  float u2 = RPMSCALE*(w2des - 1000.0) + 0.5;
  float u3 = RPMSCALE*(w3des - 1000.0) + 0.5;
  float u4 = RPMSCALE*(w4des - 1000.0) + 0.5;

  if (u1 < 0)
    u1 = 0;
  else if (u1 > 199)
    u1 = 199;

  if (u2 < 0)
    u2 = 0;
  else if (u2 > 199)
    u2 = 199;

  if (u3 < 0)
    u3 = 0;
  else if (u3 > 199)
    u3 = 199;

  if (u4 < 0)
    u4 = 0;
  else if (u4 > 199)
    u4 = 199;

  WO_SDK.ctrl_enabled = 1;
  WO_SDK.ctrl_mode = 0x01;
  WO_Direct_Motor_Control.thrust = u1;
  WO_Direct_Motor_Control.roll = u2;
  WO_Direct_Motor_Control.pitch = u3;
  WO_Direct_Motor_Control.yaw = u4;

  HL2LL_write_cycle();	//write data to transmit buffer for immediate transfer to LL processor

#ifdef OUTPUT
  Output_Data.voltage = HL_Status.battery_voltage_1;
  Output_Data.cpu_load = HL_Status.cpu_load;

  Output_Data.angle_roll = RAD2DEGCONV*rad_r;
  Output_Data.angle_pitch = RAD2DEGCONV*rad_p;
  Output_Data.angle_yaw = RAD2DEGCONV*rad_y;

  Output_Data.angvel_roll = -angvel_roll;
  Output_Data.angvel_pitch = angvel_pitch;
  Output_Data.angvel_yaw = -angvel_yaw;

  Output_Data.acc_x = acc_x;
  Output_Data.acc_y = acc_y;
  Output_Data.acc_z = acc_z;

  // Enable to send back data, don't do this unless you're using a wired connection
  static unsigned int output_counter = 0;
  static unsigned int output_rate = OUTPUT_RATE;
  ++output_counter;
  if (output_counter == output_rate)
    {
      output_counter = 0;
      memcpy(&Output_Data_tmp, &Output_Data, sizeof(Output_Data));

      if (sizeof(Output_Data_tmp) <ringbuffer(RBFREE, 0, 0))
        UART_SendPacket(&Output_Data_tmp,
                        sizeof(Output_Data_tmp), PD_OUTPUTDATA);
    }
#endif

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
