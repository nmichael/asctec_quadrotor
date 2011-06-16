#ifndef MAIN_H_
#define MAIN_H_

//extern static void timer0ISR(void);
extern void mainloop(void);
extern void timer0ISR(void);

#define PELICAN

#define BATTERY_WARNING_VOLTAGE 10000	//9800 mV
#define BAT_DIV 10//(BATTERY_WARNING_VOLTAGE-9000)/100

//#define GPS_BEEP	//Warning, if GPS has no lock
//#define CHANNEL7_TO_5VSERVO_OUT	//CAM_Commands.desired_angle_roll is used for 5V servo output
#define TRIGGER_SERVO_ACTIVE	1800
#define TRIGGER_SERVO_INACTIVE	3600
#define CAMER_OFFSET_HUMMINGBIRD	63000

#define ControllerCyclesPerSecond 	1000

//#define INDOOR_GPS

//defines for VP_CAMREMOTE (in µs)
#define VP_SHOOT				2000
#define VP_LENSCLOSE_SLEEP		1900
#define VP_REFOCUS_LOCKFOCUS	1800
#define VP_ZOOM_IN				1700
#define VP_ZOOM_OUT				1600
#define VP_NEXT_PROFILE			1500
#define VP_PREVIOUS_PROFILE		1400
#define VP_SHUTTER_SPEED_INC	1300
#define VP_SHUTTER_SPEED_DEC	1200
#define VP_APERTURE_INC			1100
#define VP_APERTURE_DEC			1000


#define OFF 0
#define ON  1

#define NORMAL 0

//packet descriptors
#define PD_IMURAWDATA       0x01
#define PD_LLSTATUS        	0x02
#define PD_IMUCALCDATA      0x03
#define PD_HLSTATUS        	0x04

#define PD_FILTERDATA		0x05
#define PD_STATUSDATA		0x06
#define PD_PDINPUT			0x08

#define PD_CTRLOUT			0x11
#define PD_FLIGHTPARAMS     0x12
#define PD_CTRLCOMMANDS		0x13
#define PD_CTRLINTERNAL		0x14
#define PD_RCDATA       	0x15
#define PD_CTRLSTATUS		0x16

#define PD_WAYPOINT     	0x20
#define PD_CURRENTWAY   	0x21
#define PD_NMEADATA     	0x22
#define PD_GPSDATA			0x23

#define PD_CAMERACOMMANDS	0x30

#define CAM_TRIGGERED		0x04

#define VICON_ROLL			1

struct PD_INPUT
{
	unsigned short kp_roll, kd_roll;
	unsigned short kp_pitch, kd_pitch;
	unsigned short kd_yaw;

	short roll;
	short pitch;
	short yaw_delta;

	unsigned short thrust;

	short p_des;
	short q_des;
	short r_des;

	short roll_delta;
	short pitch_delta;

	short vicon_roll;
	short vicon_pitch;
	short vicon_bil;

	short chksum;
};
extern struct PD_INPUT PD_Input, PD_Input_tmp;

struct STATUS_DATA
{
	  // Battery voltage
	  short battery_voltage;

	  // Serial data enable
	  unsigned short serial;

	  // CPU load
	  unsigned short cpu_load;
};
extern struct STATUS_DATA Status_Data;

struct FILTER_DATA
{
  // angles derived by integration of gyro_outputs,
  // drift compensated by data fusion;
  // -9000..+9000 pitch(nick) and roll, 0..36000 yaw; 100 = 1 degree
  short angle_roll;
  short angle_pitch;
  unsigned short angle_yaw;

  //angular velocities, raw values [16 bit], bias free,
  // in 0.0154 °/s (=> 64.8 = 1 °/s)
  short angvel_roll;
  short angvel_pitch;
  short angvel_yaw;

  //acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g
  short acc_x;
  short acc_y;
  short acc_z;

  //height in mm (after data fusion)
  int height;

  //diff. height in mm/s (after data fusion)
  int dheight;

  // Battery voltage
  short battery_voltage;

  // Serial data enable
  //unsigned short serial;

  // CPU load
  unsigned short cpu_load;

  unsigned short channel[8];
};
extern struct FILTER_DATA Filter_Data;
extern long mypitch;
extern long myroll;

struct IMU_CALCDATA {
	//angles derived by integration of gyro_outputs, drift compensated by data fusion; -90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree
    int angle_nick;
    int angle_roll;
    int angle_yaw;

    //angular velocities, raw values [16 bit], bias free, in 0.0154 °/s (=> 64.8 = 1 °/s)
    int angvel_nick;
    int angvel_roll;
    int angvel_yaw;

    //acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g
    short acc_x_calib;
    short acc_y_calib;
    short acc_z_calib;

    //horizontal / vertical accelerations: -10000..+10000 = -1g..+1g
    short acc_x;
    short acc_y;
    short acc_z;

    //reference angles derived by accelerations only: -90000..+90000; 1000 = 1 degree
    int acc_angle_nick;
    int acc_angle_roll;

    //total acceleration measured (10000 = 1g)
    int acc_absolute_value;

    //magnetic field sensors output, offset free and scaled; units not determined, as only the direction of the field vector is taken into account
    int Hx;
    int Hy;
    int Hz;

    //compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree
    int mag_heading;

    //pseudo speed measurements: integrated accelerations, pulled towards zero; units unknown; used for short-term position stabilization
    int speed_x;
    int speed_y;
    int speed_z;

    //height in mm (after data fusion)
    int height;

    //diff. height in mm/s (after data fusion)
    int dheight;

    //diff. height measured by the pressure sensor [mm/s]
    int dheight_reference;

    //height measured by the pressure sensor [mm]
    int height_reference;
};
extern struct IMU_CALCDATA IMU_CalcData, IMU_CalcData_tmp;

struct IMU_RAWDATA {
//pressure sensor 24-bit value, not scaled but bias free
	int pressure;

//16-bit gyro readings; 32768 = 2.5V
    short gyro_x;
	short gyro_y;
	short gyro_z;

//10-bit magnetic field sensor readings
    short mag_x;
	short mag_y;
	short mag_z;

//16-bit accelerometer readings
    short acc_x;
	short acc_y;
    short acc_z;

//16-bit temperature measurement using yaw-gyro internal sensor
	unsigned short temp_gyro;

//16-bit temperature measurement using ADC internal sensor
	unsigned int temp_ADC;
};

extern struct IMU_RAWDATA IMU_RawData;


struct SYSTEM_PERMANENT_DATA
{
	unsigned int total_uptime;
	unsigned int total_flighttime;
	unsigned int onoff_cycles;
	unsigned int number_of_flights;
	unsigned int chksum;
};
extern struct SYSTEM_PERMANENT_DATA SYSTEM_Permanent_Data;

struct CTRL_INPUT
{
	short pitch;
	short roll;
	short yaw;
	short thrust;
	short ctrl;
	short chksum;
};
extern struct CTRL_INPUT CTRL_Input, CTRL_Input_tmp;

struct CONTROLLER_OUTPUT
{
	short nick;
	short roll;
	short yaw;
	short thrust;
};
extern struct CONTROLLER_OUTPUT CTRL_Output;

#endif /*MAIN_H_*/

