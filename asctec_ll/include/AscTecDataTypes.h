#ifndef ASCTEC_DATA_TYPES_H
#define ASCTEC_DATA_TYPES_H

#define LL_Status 0x0001
#define IMU_RawData 0x0002
#define IMU_CalcData 0x0004
#define RC_Data 0x0008
#define CTRL_Out 0x0010
#define GPS_Data 0x0080
#define GPS_Data_Advanced 0x0200

#define PD_IMURAWDATA 0x01
#define PD_LLSTATUS 0x02
#define PD_IMUCALCDATA 0x03
#define PD_HLSTATUS 0x04
#define PD_DEBUGDATA 0x05
#define PD_CTRLOUT 0x11
#define PD_FLIGHTPARAMS 0x12
#define PD_CTRLCOMMANDS 0x13
#define PD_CTRLINTERNAL 0x14
#define PD_RCDATA 0x15
#define PD_CTRLSTATUS 0x16
#define PD_CTRLINPUT 0x17
#define PD_CTRLFALCON 0x18
#define PD_WAYPOINT 0x20
#define PD_CURRENTWAY 0x21
#define PD_NMEADATA 0x22
#define PD_GPSDATA 0x23
#define PD_SINGLEWAYPOINT 0x24
#define PD_GOTOCOMMAND 0x25
#define PD_LAUNCHCOMMAND 0x26
#define PD_LANDCOMMAND 0x27
#define PD_HOMECOMMAND 0x28
#define PD_GPSDATAADVANCED 0x29
#define PD_CAMERACOMMANDS 0x30
#define PD_CAMERADATA 0x31

typedef struct LL_STATUS
{
  //battery voltages in mV
  short battery_voltage_1;
  short battery_voltage_2;
  //don't care
  short status;
  //Controller cycles per second (should be ~1000)
  short cpu_load;
  //don't care
  char compass_enabled;
  char chksum_error;
  char flying;
  char motors_on;
  short flight_mode;
  //Time motors are turning
  short up_time;
} ll_status_t;

typedef struct IMU_RAWDATA
{
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
} imu_rawdata_t;

typedef struct IMU_CALCDATA
{
  //angles derived by integration of gyro_outputs, drift compensated by data fusion;
  // -90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree
  int angle_nick;
  int angle_roll;
  int angle_yaw;
  //angular velocities, raw values [16 bit] but bias free
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
  //magnetic field sensors output, offset free and scaled; units not determined, as
  //only the direction of the field vector is taken into account
  int Hx;
  int Hy;
  int Hz;
  //compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree
  int mag_heading;
  //pseudo speed measurements: integrated accelerations, pulled towards zero; units
  //unknown; used for short-term position stabilization
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
} imu_calcdata_t;

typedef struct GPS_DATA
{
  //latitude/longitude in deg * 10^7
  int latitude;
  int longitude;
  //GPS height in mm
  int height;
  //speed in x (E/W) and y(N/S) in mm/s
  int speed_x;
  int speed_y;
  //GPS heading in deg * 1000
  int heading;
  //accuracy estimates in mm and mm/s
  unsigned int horizontal_accuracy;
  unsigned int vertical_accuracy;
  unsigned int speed_accuracy;
  //number of satellite vehicles used in NAV solution
  unsigned int numSV;
  // GPS status information; 0x03 = valid GPS fix
  int status;
} gps_data_t;

typedef struct GPS_DATA_ADVANCED
{
  //latitude/longitude in deg * 10^7
  int latitude;
  int longitude;
  //GPS height in mm
  int height;
  //speed in x (E/W) and y(N/S) in mm/s
  int speed_x;
  int speed_y;
  //GPS heading in deg * 1000
  int heading;
  //accuracy estimates in mm and mm/s
  unsigned int horizontal_accuracy;
  unsigned int vertical_accuracy;
  unsigned int speed_accuracy;
  //number of satellite vehicles used in NAV solution
  unsigned int numSV;
  // GPS status information; 0x03 = valid GPS fix
  int status;
  //coordinates of current origin in deg * 10^7
  int latitude_best_estimate;
  int longitude_best_estimate;
  //velocities in X (E/W) and Y (N/S) after data fusion
  int speed_x_best_estimate;
  int speed_y_best_estimate;
} gps_data_advanced_t;

typedef struct RC_DATA
{
  //channels as read from R/C receiver
  unsigned short channels_in[8];
  //channels bias free, remapped and scaled to 0..4095
  unsigned short channels_out[8];
  //Indicator for valid R/C receiption
  unsigned char lock;
} rc_data_t;

typedef struct CONTROLLER_OUTPUT
{
  //attitude controller outputs; 0..200 = -100 ..+100%
  int nick;
  int roll;
  int yaw;
  //current thrust (height controller output); 0..200 = 0..100%
  int thrust;
} controller_output_t;

typedef struct CTRL_INPUT 
{ //serial commands (= Scientific Interface)
  short pitch; //Pitch input: -2047..+2047 (0=neutral)
  short roll; //Roll input: -2047..+2047 (0=neutral)
  short yaw; //(=R/C Stick input) -2047..+2047 (0=neutral)
  short thrust; //Collective: 0..4095 = 0..100%
  short ctrl; /*control byte:
                bit 0: pitch control enabled
                bit 1: roll control enabled
                bit 2: yaw control enabled
                bit 3: thrust control enabled
                These bits can be used to only enable one axis at a
                time and thus to control the other axes manually.
                This usually helps a lot to set up and finetune
                controllers for each axis seperately.
                */
  short chksum;
} ctrl_input_t;


#endif //ASCTEC_DATA_TYPES_H
