#ifndef ASCTEC_DATA_TYPES_H
#define ASCTEC_DATA_TYPES_H

#define PD_CTRLINPUT 0x05
#define PD_OUTPUTDATA 0x06

typedef struct OUTPUT_DATA
{
  unsigned short cpu_load;
  unsigned short voltage;

  short roll;
  short pitch;
  short yaw;

  short angvel_roll;
  short angvel_pitch;
  short angvel_yaw;

  short acc_x;
  short acc_y;
  short acc_z;
} output_data_t;

// Needs to match firmware definition
// Avoid using char (8-bit) values or you'll need to worry about memory alignment

typedef struct CTRL_INPUT
{
  unsigned short kp_roll, kd_roll;
  unsigned short kp_pitch, kd_pitch;
  unsigned short kp_yaw, kd_yaw;

  short roll;
  short pitch;
  short yaw;
  unsigned short thrust;

  short p_des;
  short q_des;
  short r_des;

  short z_correction;
  short r_correction;
  short p_correction;

  short chksum;
} ctrl_input_t;

#endif //ASCTEC_DATA_TYPES_H
