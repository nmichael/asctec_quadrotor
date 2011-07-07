// N. Michael, UPenn

/*
 * This file is part of asctec, a ros node for interfacing to an
 * Ascending Technologies quadrotor using the second, high-level processor.
 *
 *  The asctec ros node is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  The asctec ros node is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the asctec ros node.
 *  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AscTecInterface.h"

AscTecInterface::AscTecInterface()
{
  output_callback_set = false;

  port_open = false;
  cmd_ready = false;

  pthread_mutex_init(&sd_mutex, NULL);

  char command[4];

  command[0] = '>';
  command[1] = '*';
  command[2] = '>';
  command[3] = 'p';

  memset((void*)&command_buffer, 0, sizeof(command_buffer));
  memcpy((void*)&command_buffer, (void*)&command, sizeof(command));

  AscTecPacketInit(&packet);

  return;
}

AscTecInterface::~AscTecInterface()
{
  if (port_open)
    Disconnect();

  pthread_mutex_destroy(&sd_mutex);

  return;
}

int AscTecInterface::Connect(const char* port, unsigned int serial_speed)
{
  if (port_open)
    return 0;

  // Open the serial port
  if (sd.Connect(port, serial_speed))
    {
      printf("Failed to connect to %s at %u\n",
             port, serial_speed);
      return -1;
    }

  port_open = true;

  return 0;
}

void AscTecInterface::Disconnect()
{
  if (!port_open)
    return;

  sd.Disconnect();

  port_open = false;

  return;
}

void AscTecInterface::SetCommand(float thrust,
                                 float roll, float pitch, float yaw,
                                 float p_des, float q_des, float r_des,
                                 float kp_roll, float kd_roll,
                                 float kp_pitch, float kd_pitch,
                                 float kp_yaw, float kd_yaw,
                                 float z_correction,
                                 float r_correction,
                                 float p_correction)
{
  ctrl_input_t ctrl;
  memset((void*)&ctrl, 0, sizeof(ctrl_input_t));

  ctrl.thrust = thrust > 0 ? thrust*1e3 : 0;
  ctrl.roll = roll < 0 ? ceil(roll*1e3 - 0.5) : floor(roll*1e3 + 0.5);
  ctrl.pitch = pitch < 0 ? ceil(pitch*1e3 - 0.5) : floor(pitch*1e3 + 0.5);
  ctrl.yaw = yaw < 0 ? ceil(yaw*1e3 - 0.5) : floor(yaw*1e3 + 0.5);

  ctrl.kp_roll = kp_roll < 0 ? 0 : floor(kp_roll*1e3 + 0.5);
  ctrl.kp_pitch = kp_pitch < 0 ? 0 : floor(kp_pitch*1e3 + 0.5);
  ctrl.kp_yaw = kp_yaw < 0 ? 0 : floor(kp_yaw*1e3 + 0.5);

  ctrl.kd_roll = kd_roll < 0 ? 0 : floor(kd_roll*1e3 + 0.5);
  ctrl.kd_pitch = kd_pitch < 0 ? 0 : floor(kd_pitch*1e3 + 0.5);
  ctrl.kd_yaw = kd_yaw < 0 ? 0 : floor(kd_yaw*1e3 + 0.5);

  ctrl.p_des = p_des < 0 ? ceil(p_des*1e3 - 0.5) : floor(p_des*1e3 + 0.5);
  ctrl.q_des = q_des < 0 ? ceil(q_des*1e3 - 0.5) : floor(q_des*1e3 + 0.5);
  ctrl.r_des = r_des < 0 ? ceil(r_des*1e3 - 0.5) : floor(r_des*1e3 + 0.5);

  ctrl.z_correction = z_correction < 0 ? ceil(z_correction*1e12 - 0.5) : floor(z_correction*1e12 + 0.5);
  ctrl.r_correction = r_correction < 0 ? ceil(r_correction*1e3 - 0.5) : floor(r_correction*1e3 + 0.5);
  ctrl.p_correction = p_correction < 0 ? ceil(p_correction*1e3 - 0.5) : floor(p_correction*1e3 + 0.5);

  ctrl.chksum = ctrl.roll + ctrl.pitch + ctrl.yaw + 0xAAAA;

  pthread_mutex_lock(&sd_mutex);
  memcpy((void*)&command_buffer[4], (void*)&ctrl, sizeof(ctrl));
  cmd_ready = true;
  pthread_mutex_unlock(&sd_mutex);

  return;
}

void AscTecInterface::SendCommand()
{
  pthread_mutex_lock(&sd_mutex);
  if (cmd_ready)
    {
      sd.WriteChars(command_buffer, sizeof(command_buffer), 0);
      cmd_ready = false;
    }
  pthread_mutex_unlock(&sd_mutex);

  return;
}

inline double AscTecInterface::DegToRad(double deg)
{
  return deg*M_PI/180.0;
}

inline double AscTecInterface::RadToDeg(double rad)
{
  return rad*180.0/M_PI;
}

#define TIMEOUT 1e3

void AscTecInterface::Update()
{
  const int buffer_size = sizeof(OUTPUT_DATA);

  char buffer[buffer_size];
  char c;
  int ret;

  int buffer_idx = 0;
  int buffer_len;

  pthread_mutex_lock(&sd_mutex);
  buffer_len = sd.ReadChars(buffer, buffer_size, TIMEOUT);
  pthread_mutex_unlock(&sd_mutex);

  while (buffer_idx < buffer_len)
    {
      c = buffer[buffer_idx];
      buffer_idx++;

      ret = AscTecPacketProcessChar(c, &packet);
      if (ret > 0)
        {
          uint8_t desc = AscTecPacketGetDescriptor(&packet);

          switch(desc)
            {
            case PD_OUTPUTDATA:
              output_data_t temp_output_data;
              memcpy(&temp_output_data, AscTecPacketGetPayload(&packet), sizeof(output_data_t));

              if (output_callback_set)
                {
                  output_t output;
                  output.cpu_load = temp_output_data.cpu_load;
                  output.voltage = temp_output_data.voltage*1e-3;
                  output.roll = DegToRad(temp_output_data.roll*1e-2);
                  output.pitch = DegToRad(temp_output_data.pitch*1e-2);
                  output.yaw = DegToRad(temp_output_data.yaw*1e-2);
                  output.wx = DegToRad(temp_output_data.angvel_roll*0.0154);
                  output.wy = DegToRad(temp_output_data.angvel_pitch*0.0154);
                  output.wz = DegToRad(temp_output_data.angvel_yaw*0.0154);
                  output.ax = temp_output_data.acc_x*9.81e-3;
                  output.ay = temp_output_data.acc_y*9.81e-3;
                  output.az = temp_output_data.acc_z*9.81e-3;

                  output_cb(output);
                }
              break;
            default:
              printf("Unhandled type: %X\n", desc);
              return;
            }
        }
    }

  return;
}
