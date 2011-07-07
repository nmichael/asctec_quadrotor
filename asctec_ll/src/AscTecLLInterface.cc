// N. Michael, UPenn

/*
 * This file is part of asctec_ll, a ros node for interfacing to an
 * Ascending Technologies quadrotor using the low-level processor and
 * firmware provide stock with each quadrotor.
 *
 *  The asctec_ll ros node is free software: you can redistribute it and/or modify
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
 *  along with the asctec_ll ros node.
 *  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AscTecLLInterface.h"
#include <sstream>
#include <iostream>

#define ASCTEC_BAUD_RATE 57600
//#define DEBUG

AscTec::AscTec()
{
  ll_status_callback_set = false;
  imu_raw_data_callback_set = false;
  imu_calc_data_callback_set = false;
  rc_data_callback_set = false;
  ctrl_out_callback_set = false;
  gps_data_callback_set = false;
  gps_data_advanced_callback_set = false;

  last_status_req = 0;
  last_imuraw_req = 0;
  last_imucalc_req = 0;
  last_rc_req = 0;
  last_ctrloutput_req = 0;
  last_gps_req = 0;
  last_gpsadv_req = 0;

  port_open = false;
  cmd_ready = false;

  pthread_mutex_init(&cmd_buffer_mutex, NULL);
  pthread_mutex_init(&port_mutex, NULL);

  // Set the outgoing message request
  memcpy((void*)&data_request, ">*>p", 4);

  // Set the first part of the command buffer
  char command[5];

  command[0] = '>';
  command[1] = '*';
  command[2] = '>';
  command[3] = 'd';
  command[4] = 'i';

  memset((void*)&command_buffer, 0, sizeof(command_buffer));
  memcpy((void*)&command_buffer, (void*)&command, sizeof(command));
  AscTecPacketInit(&packet);

  // Zero pointers for casting data structures
  l = 0;
  d = 0;
  cd = 0;
  g = 0;
  ga = 0;
  rc = 0;
  co = 0;

  return;
}

AscTec::~AscTec()
{
  if (port_open)
    Disconnect();

  pthread_mutex_destroy(&cmd_buffer_mutex);
  pthread_mutex_destroy(&port_mutex);

  return;
}

int AscTec::Connect(const char* port)
{
  if (port_open)
    return 0;

  // Open the serial port
  if (sd.Connect(port, ASCTEC_BAUD_RATE))
    {
      printf("Failed to connect to %s\n", port);
      return -1;
    }

  port_open = true;

  return 0;
}

void AscTec::Disconnect()
{
  if (!port_open)
    return;

  sd.Disconnect();

  port_open = false;

  return;
}

void AscTec::SetSICommand(float thrust, float roll, float pitch, float yaw,
                          bool cmd_thrust, bool cmd_roll,
                          bool cmd_pitch, bool cmd_yaw)
{
  ctrl_input_t ctrl;
  memset((void*)&ctrl, 0, sizeof(ctrl_input_t));

  if (cmd_pitch)
    {
      ctrl.ctrl |= 0x1;
      if (pitch > 1)
        ctrl.pitch = 2047;
      else if (pitch < -1)
        ctrl.pitch = -2047;
      else
        ctrl.pitch = (short)floor(pitch*2047);
    }

  if (cmd_roll)
    {
      ctrl.ctrl |= 0x2;
      if (roll > 1)
        ctrl.roll = 2047;
      else if (roll < -1)
        ctrl.roll = -2047;
      else
        ctrl.roll = (short)floor(roll*2047);
    }

  if (cmd_yaw)
    {
      ctrl.ctrl |= 0x4;
      if (yaw > 1)
        ctrl.yaw = 2047;
      else if (yaw < -1)
        ctrl.yaw = -2047;
      else
        ctrl.yaw = (short)floor(yaw*2047);
    }

  if (cmd_thrust)
    {
      ctrl.ctrl |= 0x8;
      if (thrust > 1)
        ctrl.thrust = 4095;
      else if (thrust < 0)
        ctrl.thrust = 0;
      else
        ctrl.thrust = (short)floor(thrust*4095);
    }

  ctrl.chksum = ctrl.pitch + ctrl.roll + ctrl.yaw +
    ctrl.thrust + ctrl.ctrl + 0xAAAA;

  pthread_mutex_lock(&cmd_buffer_mutex);
  memcpy((void*)&command_buffer[5], (void*)&ctrl, sizeof(ctrl));
  // Signal that a command is ready
  cmd_ready = true;
  pthread_mutex_unlock(&cmd_buffer_mutex);
#ifdef DEBUG
  printf("AscTec SI: Thrust: %d, Roll: %d, Pitch: %d, Yaw: %d\n",
         ctrl.thrust, ctrl.roll, ctrl.pitch, ctrl.yaw);
#endif

  return;
}

void AscTec::SendCommand()
{
  pthread_mutex_lock(&cmd_buffer_mutex);
  if (cmd_ready)
    {
#ifdef DEBUG
      puts("AscTec sending command");
#endif
      pthread_mutex_lock(&port_mutex);
      sd.WriteChars(command_buffer, sizeof(command_buffer), 0);
      pthread_mutex_unlock(&port_mutex);
      cmd_ready = false;
    }
  pthread_mutex_unlock(&cmd_buffer_mutex);

  return;
}

void AscTec::SetLLStatusCallback(void (*ptr)(float /*battery_voltage*/,
                                             unsigned short /*cpu_load*/,
                                             bool /*compass_enabled*/,
                                             bool /*chksum_error*/,
                                             bool /*flying*/,
                                             bool /*motors_on*/,
                                             short /*flight_mode*/,
                                             unsigned short /*up_time*/))
{
  ll_status_ptr = ptr;
  ll_status_callback_set = true;
}

void AscTec::SetIMURawDataCallback(void (*ptr)(int /*pressure*/,
                                               short /*gyro_x*/,
                                               short /*gyro_y*/,
                                               short /*gyro_z*/,
                                               short /*mag_x*/,
                                               short /*mag_y*/,
                                               short /*mag_z*/,
                                               short /*acc_x*/,
                                               short /*acc_y*/,
                                               short /*acc_z*/,
                                               unsigned short /*temp_gyro*/,
                                               unsigned int /*temp_ADC*/))
{
  imu_raw_data_ptr = ptr;
  imu_raw_data_callback_set = true;
}

void AscTec::SetIMUCalcDataCallback(void (*ptr)(float /*angle_roll*/,
                                                float /*angle_pitch*/,
                                                float /*angle_yaw*/,
                                                float /*angvel_roll*/,
                                                float /*angvel_pitch*/,
                                                float /*angvel_yaw*/,
                                                float /*acc_x_calib*/,
                                                float /*acc_y_calib*/,
                                                float /*acc_z_calib*/,
                                                float /*acc_x*/,
                                                float /*acc_y*/,
                                                float /*acc_z*/,
                                                float /*acc_angle_roll*/,
                                                float /*acc_angle_pitch*/,
                                                float /*acc_absolute_value*/,
                                                int /*Hx*/, int /*Hy*/, int /*Hz*/,
                                                float /*mag_heading*/,
                                                int /*speed_x*/,
                                                int /*speed_y*/,
                                                int /*speed_z*/,
                                                float /*height*/, float /*dheight*/,
                                                float /*height_reference*/,
                                                float /*dheight_reference*/))
{
  imu_calc_data_ptr = ptr;
  imu_calc_data_callback_set = true;
}

void AscTec::SetRCDataCallback(void (*ptr)(const unsigned short (*/*channels_in*/)[8],
                                           const unsigned short (*/*channels_out*/)[8],
                                           bool /*lock*/))
{
  rc_data_ptr = ptr;
  rc_data_callback_set = true;
}

void AscTec::SetCTRLOutCallback(void (*ptr)(int /*thrust*/,
                                            int /*roll*/,
                                            int /*pitch*/,
                                            int /*yaw*/))
{
  ctrl_out_ptr = ptr;
  ctrl_out_callback_set = true;
}

void AscTec::SetGPSDataCallback(void (*ptr)(double /*latitude*/,
                                            double /*longitude*/,
                                            float /*height*/,
                                            float /*speed_x*/,
                                            float /*speed_y*/,
                                            float /*heading*/,
                                            float /*horizontal_accuracy*/,
                                            float /*vertical_accuracy*/,
                                            float /*speed_accuracy*/,
                                            unsigned int /*numSV*/,
                                            bool /*gps_fix*/))
{
  gps_data_ptr = ptr;
  gps_data_callback_set = true;
}

void AscTec::SetGPSDataAdvancedCallback(void (*ptr)(double /*latitude*/,
                                                    double /*longitude*/,
                                                    float /*height*/,
                                                    float /*speed_x*/,
                                                    float /*speed_y*/,
                                                    float /*heading*/,
                                                    float /*horizontal_accuracy*/,
                                                    float /*vertical_accuracy*/,
                                                    float /*speed_accuracy*/,
                                                    unsigned int /*numSV*/,
                                                    bool /*gps_fix*/,
                                                    double /*latitude_best_estimate*/,
                                                    double /*longitude_best_estimate*/,
                                                    double /*speed_x_best_estimate*/,
                                                    double /*speed_y_best_estimate*/))
{
  gps_data_advanced_ptr = ptr;
  gps_data_advanced_callback_set = true;
}

inline double AscTec::GetTimeDouble()
{
  struct timeval ts;
  gettimeofday(&ts,0);
  return ts.tv_sec + ts.tv_usec/1e6;
}

inline double AscTec::DegToRad(double deg)
{
  return deg*M_PI/180.0;
}

void AscTec::RequestData()
{
  double t = GetTimeDouble();

  unsigned short request = 0;
  bool send_request = false;

#ifdef DEBUG
  std::ostringstream request_str;
  request_str << "Requesting: ";
#endif

  if (ll_status_rate > 0 &&
      t - last_status_req > 1.0/ll_status_rate)
    {
      request |= LL_Status;
      last_status_req = t;
      send_request = true;
#ifdef DEBUG
      request_str << "LL_Status, ";
#endif
    }

  if (imu_raw_data_rate > 0 &&
      t - last_imuraw_req > 1.0/imu_raw_data_rate)
    {
      request |= IMU_RawData;
      last_imuraw_req = t;
      send_request = true;
#ifdef DEBUG
      request_str << "IMURawData, ";
#endif
    }

  if (imu_calc_data_rate > 0 &&
      t - last_imucalc_req > 1.0/imu_calc_data_rate)
    {
      request |= IMU_CalcData;
      last_imucalc_req = t;
      send_request = true;
#ifdef DEBUG
      request_str << "IMUCalcData, ";
#endif
    }

  if (rc_data_rate > 0 &&
      t - last_rc_req > 1.0/rc_data_rate)
    {
      request |= RC_Data;
      last_rc_req = t;
      send_request = true;
#ifdef DEBUG
      request_str << "RCData, ";
#endif
    }

  if (ctrl_out_rate > 0 &&
      t - last_ctrloutput_req > 1.0/ctrl_out_rate)
    {
      request |= CTRL_Out;
      last_ctrloutput_req = t;
      send_request = true;
#ifdef DEBUG
      request_str << "CTRLOut, ";
#endif
    }

  if (gps_data_rate > 0 &&
      t - last_gps_req > 1.0/gps_data_rate)
    {
      request |= GPS_Data;
      last_gps_req = t;
      send_request = true;
#ifdef DEBUG
      request_str << "GPSData, ";
#endif
    }

  if (gps_data_advanced_rate > 0 &&
      t - last_gpsadv_req > 1.0/gps_data_advanced_rate)
    {
      request |= GPS_Data_Advanced;
      last_gpsadv_req = t;
      send_request = true;
#ifdef DEBUG
      request_str << "GPSDataAdvanced ";
#endif
    }

  if (send_request)
    {
#ifdef DEBUG
      std::cout << request_str.str() << std::endl;
#endif
      memcpy((void*)&data_request[4], (void*)&request,
             sizeof(unsigned short));

      pthread_mutex_lock(&port_mutex);
      sd.WriteChars(data_request, sizeof(data_request), 0);
      pthread_mutex_unlock(&port_mutex);
    }
}

void AscTec::Update(double timeout_ms)
{
  char c;
  int nchars, ret;

  double dt = 0;
  double tstart = GetTimeDouble();

  do
    {
      nchars = sd.ReadChars(&c, 1, 1e4);

      if (nchars > 0)
        {
          ret = AscTecPacketProcessChar(c, &packet);

          if (ret > 0)
            {
              uint8_t desc = AscTecPacketGetDescriptor(&packet);

              switch(desc)
                {
                case PD_LLSTATUS:
#ifdef DEBUG
                  printf("Status packet\n");
#endif
                  if (ll_status_callback_set)
                    {
                      l = (ll_status_t*)AscTecPacketGetPayload(&packet);
                      (*ll_status_ptr)(l->battery_voltage_1/1e3,
                                       l->cpu_load,
                                       l->compass_enabled,
                                       l->chksum_error,
                                       l->flying,
                                       l->motors_on,
                                       l->flight_mode,
                                       l->up_time);
                    }
                  break;
                case PD_IMUCALCDATA:
#ifdef DEBUG
                  printf("IMUCALC packet\n");
#endif
                  if (imu_calc_data_callback_set)
                    {
                      cd = (imu_calcdata_t*)AscTecPacketGetPayload(&packet);
                      (*imu_calc_data_ptr)(DegToRad(cd->angle_roll/1e3),
                                           DegToRad(cd->angle_nick/1e3),
                                           DegToRad(cd->angle_yaw/1e3),
                                           DegToRad(cd->angvel_roll/1e3),
                                           DegToRad(cd->angvel_nick/1e3),
                                           DegToRad(cd->angvel_yaw/1e3),
                                           cd->acc_x_calib/1e4,
                                           cd->acc_y_calib/1e4,
                                           cd->acc_z_calib/1e4,
                                           cd->acc_x/1e4,
                                           cd->acc_y/1e4,
                                           cd->acc_z/1e4,
                                           DegToRad(cd->acc_angle_roll/1e3),
                                           DegToRad(cd->acc_angle_nick/1e3),
                                           cd->acc_absolute_value/1e4,
                                           cd->Hx,
                                           cd->Hy,
                                           cd->Hz,
                                           DegToRad(cd->mag_heading/1e3),
                                           cd->speed_x,
                                           cd->speed_y,
                                           cd->speed_z,
                                           cd->height/1e3,
                                           cd->dheight/1e3,
                                           cd->height_reference/1e3,
                                           cd->dheight_reference/1e3);
                    }
                  break;
                case PD_CTRLOUT:
#ifdef DEBUG
                  printf("CTRLOUT packet\n");
#endif
                  if (ctrl_out_callback_set)
                    {
                      co = (controller_output_t*)AscTecPacketGetPayload(&packet);
                      (*ctrl_out_ptr)(co->roll/100.0,
                                      co->nick/100.0,
                                      co->yaw/100.0,
                                      co->thrust/200.0);
                    }
                  break;
                case PD_IMURAWDATA:
#ifdef DEBUG
                  printf("IMURAW packet\n");
#endif
                  if (imu_raw_data_callback_set)
                    {
                      d = (imu_rawdata_t*)AscTecPacketGetPayload(&packet);
                      (*imu_raw_data_ptr)(d->pressure,
                                          d->gyro_x,
                                          d->gyro_y,
                                          d->gyro_z,
                                          d->mag_x,
                                          d->mag_y,
                                          d->mag_z,
                                          d->acc_x,
                                          d->acc_y,
                                          d->acc_z,
                                          d->temp_gyro,
                                          d->temp_ADC);
                    }
                  break;
                case PD_RCDATA:
#ifdef DEBUG
                  printf("RCDATA packet\n");
#endif
                  if (rc_data_callback_set)
                    {
                      rc = (rc_data_t*)AscTecPacketGetPayload(&packet);
                      (*rc_data_ptr)(&(rc->channels_in),
                                     &(rc->channels_out),
                                     rc->lock);
                    }
                  break;
                case PD_GPSDATA:
#ifdef DEBUG
                  printf("GPS packet\n");
#endif
                  if (gps_data_callback_set)
                    {
                      g = (gps_data_t*)AscTecPacketGetPayload(&packet);
                      (*gps_data_ptr)(g->latitude/10e6,
                                      g->longitude/10e6,
                                      g->height/1e3,
                                      g->speed_x/1e3,
                                      g->speed_y/1e3,
                                      DegToRad(g->heading/1e3),
                                      g->horizontal_accuracy/1e3,
                                      g->vertical_accuracy/1e3,
                                      g->vertical_accuracy/1e3,
                                      g->numSV, g->status);
                    }
                  break;
                case PD_GPSDATAADVANCED:
#ifdef DEBUG
                  printf("GPSA packet\n");
#endif
                  if (gps_data_advanced_callback_set)
                    {
                      ga = (gps_data_advanced_t*)AscTecPacketGetPayload(&packet);
                      (*gps_data_advanced_ptr)(ga->latitude/10e6,
                                               ga->longitude/10e6,
                                               ga->height/1e3,
                                               ga->speed_x/1e3,
                                               ga->speed_y/1e3,
                                               DegToRad(ga->heading/1e3),
                                               ga->horizontal_accuracy/1e3,
                                               ga->vertical_accuracy/1e3,
                                               ga->vertical_accuracy/1e3,
                                               ga->numSV, ga->status,
                                               ga->latitude_best_estimate/10e6,
                                               ga->longitude_best_estimate/10e6,
                                               ga->speed_x_best_estimate/1e3,
                                               ga->speed_y_best_estimate/1e3);
                    }
                  break;
                default:
#ifdef DEBUG
                  printf("Unhandled type: %X\n", desc);
#endif
                  return;
                }

              break;
            }
          else if (ret < 0)
            {
#ifdef DEBUG
              printf("ERROR: %s\n", AscTecPacketPrintError(ret));
#endif
            }
        }

      dt = (GetTimeDouble() - tstart)*1e3;
    }
  while (dt < timeout_ms);

  RequestData();

  return;
}
