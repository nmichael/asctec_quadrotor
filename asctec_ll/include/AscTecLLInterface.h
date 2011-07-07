#ifndef __ASCTECLLINTERFACE__
#define __ASCTECLLINTERFACE__

// Class object to interface with AscTec Robots using LL communications

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

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cmath>

#include "SerialDevice.hh"
#include "AscTecPacket.h"
#include "AscTecDataTypes.h"

class AscTec
{
 public:
  AscTec();
  ~AscTec();

  // Connect to the serial port and set everything up
  int Connect(const char* serial_port);
  // Close the serial port
  void Disconnect();

  // Cycle once to process serial buffer and callbacks for data
  void Update(double timeout_ms = 1);

  // Set the command
  void SetSICommand(float thrust, float roll, float pitch, float yaw,
                    bool cmd_thrust, bool cmd_roll,
                    bool cmd_pitch, bool cmd_yaw);

  // Trigger the sending of the command
  void SendCommand();

  // Methods to set data request rates
  void SetLLStatusRate(double rate) {ll_status_rate = rate;}
  void SetIMURawDataRate(double rate) {imu_raw_data_rate = rate;}
  void SetIMUCalcDataRate(double rate) {imu_calc_data_rate = rate;}
  void SetRCDataRate(double rate) {rc_data_rate = rate;}
  void SetCTRLOutRate(double rate) {ctrl_out_rate = rate;}
  void SetGPSDataRate(double rate) {gps_data_rate = rate;}
  void SetGPSDataAdvancedRate(double rate) {gps_data_advanced_rate = rate;}

  // Callbacks for Data I/O
  void SetLLStatusCallback(void (*ptr)(float /*battery_voltage*/,
                                       unsigned short /*cpu_load*/,
                                       bool /*compass_enabled*/,
                                       bool /*chksum_error*/,
                                       bool /*flying*/,
                                       bool /*motors_on*/,
                                       short /*flight_mode*/,
                                       unsigned short /*up_time*/));

  void SetIMURawDataCallback(void (*ptr)(int /*pressure*/,
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
                                         unsigned int /*temp_ADC*/));

  void SetIMUCalcDataCallback(void (*ptr)(float /*angle_roll*/,
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
                                          float /*dheight_reference*/));

  void SetRCDataCallback(void (*ptr)(const unsigned short (*/*channels_in*/)[8],
                                     const unsigned short (*/*channels_out*/)[8],
                                     bool /*lock*/));

  void SetCTRLOutCallback(void (*ptr)(int /*thrust*/,
                                      int /*roll*/,
                                      int /*pitch*/,
                                      int /*yaw*/));

  void SetGPSDataCallback(void (*ptr)(double /*latitude*/,
                                      double /*longitude*/,
                                      float /*height*/,
                                      float /*speed_x*/,
                                      float /*speed_y*/,
                                      float /*heading*/,
                                      float /*horizontal_accuracy*/,
                                      float /*vertical_accuracy*/,
                                      float /*speed_accuracy*/,
                                      unsigned int /*numSV*/,
                                      bool /*gps_fix*/));

  void SetGPSDataAdvancedCallback(void (*ptr)(double /*latitude*/,
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
                                              double /*speed_y_best_estimate*/));

 private:
  double DegToRad(double deg);
  double GetTimeDouble();
  void RequestData();

  bool ll_status_callback_set;
  bool imu_raw_data_callback_set;
  bool imu_calc_data_callback_set;
  bool rc_data_callback_set;
  bool ctrl_out_callback_set;
  bool gps_data_callback_set;
  bool gps_data_advanced_callback_set;

  double ll_status_rate;
  double imu_raw_data_rate;
  double imu_calc_data_rate;
  double rc_data_rate;
  double ctrl_out_rate;
  double gps_data_rate;
  double gps_data_advanced_rate;

  double last_status_req;
  double last_imuraw_req;
  double last_imucalc_req;
  double last_rc_req;
  double last_ctrloutput_req;
  double last_gps_req;
  double last_gpsadv_req;

  pthread_mutex_t cmd_buffer_mutex;
  pthread_mutex_t port_mutex;

  char data_request[4 + sizeof(unsigned short)];
  char command_buffer[5 + sizeof(ctrl_input_t)];
  bool cmd_ready;
  bool pd_cmd_ready;
  bool port_open;

  ll_status_t *l;
  imu_rawdata_t *d;
  imu_calcdata_t *cd;
  gps_data_t *g;
  gps_data_advanced_t *ga;
  rc_data_t *rc;
  controller_output_t *co;

  SerialDevice sd;
  AscTecPacket packet;

  void (*ll_status_ptr)(float /*battery_voltage*/,
                        unsigned short /*cpu_load*/,
                        bool /*compass_enabled*/,
                        bool /*chksum_error*/,
                        bool /*flying*/,
                        bool /*motors_on*/,
                        short /*flight_mode*/,
                        unsigned short /*up_time*/);

  void (*imu_raw_data_ptr)(int /*pressure*/,
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
                           unsigned int /*temp_ADC*/);

  void (*imu_calc_data_ptr)(float /*angle_roll*/,
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
                            float /*dheight_reference*/);

  void (*rc_data_ptr)(const unsigned short (*/*channels_in*/)[8],
                      const unsigned short (*/*channels_out*/)[8],
                      bool /*lock*/);

  void (*ctrl_out_ptr)(int /*thrust*/,
                       int /*roll*/,
                       int /*pitch*/,
                       int /*yaw*/);

  void (*gps_data_ptr)(double /*latitude*/,
                       double /*longitude*/,
                       float /*height*/,
                       float /*speed_x*/,
                       float /*speed_y*/,
                       float /*heading*/,
                       float /*horizontal_accuracy*/,
                       float /*vertical_accuracy*/,
                       float /*speed_accuracy*/,
                       unsigned int /*numSV*/,
                       bool /*gps_fix*/);

  void (*gps_data_advanced_ptr)(double /*latitude*/,
                                double /*longitude*/,
                                float /*height*/,
                                float /*speed_x*/, float /*speed_y*/,
                                float /*heading*/,
                                float /*horizontal_accuracy*/,
                                float /*vertical_accuracy*/,
                                float /*speed_accuracy*/,
                                unsigned int /*numSV*/,
                                bool /*gps_fix*/,
                                double /*latitude_best_estimate*/,
                                double /*longitude_best_estimate*/,
                                double /*speed_x_best_estimate*/,
                                double /*speed_y_best_estimate*/);
};
#endif
