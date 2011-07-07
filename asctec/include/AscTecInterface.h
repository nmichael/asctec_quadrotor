#ifndef __ASCTECINTERFACE__
#define __ASCTECINTERFACE__

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

// Class object to interface with AscTec Robots
// Stripped down to only handle sending control to vehicles

// Note that it is easy to modify this to get data back,
// however, this is a bad idea unless you are directly interfacing
// to the vehicle via a wired connection

// The vehicle communicates via zigbee and will block on CTS,
// If the zigbee channel is noisy (other zigbees, transmitter on same freq, etc.),
// the CTS block is long enough that the online control loop will miss on the
// vehicle - the robot will fall trying to send you the data
// The block may be short, but when many robots are flying, it can lead to
// non-trivial failures.


#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include "SerialDevice.hh"
#include "AscTecPacket.h"
#include "AscTecDataTypes.h"

class AscTecInterface
{
 public:
  AscTecInterface();
  ~AscTecInterface();

  // Connect to the serial port and set everything up
  int Connect(const char* serial_port, unsigned int serial_speed);
  // Close the serial port
  void Disconnect();

  // Cycle once to process serial buffer and callbacks for data
  void Update();

  // Set the command
  void SetCommand(float thrust,
                  float roll, float pitch, float yaw,
                  float p_des, float q_des, float r_des,
                  float kp_roll, float kd_roll,
                  float kp_pitch, float kd_pitch,
                  float kp_yaw, float kd_yaw,
                  float z_correction,
                  float r_correction,
                  float p_correction);

  // Trigger the sending of the command
  void SendCommand();

  typedef struct Output
  {
    unsigned int cpu_load;
    double voltage;
    double roll, pitch, yaw;
    double wx, wy, wz;
    double ax, ay, az;
  } output_t;

  void SetOutputCallback(void(*fp)(const output_t&))
  {
    output_cb = boost::bind(fp, _1);
    output_callback_set = true;
  }

  template<class T>
  void SetOutputCallback(void(T::*fp)(const output_t&), T* obj)
  {
    output_cb = boost::bind(fp, obj, _1);
    output_callback_set = true;
  }

 private:
  double DegToRad(double deg);
  double RadToDeg(double rad);

  pthread_mutex_t sd_mutex;

  char command_buffer[4 + sizeof(ctrl_input_t)];
  bool cmd_ready;
  bool port_open;

  SerialDevice sd;
  AscTecPacket packet;

  boost::function<void(const output_t&)> output_cb;
  bool output_callback_set;
};
#endif
