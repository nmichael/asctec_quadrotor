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

/* To output data from the vehicle, enable the OUTPUT macro in main.h and
 * enable the OUTPUT_DATA macro below.
 * It will stream the output data and publish using the callback below.
 * DO NOT output data from the vehicle unless you are using a tethered connection.
 * Also, note that the baud rate changes when this macro is defined (see firmware/system.c)
 * so you'll need to set the port_rate value to 230400 below.
 */

#define OUTPUT_DATA

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

#include <asctec/Status.h>
#include <asctec/Command.h>

#include "AscTecInterface.h"

AscTecInterface asctec_interface;

ros::Subscriber cmd_sub;

void cmd_timer(const ros::TimerEvent& e)
{
  asctec_interface.SendCommand();
}

void cmd_callback(const asctec::Command::ConstPtr& msg)
{
  asctec_interface.SetCommand(msg->thrust,
                              msg->roll, msg->pitch, msg->yaw,
                              msg->p, msg->q, msg->r,
                              msg->kp_roll, msg->kd_roll,
                              msg->kp_pitch, msg->kd_pitch,
                              msg->kp_yaw, msg->kd_yaw,
                              msg->z_correction,
                              msg->r_correction,
                              msg->p_correction);
}

#ifdef OUTPUT_DATA
ros::Publisher s_pub;
ros::Publisher i_pub;

sensor_msgs::Imu msg;
asctec::Status status_msg;
std::string frame_id;

void output_callback(const AscTecInterface::output_t &data)
{
  unsigned int cpu_load = data.cpu_load;
  double roll = data.roll;
  double pitch = data.pitch;
  double yaw = data.yaw;
  double voltage = data.voltage;
  double wx = data.wx;
  double wy = data.wy;
  double wz = data.wz;
  double ax = data.ax;
  double ay = data.ay;
  double az = data.az;

#if 0
  printf("CPU Load: %u\n", cpu_load);
  printf("rpy: %f, %f, %f\n", roll, pitch, yaw);
  printf("voltage: %f\n", voltage);
#endif

  status_msg.cpu_load = cpu_load;
  status_msg.voltage = float(voltage);
  s_pub.publish(status_msg);

  // RPY using ZXY convention
  // Defines R' (rotation from world to body)
  double cr = cos(roll);
  double sr = sin(roll);
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cy = cos(yaw);
  double sy = sin(yaw);

  double R11 = cp*cy - sp*sr*sy;
  double R12 = -cr*sy;
  double R13 = cy*sp + cp*sr*sy;
  double R21 = cy*sp*sr + cp*sy;
  double R22 = cr*cy;
  double R23 = -cp*cy*sr + sp*sy;
  double R31 = -cr*sp;
  double R32 = sr;
  double R33 = cp*cr;

  btMatrix3x3 rot(R11, R12, R13,
                  R21, R22, R23,
                  R31, R32, R33);
  btQuaternion quat;
  rot.getRotation(quat);
  quat.normalize();

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  tf::quaternionTFToMsg(quat, msg.orientation);
  msg.angular_velocity.x = wx;
  msg.angular_velocity.y = wy;
  msg.angular_velocity.z = wz;
  msg.linear_acceleration.x = ax;
  msg.linear_acceleration.y = ay;
  msg.linear_acceleration.z = az;
  i_pub.publish(msg);
}
#endif

int main(int argc, char** argv)
{
  ros::init(argc, argv, "asctec");
  ros::NodeHandle n("~");

  double cmd_rate;
  n.param("cmd_rate", cmd_rate, 100.0);

  std::string port;
  n.param("port", port, std::string("/dev/ttyS0"));

  int port_rate;
  n.param("port_rate", port_rate, 57600);

  if (asctec_interface.Connect(port.c_str(), port_rate) != 0)
    {
      ROS_ERROR("%s: unable to open port %s",
                ros::this_node::getName().c_str(),
                port.c_str());
      return -1;
    }

#ifdef OUTPUT_DATA
  i_pub = n.advertise<sensor_msgs::Imu>("imu", 100);
  s_pub = n.advertise<asctec::Status>("status", 100, true);
  asctec_interface.SetOutputCallback(output_callback);

  n.param("frame_id", frame_id, std::string("quadrotor/base"));
#endif

  ros::Subscriber cmd_sub = n.subscribe("cmd", 10, cmd_callback);

  ros::Timer timer = n.createTimer(ros::Duration(1.0/cmd_rate), cmd_timer);

#ifndef OUTPUT_DATA
  ros::spin();
#else
  while (n.ok())
    {
      ros::spinOnce();
      asctec_interface.Update();
    }
#endif

  asctec_interface.Disconnect();

  return 0;
}
