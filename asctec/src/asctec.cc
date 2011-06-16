#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

#include <asctec/SerialStatus.h>
#include <asctec/Voltage.h>
#include <asctec/Altitude.h>
#include <asctec/HWCmd.h>
#include <asctec/DMCmd.h>
#include <asctec/PDCmd.h>

#include "AscTecInterface.h"

AscTec asctec_interface;

ros::Subscriber cmd_sub;

ros::Publisher imu_pub;
sensor_msgs::Imu imu_msg;

void cmd_callback(const ros::TimerEvent& e)
{
  asctec_interface.SendCommand();
}

void RPYToQuat(float roll, float pitch, float yaw, 
               btQuaternion &quat)
{
  // Rot RZ(yaw)*RX(roll)*RY(pitch)
  btMatrix3x3 rot(cos(pitch)*cos(yaw) - sin(pitch)*sin(roll)*sin(yaw),
                  -(cos(roll)*sin(yaw)),
                  cos(yaw)*sin(pitch) + cos(pitch)*sin(roll)*sin(yaw),
                  cos(yaw)*sin(pitch)*sin(roll) + cos(pitch)*sin(yaw),
                  cos(roll)*cos(yaw),
                  -(cos(pitch)*cos(yaw)*sin(roll)) + sin(pitch)*sin(yaw),
                  -(cos(roll)*sin(pitch)),
                  sin(roll),
                  cos(pitch)*cos(roll));
  rot.getRotation(quat);
}

void imu_callback(float roll, float pitch, float yaw,
                  float wx, float wy, float wz,
                  float ax, float ay, float az)
{
  ROS_DEBUG("imu data received");

  btQuaternion quat;
  RPYToQuat(roll, pitch, yaw, quat);
  tf::quaternionTFToMsg(quat, imu_msg.orientation);

  imu_msg.angular_velocity.x = wx;
  imu_msg.angular_velocity.y = wy;
  imu_msg.angular_velocity.z = wz;
  imu_msg.orientation_covariance[0] = roll;
  imu_msg.orientation_covariance[1] = pitch;
  imu_msg.orientation_covariance[2] = yaw;
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;
  imu_msg.header.stamp = ros::Time::now();
  imu_pub.publish(imu_msg);

  return;
}

ros::Publisher altitude_pub;
asctec::Altitude altitude_msg;

void altitude_callback(float height, float dheight)
{
  altitude_msg.height = height;
  altitude_msg.dheight = dheight;

  altitude_pub.publish(altitude_msg);

  return;
}

ros::Publisher serial_pub;
asctec::SerialStatus serial_msg;

void serial_callback(bool serial_status)
{
  ROS_DEBUG("Serial data received");

  serial_msg.serial = serial_status;
  serial_pub.publish(serial_msg);

  return;
}

ros::Publisher voltage_pub;
asctec::Voltage voltage_msg;

void voltage_callback(float voltage)
{
  ROS_DEBUG("Voltage data received");

  voltage_msg.battery_voltage = voltage;
  voltage_msg.header.stamp = ros::Time::now();
  voltage_pub.publish(voltage_msg);

  return;
}

void cmd_pd_callback(const asctec::PDCmd::ConstPtr& msg)
{
  asctec_interface.SetPDCommand(msg->thrust, msg->roll, 
                                msg->pitch, msg->yaw_delta,
                                msg->kp_roll, msg->kd_roll,
                                msg->kp_pitch, msg->kd_pitch,
                                msg->kd_yaw, 
                                msg->p_des, msg->q_des, msg->r_des,
                                msg->roll_delta, msg->pitch_delta,
                                msg->vicon_roll, msg->vicon_pitch, 
                                msg->vicon_bil);
}

void cmd_hw_callback(const asctec::HWCmd::ConstPtr& msg)
{
  float thrust = msg->thrust;
  float roll = msg->roll;
  float pitch = msg->pitch;
  float yaw = msg->yaw;

  bool cmd_thrust = msg->cmd[0];
  bool cmd_roll = msg->cmd[1];
  bool cmd_pitch = msg->cmd[2];
  bool cmd_yaw = msg->cmd[3];

  asctec_interface.SetHWCommand(thrust, roll, pitch, yaw,
                                cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw);
}

void cmd_dm_callback(const asctec::DMCmd::ConstPtr& msg)
{
  float u1 = msg->u1;
  float u2 = msg->u2;
  float u3 = msg->u3;
  float u4 = msg->u4;

  asctec_interface.SetDMCommand(u1, u2, u3, u4);                                
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "asctec");
  ros::NodeHandle n("~");

  double cmd_rate;
  n.param("cmd_rate", cmd_rate, 100.0);

  bool zigbee;
  n.param("zigbee", zigbee, true);
  
  if (!zigbee)
    {
      imu_pub = n.advertise<sensor_msgs::Imu>("imu", 100);
      n.param("frame_id", imu_msg.header.frame_id, std::string("imu"));
      asctec_interface.SetIMUCallback(imu_callback, 1);

      serial_pub = n.advertise<asctec::SerialStatus>("serial", 100);
      voltage_pub = n.advertise<asctec::Voltage>("voltage", 100);

      asctec_interface.SetSerialCallback(serial_callback, 100);
      asctec_interface.SetVoltageCallback(voltage_callback, 100);

      altitude_pub = n.advertise<asctec::Altitude>("altitude", 10);
      asctec_interface.SetAltitudeCallback(altitude_callback, 10);
    }
  else
    {
      serial_pub = n.advertise<asctec::SerialStatus>("serial", 100);
      voltage_pub = n.advertise<asctec::Voltage>("voltage", 100);

      asctec_interface.SetSerialCallback(serial_callback);
      asctec_interface.SetVoltageCallback(voltage_callback);
    }
  
  std::string port;
  n.param("port", port, std::string("/dev/ttyS0"));

  int port_rate;
  if (!zigbee)
    port_rate = 230400;
  else
    port_rate = 57600;

  if (asctec_interface.Connect(port.c_str(), port_rate) != 0)
    {
      ROS_ERROR("%s: unable to open port %s", 
                ros::this_node::getName().c_str(),
                port.c_str());
      return -1; 
    }

  // Register the cmd subscriber just as we get rolling
  ros::Subscriber cmd_hw_sub = n.subscribe("cmd_hw", 10, cmd_hw_callback);
  ros::Subscriber cmd_dm_sub;
  if (!zigbee)
    cmd_dm_sub = n.subscribe("cmd_dm", 10, cmd_dm_callback);
  ros::Subscriber cmd_pd_sub = n.subscribe("cmd_pd", 10, cmd_pd_callback);

  ros::Timer timer = n.createTimer(ros::Duration(1.0/cmd_rate), cmd_callback);
 
  while (n.ok())
    {
      asctec_interface.Update();
      
      ros::spinOnce();   
    }

  asctec_interface.Disconnect();

  return 0;
}
