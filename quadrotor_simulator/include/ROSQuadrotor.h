#ifndef __ROSQUADROTOR__
#define __ROSQUADROTOR__
#include <ros/ros.h>
#include <tf/tf.h>

#include <asctec/Altitude.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <asctec/HWCmd.h>
#include <asctec/DMCmd.h>
#include <asctec/PDCmd.h>

#include "QuadrotorModel.h"

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
  quat.normalize();
}

class ROSQuadrotor
{
public:
  ROSQuadrotor(QuadrotorModel *model_, ros::NodeHandle &parent, std::string &name)
  {
    model = model_;
    if (model->RequiresIMU())
      model->SetIMUCallback(IMUCallback, (void*)this);
    if (model->RequiresAltitude())
      model->SetAltitudeCallback(AltitudeCallback, (void*)this);
    if (model->RequiresOdometry())
      model->SetOdometryCallback(OdometryCallback, (void*)this);

    ros::NodeHandle n(parent, name);
    
    imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);
    altitude_pub = n.advertise<asctec::Altitude>("altitude", 10);
    odometry_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

    hw_cmd_sub = n.subscribe("cmd_hw", 10, &ROSQuadrotor::HWCmdCallback, this);
    dm_cmd_sub = n.subscribe("cmd_dm", 10, &ROSQuadrotor::DMCmdCallback, this);
    pd_cmd_sub = n.subscribe("cmd_pd", 10, &ROSQuadrotor::PDCmdCallback, this);
  }
  ~ROSQuadrotor() {}

  static void IMUCallback(float roll, float pitch, float yaw,
                          float wb1, float wb2, float wb3,
                          float lab1, float lab2, float lab3, 
                          void *user_data)
  {
    ROS_DEBUG("IMU data received");
    ROSQuadrotor* q = reinterpret_cast<ROSQuadrotor*>(user_data);
    sensor_msgs::Imu* imu_msg = q->GetImuMessagePtr();

    btQuaternion quat;
    RPYToQuat(roll, pitch, yaw, quat);
    tf::quaternionTFToMsg(quat, imu_msg->orientation);

    imu_msg->orientation_covariance[0] = roll;
    imu_msg->orientation_covariance[1] = pitch;
    imu_msg->orientation_covariance[2] = yaw;
    imu_msg->angular_velocity.x = wb1;
    imu_msg->angular_velocity.y = wb2;
    imu_msg->angular_velocity.z = wb3;
    imu_msg->linear_acceleration.x = lab1;
    imu_msg->linear_acceleration.y = lab2;
    imu_msg->linear_acceleration.z = lab3;
    imu_msg->header.stamp = ros::Time::now();

    ros::Publisher* imu_pub = q->GetImuPublisherPtr();
    imu_pub->publish(*imu_msg);
  }

  sensor_msgs::Imu* GetImuMessagePtr()
  {
    return &imu_msg;
  }
 
  ros::Publisher* GetImuPublisherPtr()
  {
    return &imu_pub;
  }

  static void AltitudeCallback(float height, float dheight, void *user_data)
  {
    ROS_DEBUG("Altitude data received");

    ROSQuadrotor* q = reinterpret_cast<ROSQuadrotor*>(user_data);
    asctec::Altitude* altitude_msg = q->GetAltitudeMessagePtr();
    
    altitude_msg->height = height;
    altitude_msg->dheight = dheight;

    ros::Publisher* altitude_pub = q->GetAltitudePublisherPtr();
    altitude_pub->publish(*altitude_msg);
    
    return;
  }

  asctec::Altitude* GetAltitudeMessagePtr()
  {
    return &altitude_msg;
  }

  ros::Publisher* GetAltitudePublisherPtr()
  {
    return &altitude_pub;
  }

  static void OdometryCallback(float x, float y, float z,
                               float roll, float pitch, float yaw,
                               float xdot, float ydot, float zdot,
                               float wb1, float wb2, float wb3,
                               void *user_data)
  {
    ROS_DEBUG("Odometry data received");
    ROSQuadrotor* q = reinterpret_cast<ROSQuadrotor*>(user_data);
    nav_msgs::Odometry* odometry_msg = q->GetOdometryMessagePtr();

    odometry_msg->header.stamp = ros::Time::now();
    odometry_msg->pose.pose.position.x = x;
    odometry_msg->pose.pose.position.y = y;
    odometry_msg->pose.pose.position.z = z;
    
    btQuaternion quat;
    RPYToQuat(roll, pitch, yaw, quat);
    tf::quaternionTFToMsg(quat, odometry_msg->pose.pose.orientation);
    
    odometry_msg->twist.twist.linear.x = xdot;
    odometry_msg->twist.twist.linear.y = ydot;
    odometry_msg->twist.twist.linear.z = zdot;
    odometry_msg->twist.twist.angular.x = wb1;
    odometry_msg->twist.twist.angular.y = wb2;
    odometry_msg->twist.twist.angular.z = wb3;

    ros::Publisher* odometry_pub = q->GetOdometryPublisherPtr();
    odometry_pub->publish(*odometry_msg);
    
    return;
  }

  nav_msgs::Odometry* GetOdometryMessagePtr()
  {
    return &odometry_msg;
  }

  ros::Publisher* GetOdometryPublisherPtr()
  {
    return &odometry_pub;
  }

  void HWCmdCallback(const asctec::HWCmd::ConstPtr& msg)
  {   
    model->SetHWCommand(msg->thrust, msg->roll, msg->pitch, msg->yaw,
                        msg->cmd[0], msg->cmd[1], msg->cmd[2], msg->cmd[3]);
  }

  void PDCmdCallback(const asctec::PDCmd::ConstPtr& msg)
  {
    model->SetPDCommand(msg->thrust, msg->roll, msg->pitch, msg->yaw_delta,
                        msg->kp_roll, msg->kd_roll,
                        msg->kp_pitch, msg->kd_pitch,
                        msg->kd_yaw);
  }
  
  void DMCmdCallback(const asctec::DMCmd::ConstPtr& msg)
  {
    model->SetDMCommand(msg->u1, msg->u2, msg->u3, msg->u4);
  }
  
private:
  QuadrotorModel* model;

  ros::Subscriber hw_cmd_sub, dm_cmd_sub, pd_cmd_sub;
  ros::Publisher imu_pub, altitude_pub, odometry_pub;

  sensor_msgs::Imu imu_msg;
  nav_msgs::Odometry odometry_msg;
  asctec::Altitude altitude_msg;
};
#endif
