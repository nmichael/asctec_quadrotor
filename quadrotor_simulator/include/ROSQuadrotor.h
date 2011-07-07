#ifndef __ROSQUADROTOR__
#define __ROSQUADROTOR__
#include <ros/ros.h>
#include <tf/tf.h>

#include <nav_msgs/Odometry.h>
#include <asctec/Command.h>

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
    frame_id = name;

    ros::NodeHandle n(parent, name);

    double odometry_rate;
    if (parent.getParam("rates/odometry", odometry_rate))
      {
        odom_timer = parent.createTimer(ros::Rate(odometry_rate).expectedCycleTime(),
                                        &ROSQuadrotor::odometry_callback, this);
        odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
      }

    cmd_sub = n.subscribe("cmd", 10, &ROSQuadrotor::CmdCallback, this);
  }
  ~ROSQuadrotor() {}

  void odometry_callback(const ros::TimerEvent& e)
  {
    ROS_DEBUG("Sending Odometry");

    // Get the state of the quadrotor
    double x, y, z;
    double roll, pitch, yaw;
    double vx, vy, vz;
    double wx, wy, wz;

    model->GetXYZ(x, y, z);
    model->GetRPY(roll, pitch, yaw);
    model->GetWorldLinearVelocity(vx, vy, vz);
    model->GetBodyAngularVelocity(wx, wy, wz);

    btQuaternion quat;
    RPYToQuat(roll, pitch, yaw, quat);

    // *** Load up and send the Odometry message
    odom_msg.header.frame_id = "/map";
    odom_msg.child_frame_id = frame_id;
    odom_msg.header.stamp = e.current_real;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = z;
    tf::quaternionTFToMsg(quat, odom_msg.pose.pose.orientation);
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = vz;
    odom_msg.twist.twist.angular.x = wx;
    odom_msg.twist.twist.angular.y = wy;
    odom_msg.twist.twist.angular.z = wz;

    odom_pub.publish(odom_msg);
  }

  void CmdCallback(const asctec::Command::ConstPtr& msg)
  {
    model->SetCommand(msg->thrust,
                      msg->roll, msg->pitch, msg->yaw,
                      msg->p, msg->q, msg->r,
                      msg->kp_roll, msg->kd_roll,
                      msg->kp_pitch, msg->kd_pitch,
                      msg->kp_yaw, msg->kd_yaw,
                      msg->z_correction,
                      msg->r_correction,
                      msg->p_correction);
  }

  void SetFrameId(const std::string &frame_id_)
  {
    frame_id = frame_id_;
  }

private:
  QuadrotorModel* model;

  ros::Subscriber cmd_sub;
  ros::Publisher odom_pub;

  nav_msgs::Odometry odom_msg;

  ros::Timer odom_timer;

  std::string frame_id;
};
#endif
