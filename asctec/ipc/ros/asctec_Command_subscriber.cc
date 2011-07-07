#include <ros/ros.h>
#include <ipc_bridge/ipc_bridge.h>

#include <asctec/Command.h>
#include <ipc_bridge/msgs/asctec_Command.h>

#define NAMESPACE asctec
#define NAME Command

ros::Publisher pub;
NAMESPACE::NAME out_msg;

void callback(const ipc_bridge::NAMESPACE::NAME &msg)
{
  out_msg.header.seq = msg.header.seq;
  out_msg.header.stamp = ros::Time(msg.header.stamp);
  if (msg.header.frame_id != 0)
    out_msg.header.frame_id = std::string(msg.header.frame_id);
  else
    out_msg.header.frame_id = std::string("");

  out_msg.thrust = msg.thrust;
  out_msg.roll = msg.roll;
  out_msg.pitch = msg.pitch;
  out_msg.yaw = msg.yaw;
  out_msg.kp_roll = msg.kp_roll;
  out_msg.kd_roll = msg.kd_roll;
  out_msg.kp_pitch = msg.kp_pitch;
  out_msg.kd_pitch = msg.kd_pitch;
  out_msg.kp_yaw = msg.kp_yaw;
  out_msg.kd_yaw = msg.kd_yaw;
  out_msg.p = msg.p;
  out_msg.q = msg.q;
  out_msg.r = msg.r;
  out_msg.z_correction = msg.z_correction;
  out_msg.r_correction = msg.r_correction;
  out_msg.p_correction = msg.p_correction;

  pub.publish(out_msg);
}

#include "subscriber.h"
