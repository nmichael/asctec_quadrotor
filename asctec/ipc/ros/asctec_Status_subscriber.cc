#include <ros/ros.h>
#include <ipc_bridge/ipc_bridge.h>

#include <asctec/Status.h>
#include <ipc_bridge/msgs/asctec_Status.h>

#define NAMESPACE asctec
#define NAME Status

ros::Publisher pub;
NAMESPACE::NAME out_msg;

void callback(const ipc_bridge::NAMESPACE::NAME &msg)
{
  out_msg.cpu_load = msg.cpu_load;
  out_msg.voltage = msg.voltage;

  pub.publish(out_msg);
}

#include "subscriber.h"
