#include <ros/ros.h>
#include <ipc_bridge/ipc_bridge.h>

#include <asctec/Status.h>
#include <ipc_bridge/msgs/asctec_Status.h>

#define NAMESPACE asctec
#define NAME Status

ipc_bridge::Publisher<ipc_bridge::NAMESPACE::NAME> *p;
ipc_bridge::NAMESPACE::NAME out_msg;

void callback(const NAMESPACE::NAME::ConstPtr &msg)
{
  out_msg.cpu_load = msg->cpu_load;
  out_msg.voltage = msg->voltage;

  p->Publish(out_msg);
}

#include "publisher.h"
