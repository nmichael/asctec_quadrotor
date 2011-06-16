#include <sstream>

#include <ros/ros.h>
#include <roslib/Clock.h>

#include <quadrotor_simulator/ResetSimulation.h>
#include "ROSQuadrotor.h"
#include "MultiQuadrotorSimulator.h"

MultiQuadrotorSimulator simulator;

bool reset_simulation(quadrotor_simulator::ResetSimulation::Request& req,
                      quadrotor_simulator::ResetSimulation::Response& res)
{
  simulator.ResetSimulation();

  return true;
}

std::vector<ROSQuadrotor*> quadrotors;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_quadrotor_simulator");
  ros::NodeHandle n("~");

  bool use_sim_time;
  n.param("use_sim_time", use_sim_time, true);

  if (use_sim_time)
    n.setParam("/use_sim_time", true);

  int count;
  n.param("count", count, 1);

  std::string model_file;
  n.param("model_file", model_file, std::string("model.xml"));

  if (simulator.Initialize(count, model_file) != 0)
    {
      ROS_FATAL("%s: failed to initialize quadrotor models",
                ros::this_node::getName().c_str());
      return -1;
    }

  for (int i = 0; i < count; i++)
    {
      std::stringstream ss;
      ss << i + 1;
      std::string name = std::string("quadrotor") + ss.str();
      ROSQuadrotor *quad = new ROSQuadrotor(simulator.GetModelPtr(i), n, name);

      nav_msgs::Odometry* odometry_msg = quad->GetOdometryMessagePtr();
      odometry_msg->header.frame_id = std::string("map");
  
      std::string quadrotor_frame_id;
      n.param(name + "/frame_id", 
              odometry_msg->child_frame_id, name + "/base");

      sensor_msgs::Imu* imu_msg = quad->GetImuMessagePtr();
      n.param(name + "/imu_frame_id", 
              imu_msg->header.frame_id, name + "/imu");
      quadrotors.push_back(quad);

      double init_x, init_y, init_z, init_roll, init_pitch, init_yaw;
      n.param(name + "/init/x", init_x, 0.0);
      n.param(name + "/init/y", init_y, 0.0);
      n.param(name + "/init/z", init_z, 0.0);
      n.param(name + "/init/roll", init_roll, 0.0);
      n.param(name + "/init/pitch", init_pitch, 0.0);
      n.param(name + "/init/yaw", init_yaw, 0.0);

      simulator.GetModelPtr(i)->SetInitialXYZ(init_x, init_y, init_z);
      simulator.GetModelPtr(i)->SetInitialRPY(init_roll, init_pitch, init_yaw);
    }

  ros::WallRate r(1000);

  ros::ServiceServer service = n.advertiseService("reset_simulation", 
                                                  reset_simulation);

  ros::Publisher clock_pub = n.advertise<roslib::Clock>("/clock", 1000);
   
  roslib::Clock msgc;

  while (n.ok())
    {
      simulator.Update();

      ros::spinOnce();

      for (int i = 0; i < count; i++)
        simulator.GetModelPtr(i)->SendCommand();

      if (use_sim_time)
        {
          msgc.clock = ros::Time(simulator.GetSimulationTime());
          clock_pub.publish(msgc);
        }

      r.sleep();
    }

  for (std::vector<ROSQuadrotor*>::iterator i = quadrotors.begin();
       i != quadrotors.end(); ++i)
    delete *i;

  return 0;
}
