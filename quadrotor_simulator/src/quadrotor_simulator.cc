#include <ros/ros.h>
#include <roslib/Clock.h>

#include <quadrotor_simulator/ResetSimulation.h>
#include "ROSQuadrotor.h"
#include "QuadrotorSimulator.h"

QuadrotorSimulator simulator;

bool reset_simulation(quadrotor_simulator::ResetSimulation::Request& req,
                      quadrotor_simulator::ResetSimulation::Response& res)
{
  simulator.ResetSimulation();

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quadrotor_simulator");
  ros::NodeHandle n("~");

  bool use_sim_time;
  n.param("use_sim_time", use_sim_time, true);

  if (use_sim_time)
    n.setParam("/use_sim_time", true);

  std::string model_file;
  n.param("model_file", model_file, std::string("model.xml"));
  
  if (simulator.LoadModel(model_file) != 0)
    {
      ROS_ERROR("%s: failed to load the model file %s",
                ros::this_node::getName().c_str(),
                model_file.c_str());
      return -1;
    }

  std::string name("quadrotor");
  ROSQuadrotor quad(simulator.GetModelPtr(), n, name);

  nav_msgs::Odometry* odometry_msg = quad.GetOdometryMessagePtr();
  odometry_msg->header.frame_id = std::string("world");
  
  std::string quadrotor_frame_id;
  n.param("frame_id", odometry_msg->child_frame_id, std::string("quadrotor"));

  sensor_msgs::Imu* imu_msg = quad.GetImuMessagePtr();
  n.param("imu_frame_id", imu_msg->header.frame_id, std::string("imu"));

  double init_x, init_y, init_z, init_roll, init_pitch, init_yaw;
  n.param("init/x", init_x, 0.0);
  n.param("init/y", init_y, 0.0);
  n.param("init/z", init_z, 0.0);
  n.param("init/roll", init_roll, 0.0);
  n.param("init/pitch", init_pitch, 0.0);
  n.param("init/yaw", init_yaw, 0.0);

  simulator.GetModelPtr()->SetInitialXYZ(init_x, init_y, init_z);
  simulator.GetModelPtr()->SetInitialRPY(init_roll, init_pitch, init_yaw);

  ros::WallRate r(1000);

  ros::ServiceServer service = n.advertiseService("reset_simulation", 
                                                  reset_simulation);

  ros::Publisher clock_pub = n.advertise<roslib::Clock>("/clock", 100);
    
  roslib::Clock msgc;
  unsigned int cntr = 0;

  while (n.ok())
    {
      simulator.Update();
      
      ros::spinOnce();

      simulator.GetModelPtr()->SendCommand();

      if (use_sim_time)
        {
          cntr++;
          if (cntr == 10)
            {
              cntr = 0;
              msgc.clock = ros::Time(simulator.GetSimulationTime());
              clock_pub.publish(msgc);
            }
        }

      r.sleep();
    }

  return 0;
}
