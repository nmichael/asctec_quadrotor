#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Empty.h>

#include "QuadrotorSimulator.h"
#include "ROSQuadrotor.h"

QuadrotorSimulator simulator;

void reset(const std_msgs::Empty::ConstPtr &msg)
{
  ROS_INFO("%s: resetting simulation", ros::this_node::getName().c_str());
  simulator.ResetSimulation();
}

void load_model_params(ros::NodeHandle &n)
{
  QuadrotorModel *model = simulator.GetModelPtr();

  double m;
  n.getParam("mass", m);
  model->SetMass(m);

  double Ixx, Iyy, Izz;
  n.getParam("inertia/Ixx", Ixx);
  n.getParam("inertia/Iyy", Iyy);
  n.getParam("inertia/Izz", Izz);
  model->SetInertia(Ixx, Iyy, Izz);

  double length;
  n.getParam("length", length);
  model->SetLength(length);

  double min, max;
  n.getParam("motor/limits/min", min);
  n.getParam("motor/limits/max", max);
  model->SetMotorRPMLimits(min, max);

  double radius;
  n.getParam("propeller_radius", radius);
  model->SetPropellerRadius(radius);

  double kmotor;
  n.getParam("gains/motor", kmotor);
  model->SetMotorGain(kmotor);

  double kthrust;
  n.getParam("gains/thrust", kthrust);
  model->SetThrustGain(kthrust);

  double limit;
  n.getParam("gyro_limit", limit);
  model->SetGyroLimit(limit);

  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quadrotor_simulator");
  ros::NodeHandle n("~");

  bool use_sim_time;
  n.param("use_sim_time", use_sim_time, true);

  if (use_sim_time)
    n.setParam("/use_sim_time", true);

  // Need to load model params before instantiating ROSQuadrotorSO3 object
  load_model_params(n);

  std::string name("quadrotor");
  ROSQuadrotor quad(simulator.GetModelPtr(), n, name);

  std::string frame_id;
  n.param("frame_id", frame_id, std::string("quadrotor"));
  quad.SetFrameId(frame_id);

  double init_x, init_y, init_z, init_roll, init_pitch, init_yaw;
  n.param("init/x", init_x, 0.0);
  n.param("init/y", init_y, 0.0);
  n.param("init/z", init_z, 0.0);
  n.param("init/roll", init_roll, 0.0);
  n.param("init/pitch", init_pitch, 0.0);
  n.param("init/yaw", init_yaw, 0.0);

  simulator.GetModelPtr()->SetInitialXYZ(init_x, init_y, init_z);
  simulator.GetModelPtr()->SetInitialRPY(init_roll, init_pitch, init_yaw);

  int speedup;
  n.param("speedup",speedup,1);
  if (speedup < 1)
    {
      ROS_ERROR("Simulation speedup of %d is not possible", speedup);
      return -1;
    }
  ros::WallRate r(speedup*1000);

  ros::Subscriber sub = n.subscribe("reset", 10, reset);
  ros::Publisher clock_pub = n.advertise<rosgraph_msgs::Clock>("/clock", 100);

  rosgraph_msgs::Clock msgc;

  unsigned int cntr = 0;

  while (n.ok())
    {
      simulator.Update();

      if (use_sim_time)
        {
          if (++cntr == 10)
            {
              msgc.clock = ros::Time(simulator.GetSimulationTime());
              clock_pub.publish(msgc);
              cntr = 0;
            }
        }

      ros::spinOnce();

      simulator.GetModelPtr()->SendCommand();

      r.sleep();
    }

  return 0;
}
