#include <cstdio>
#include <cassert>
#include <cmath>

#include "QuadrotorSimulator.h"

using namespace std;

QuadrotorSimulator::QuadrotorSimulator()
{
  simulation_time = 0;
  time_step = 1e-3;

  return;
}

QuadrotorSimulator::~QuadrotorSimulator()
{
  return;
}

double QuadrotorSimulator::GetSimulationTime()
{
  return simulation_time;
}

QuadrotorModel* QuadrotorSimulator::GetModelPtr()
{
  return &quad;
}

int QuadrotorSimulator::CheckCollision()
{
  double x, y, z;
  quad.GetXYZ(x, y, z);

  // Simple ground plane check
  if (z <= 0)
    {
      // Put the robot just above the ground
      quad.SetXYZ(x, y, 1e-2);
      return -1;
    }

  return 0;
}

void QuadrotorSimulator::ResetSimulation()
{
  simulation_time = 0;
  quad.ResetSimulation();
}

void QuadrotorSimulator::Update()
{
  simulation_time += time_step;
  quad.Update(simulation_time);

  if (CheckCollision() != 0)
    {
      double x, y, z, roll, pitch, yaw;

      quad.GetXYZ(x, y, z);
      quad.GetRPY(roll, pitch, yaw);
      quad.ResetSimulation(simulation_time,
                           x, y, z, roll, pitch, yaw);
    }

  return;
}
