#ifndef __QUADROTORSIMULATOR__
#define __QUADROTORSIMULATOR__
#include <string>
#include "QuadrotorModel.h"

class QuadrotorSimulator
{
 public:
  QuadrotorSimulator();
  ~QuadrotorSimulator();

  double GetSimulationTime();
  int LoadModel(const std::string& model_file);
  void SendCommand();
  void Update();
  QuadrotorModel* GetModelPtr();

  void ResetSimulation();

 private:
  double time_step;
  double simulation_time;

  QuadrotorModel quad; 

  bool config_loaded;
  
  int CheckCollision();
};
#endif
