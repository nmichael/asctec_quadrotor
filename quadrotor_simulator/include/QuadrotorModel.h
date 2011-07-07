#ifndef __QUADROTORMODEL__
#define __QUADROTORMODEL__
#include <gsl/gsl_errno.h>
#include <gsl/gsl_odeiv.h>

#include "AttitudeController.h"

typedef struct
{
  double mass;
  double length;
  double propeller_radius;

  double motors_min_rpm;
  double motors_max_rpm;

  double kmotor;
  double kthrust;

  double Ixx, Ixy, Ixz, Iyy, Iyz, Izz;
  double IxxInv, IxyInv, IxzInv, IyyInv, IyzInv, IzzInv;

  double acc[3];
  double control[4];
} model_params_t;

class QuadrotorModel
{
 public:
  QuadrotorModel();
  ~QuadrotorModel();

  void Update(double time);

  void SetCommand(double thrust,
                  double roll, double pitch, double yaw,
                  double angvel_roll, double angvel_pitch, double angvel_yaw,
                  double kp_roll, double kd_roll,
                  double kp_pitch, double kd_pitch,
                  double kp_yaw, double kd_yaw,
                  double z_correction,
                  double r_correction,
                  double p_correction);

  void SendCommand();

  double GetTime();
  void SetTime(double time);

  void GetXYZ(double& x, double& y, double& z);
  void SetXYZ(double x, double y, double z);

  void GetRPY(double& roll, double& pitch, double& yaw);
  void SetRPY(double roll, double pitch, double yaw);

  void SetInitialXYZ(double x, double y, double z);
  void SetInitialRPY(double roll, double pitch, double yaw);

  void GetWorldLinearVelocity(double& x, double& y, double& z);
  void SetWorldLinearVelocity(double x, double y, double z);

  void GetBodyAngularVelocity(double& wb1, double& wb2, double& wb3);
  void SetBodyAngularVelocity(double wb1, double wb2, double wb3);

  void GetBodyLinearAcceleration(double& lab1, double& lab2, double& lab3);
  void SetBodyLinearAcceleration(double lab1, double lab2, double lab3);

  void SetRotorVelocity(double w1, double w2, double w3, double w4);
  void GetRotorVelocity(double &w1, double &w2, double &w3, double &w4);

  void ResetSimulation(double time, double x, double y, double z,
                       double roll, double pitch, double yaw);
  void ResetSimulation();

  void SetMass(double mass);
  void SetInertia(double Ixx, double Iyy, double Izz);
  void SetLength(double length);
  void SetMotorRPMLimits(double min, double max);
  void SetPropellerRadius(double radius);
  void SetMotorGain(double k);
  void SetThrustGain(double k);
  void SetGyroLimit(double limit);

  static double Normalize(double angle);

 private:
  static int ODEStep(double t, const double* y, double* dydt, void* params);

  double time;

  double x, y, z;
  double phi, theta, psi;

  double init_x, init_y, init_z;
  double init_phi, init_theta, init_psi;

  double xdot, ydot, zdot;
  double wb1, wb2, wb3;

  double thrust_w, roll_w, pitch_w, yaw_w;
  double angvel_roll_w, angvel_pitch_w, angvel_yaw_w;
  double kp_roll_w, kp_pitch_w, kp_yaw_w;
  double kd_roll_w, kd_pitch_w, kd_yaw_w;
  double z_correction_w, r_correction_w, p_correction_w;

  bool cmd_updated;
  double gyro_limit;

  double statespace[16];
  model_params_t model_params;

  // GSL ode
  gsl_odeiv_step* step;
  gsl_odeiv_control* control;
  gsl_odeiv_evolve* evolve;

  AttitudeController c;
};
#endif
