#include <cmath>
#include <cstring>

#include "QuadrotorModel.h"

using namespace std;

QuadrotorModel::QuadrotorModel()
{
  time = 0;
  memset((void*)&statespace, 0, sizeof(statespace));
  memset((void*)&model_params, 0, sizeof(model_params));

  const gsl_odeiv_step_type* step_type = gsl_odeiv_step_rkf45;//gsl_odeiv_step_rk4;
  step = gsl_odeiv_step_alloc(step_type, 16);
  control = gsl_odeiv_control_y_new(1e-6, 0.0);
  evolve = gsl_odeiv_evolve_alloc(16);

  x = y = z = 0;
  phi = theta = psi = 0;

  init_x = init_y = init_z = 0;
  init_phi = init_theta = init_psi = 0;

  xdot = ydot = zdot = 0;
  wb1 = wb2 = wb3 = 0;

  z_correction_w = r_correction_w = p_correction_w = 0;

  cmd_updated = false;

  // Set the default gyro saturation to 300 deg/sec;
  gyro_limit = 300.0*M_PI/180.0;

  return;
}

QuadrotorModel::~QuadrotorModel()
{
  gsl_odeiv_evolve_free(evolve);
  gsl_odeiv_control_free(control);
  gsl_odeiv_step_free(step);

  return;
}

void QuadrotorModel::SetTime(double time_)
{
  time = time_;

  return;
}

double QuadrotorModel::GetTime()
{
  return time;
}

void QuadrotorModel::SetMass(double mass)
{
  model_params.mass = mass;
}

void QuadrotorModel::SetInertia(double Ixx, double Iyy, double Izz)
{
  model_params.Ixx = Ixx;
  model_params.Ixy = 0;
  model_params.Ixz = 0;
  model_params.Iyy = Iyy;
  model_params.Iyz = 0;
  model_params.Izz = Izz;

  double I11 = Ixx;
  double I12 = 0;
  double I13 = 0;
  double I22 = Iyy;
  double I23 = 0;
  double I33 = Izz;

  model_params.IxxInv =
    (I23*I23 - I22*I33)/(I33*I12*I12 - 2*I12*I13*I23 + I22*I13*I13 + I11*I23*I23 - I11*I22*I33);
  model_params.IxyInv =
    -(I13*I23 - I12*I33)/(I33*I12*I12 - 2*I12*I13*I23 + I22*I13*I13 + I11*I23*I23 - I11*I22*I33);
  model_params.IxzInv =
    -(I12*I23 - I13*I22)/(I33*I12*I12 - 2*I12*I13*I23 + I22*I13*I13 + I11*I23*I23 - I11*I22*I33);
  model_params.IyyInv =
    (I13*I13 - I11*I33)/(I33*I12*I12 - 2*I12*I13*I23 + I22*I13*I13 + I11*I23*I23 - I11*I22*I33);
  model_params.IyzInv =
    -(I12*I13 - I11*I23)/(I33*I12*I12 - 2*I12*I13*I23 + I22*I13*I13 + I11*I23*I23 - I11*I22*I33);
  model_params.IzzInv =
    (I12*I12 - I11*I22)/(I33*I12*I12 - 2*I12*I13*I23 + I22*I13*I13 + I11*I23*I23 - I11*I22*I33);
}

void QuadrotorModel::SetLength(double length)
{
  model_params.length = length;
  c.SetLength(length);
}

void QuadrotorModel::SetMotorRPMLimits(double min, double max)
{
  model_params.motors_min_rpm = min;
  model_params.motors_max_rpm = max;
}

void QuadrotorModel::SetPropellerRadius(double radius)
{
  model_params.propeller_radius = radius;
  c.SetPropellerRadius(radius);
}

void QuadrotorModel::SetMotorGain(double k)
{
  model_params.kmotor = k;
}

void QuadrotorModel::SetThrustGain(double k)
{
  model_params.kthrust = k;
  c.SetThrustGain(k);
}

void QuadrotorModel::SetGyroLimit(double limit)
{
  gyro_limit = limit;
}

void QuadrotorModel::SetCommand(double thrust_,
                                double roll_, double pitch_, double yaw_,
                                double angvel_roll_, double angvel_pitch_, double angvel_yaw_,
                                double kp_roll_, double kd_roll_,
                                double kp_pitch_, double kd_pitch_,
                                double kp_yaw_, double kd_yaw_,
                                double z_correction,
                                double r_correction,
                                double p_correction)
{
  thrust_w = thrust_;
  roll_w = roll_;
  pitch_w = pitch_;
  yaw_w = yaw_;

  angvel_roll_w = angvel_roll_;
  angvel_pitch_w = angvel_pitch_;
  angvel_yaw_w = angvel_yaw_;

  kp_roll_w = kp_roll_;
  kd_roll_w = kd_roll_;

  kp_pitch_w = kp_pitch_;
  kd_pitch_w = kd_pitch_;

  kp_yaw_w = kp_yaw_;
  kd_yaw_w = kd_yaw_;

  z_correction_w = z_correction;
  r_correction_w = r_correction;
  p_correction_w = p_correction;

  cmd_updated = true;

  return;
}

void QuadrotorModel::SendCommand()
{
  if (!cmd_updated)
    return;

  cmd_updated = false;

  c.SetDesiredThrust(thrust_w);
  c.SetDesiredRotation(roll_w, pitch_w, yaw_w);
  c.SetDesiredAngularVelocity(angvel_roll_w, angvel_pitch_w, angvel_yaw_w);

  c.SetRotationGain(kp_roll_w, kp_pitch_w, kp_yaw_w);
  c.SetAngularVelocityGain(kd_roll_w, kd_pitch_w, kd_yaw_w);
  c.SetCorrections(z_correction_w, r_correction_w, p_correction_w);

  return;
}

inline double QuadrotorModel::Normalize(double angle)
{
  while (angle > M_PI)
    angle -= 2*M_PI;

  while (angle < -M_PI)
    angle += 2*M_PI;

  return angle;
}

int QuadrotorModel::ODEStep(double t, const double* x, double* xdot, void* params)
{
  model_params_t* param = reinterpret_cast<model_params_t*>(params);

  double g = 9.81;
  double m = param->mass;
  double L = param->length;
  double R = param->propeller_radius;

  double k_thrust = param->kthrust;
  double k_motor = param->kmotor;

  double I11 = param->Ixx;
  double I12 = param->Ixy;
  double I13 = param->Ixz;
  double I21 = I12;
  double I22 = param->Iyy;
  double I23 = param->Iyz;
  double I31 = I13;
  double I32 = I23;
  double I33 = param->Izz;

  double Iinv11 = param->IxxInv;
  double Iinv12 = param->IxyInv;
  double Iinv13 = param->IxzInv;
  double Iinv21 = Iinv12;
  double Iinv22 = param->IyyInv;
  double Iinv23 = param->IyzInv;
  double Iinv31 = Iinv13;
  double Iinv32 = Iinv23;
  double Iinv33 = param->IzzInv;

  // Linear velocity in the spatial frame
  double vs1 = x[3];
  double vs2 = x[4];
  double vs3 = x[5];

  xdot[0] = vs1;
  xdot[1] = vs2;
  xdot[2] = vs3;

  // Attitude in spatial frame
  double phi = Normalize(x[6]);
  double theta = Normalize(x[7]);
  double psi = Normalize(x[8]);

  double ct = cos(theta);
  double st = sin(theta);

  double cph = cos(phi);
  double sph = sin(phi);

  double cps = cos(psi);
  double sps = sin(psi);

  double bRw11 = ct*cps - st*sph*sps;
  double bRw12 = cps*st*sph + ct*sps;
  double bRw13 = -(cph*st);
  double bRw21 = -(cph*sps);
  double bRw22 = cph*cps;
  double bRw23 = sph;
  double bRw31 = cps*st + ct*sph*sps;
  double bRw32 = -(ct*cps*sph) + st*sps;
  double bRw33 = ct*cph;

  // Linear velocity in the body frame vb = R'vs
  double vb1 = bRw11*vs1 + bRw12*vs2 + bRw13*vs3;
  double vb2 = bRw21*vs1 + bRw22*vs2 + bRw23*vs3;
  double vb3 = bRw31*vs1 + bRw32*vs2 + bRw33*vs3;

  // Angular velocity in the body frame
  double wb1 = x[9];
  double wb2 = x[10];
  double wb3 = x[11];

  // Angular velocity of the motors
  double w1 = x[12];
  double w2 = x[13];
  double w3 = x[14];
  double w4 = x[15];

  // Fi = k_thrust*wi^2
  // Newtons = k_thrust * RPM^2

  // Forces due to rotors in the body frame
  double F1 = k_thrust*w1*w1;
  double F2 = k_thrust*w2*w2;
  double F3 = k_thrust*w3*w3;
  double F4 = k_thrust*w4*w4;

  // Force due to gravity in spatial frame
  // Fg = [0,0,-m*g]';
  double Fg = -m*g;

  // Force due to gravity in the body frame
  double bFg1 = bRw13*Fg;
  double bFg2 = bRw23*Fg;
  double bFg3 = bRw33*Fg;

  // Applied force is from the thrust - it is along the z axis
  // of the body frame
  double Fb = F1 + F2 + F3 + F4;

  // Sum of forces on body in body frame
  double Fext1 = bFg1;
  double Fext2 = bFg2;
  double Fext3 = bFg3 + Fb;

  // wb x mvb
  double wb_cross1 = m*(vb3*wb2 - vb2*wb3);
  double wb_cross2 = m*(-vb3*wb1 + vb1*wb3);
  double wb_cross3 = m*(vb2*wb1 - vb1*wb2);

  // Linear accelerations in the body frame (lab
  double lab1 = (Fext1 - wb_cross1)/m;
  double lab2 = (Fext2 - wb_cross2)/m;
  double lab3 = (Fext3 - wb_cross3)/m;

  // Copy for use in providing accelerometer data
  param->acc[0] = lab1;
  param->acc[1] = lab2;
  param->acc[2] = lab3;

  // Linear accelerations in the world frame
  // (R[0, 0, Fb]' + Fg)/m
  xdot[3] = bRw31*Fb/m;
  xdot[4] = bRw32*Fb/m;
  xdot[5] = (bRw33*Fb + Fg)/m;

  // ws = R*wb = (d(R[t])/dt*R')vee = H*[phidot; thetadot; psidot]
  // [phidot; thetadot; psidot] = Hinv*R*wb
  xdot[6] = ct*wb1 + st*wb3;
  xdot[7] = wb2 + (wb1*st - wb3*ct)*tan(phi);
  xdot[8] = (wb3*ct - wb1*st)/cph;

  // Define body forces in quadrotor frame
  //        *1*                 *front*
  //   4           2
  //         3

  //  taui are the torques in the Body Frame
  double taux = L*(F4 - F2);
  double tauy = L*(F3 - F1);

  // Moment on a single rotor = k_yaw*w^2;
  // k_yaw = CQ/CT*R*k_thrust, where R is the radius of the propeller
  // CQ is a function of inflow/outflow ratios,
  // propeller design, and coefficient of drag
  // CT is a function of inflow/outflow ratios, propeller design
  // In general CQ/CT ~= 0.5*effect due to drag (ignoring ground effect)
  double k_yaw = 0.45*R*k_thrust;

  // Rotors 1 and 3 rotate clockwise, 2 and 4 counter-clockwise
  double tauz = k_yaw*(w1*w1 - w2*w2 + w3*w3 - w4*w4);

  double Iomega1 = I11*wb1 + I12*wb2 + I13*wb3;
  double Iomega2 = I21*wb1 + I22*wb2 + I23*wb3;
  double Iomega3 = I31*wb1 + I32*wb2 + I33*wb3;

  double cross1 = wb2*Iomega3 - wb3*Iomega2;
  double cross2 = wb3*Iomega1 - wb1*Iomega3;
  double cross3 = wb1*Iomega2 - wb2*Iomega1;

  double tmc1 = taux - cross1;
  double tmc2 = tauy - cross2;
  double tmc3 = tauz - cross3;

  // Angular accelerations in the body frame (aab)
  // I*aab = tau - wb x Iwb
  xdot[9] = Iinv11*tmc1 + Iinv12*tmc2 + Iinv13*tmc3;
  xdot[10] = Iinv21*tmc1 + Iinv22*tmc2 + Iinv23*tmc3;
  xdot[11] = Iinv31*tmc1 + Iinv32*tmc2 + Iinv33*tmc3;

  double w1_des = param->control[0];
  double w2_des = param->control[1];
  double w3_des = param->control[2];
  double w4_des = param->control[3];

  double w_min = param->motors_min_rpm;
  double w_max = param->motors_max_rpm;

  w1_des = fmin(fmax(w1_des, w_min), w_max);
  w2_des = fmin(fmax(w2_des, w_min), w_max);
  w3_des = fmin(fmax(w3_des, w_min), w_max);
  w4_des = fmin(fmax(w4_des, w_min), w_max);

  xdot[12] = k_motor*(w1_des - w1);
  xdot[13] = k_motor*(w2_des - w2);
  xdot[14] = k_motor*(w3_des - w3);
  xdot[15] = k_motor*(w4_des - w4);

  return GSL_SUCCESS;
}

void QuadrotorModel::Update(double time_)
{
  double tstart = time;
  double tstop = time_;
  double h = 1e-6;

  statespace[0] = x;
  statespace[1] = y;
  statespace[2] = z;
  statespace[3] = xdot;
  statespace[4] = ydot;
  statespace[5] = zdot;

  statespace[6] = Normalize(phi);
  statespace[7] = Normalize(theta);
  statespace[8] = Normalize(psi);
  statespace[9] = wb1;
  statespace[10] = wb2;
  statespace[11] = wb3;

  gsl_odeiv_system sys = {QuadrotorModel::ODEStep, NULL, 16, (void*)&model_params};

  while (tstart < tstop)
    {
      int status = gsl_odeiv_evolve_apply(evolve,
                                          control,
                                          step,
                                          &sys,
                                          &tstart, tstop,
                                          &h, statespace);

      if (status != GSL_SUCCESS)
        break;
    }

  time = tstop;

  x = statespace[0];
  y = statespace[1];
  z = statespace[2];

  xdot = statespace[3];
  ydot = statespace[4];
  zdot = statespace[5];

  phi = Normalize(statespace[6]);
  theta = Normalize(statespace[7]);
  psi = Normalize(statespace[8]);

  wb1 = statespace[9];
  wb2 = statespace[10];
  wb3 = statespace[11];

  // Saturate the gyro data if appropriate
  double wb1_s = fabs(wb1) > gyro_limit ? copysign(gyro_limit, wb1) : wb1;
  double wb2_s = fabs(wb2) > gyro_limit ? copysign(gyro_limit, wb2) : wb2;
  double wb3_s = fabs(wb3) > gyro_limit ? copysign(gyro_limit, wb3) : wb3;

  c.SetCurrentRotation(phi, theta, psi);
  c.SetCurrentAngularVelocity(wb1_s, wb2_s, wb3_s);

  double w1, w2, w3, w4;
  c.GetControls(w1, w2, w3, w4);

  model_params.control[0] = w1;
  model_params.control[1] = w2;
  model_params.control[2] = w3;
  model_params.control[3] = w4;

  return;
}

void QuadrotorModel::GetXYZ(double& x_, double& y_, double& z_)
{
  x_ = x;
  y_ = y;
  z_ = z;
}

void QuadrotorModel::SetXYZ(double x_, double y_, double z_)
{
  x = x_;
  y = y_;
  z = z_;
}

void QuadrotorModel::GetRPY(double& roll_, double& pitch_, double& yaw_)
{
  roll_ = phi;
  pitch_ = theta;
  yaw_ = psi;
}

void QuadrotorModel::SetRPY(double roll_, double pitch_, double yaw_)
{
  phi = roll_;
  theta = pitch_;
  psi = yaw_;
}

void QuadrotorModel::SetInitialXYZ(double x_, double y_, double z_)
{
  init_x = x = x_;
  init_y = y = y_;
  init_z = z = z_;
}

void QuadrotorModel::SetInitialRPY(double roll_, double pitch_, double yaw_)
{
  init_phi = phi = roll_;
  init_theta = theta = pitch_;
  init_psi = psi = yaw_;
}


void QuadrotorModel::SetWorldLinearVelocity(double x_, double y_, double z_)
{
  xdot = x_;
  ydot = y_;
  zdot = z_;
}

void QuadrotorModel::GetWorldLinearVelocity(double& xdot_, double& ydot_, double& zdot_)
{
  xdot_ = xdot;
  ydot_ = ydot;
  zdot_ = zdot;
}

void QuadrotorModel::GetBodyAngularVelocity(double& wb1_, double& wb2_, double& wb3_)
{
  wb1_ = wb1;
  wb2_ = wb2;
  wb3_ = wb3;
}

void QuadrotorModel::SetBodyAngularVelocity(double wb1_, double wb2_, double wb3_)
{
  wb1 = wb1_;
  wb2 = wb2_;
  wb3 = wb3_;
}

void QuadrotorModel::GetBodyLinearAcceleration(double& lab1, double& lab2, double& lab3)
{
  lab1 = model_params.acc[0];
  lab2 = model_params.acc[1];
  lab3 = model_params.acc[2];
}

void QuadrotorModel::SetRotorVelocity(double w1, double w2, double w3, double w4)
{
  statespace[12] = w1;
  statespace[13] = w2;
  statespace[14] = w3;
  statespace[15] = w4;

  return;
}

void QuadrotorModel::GetRotorVelocity(double &w1, double &w2, double &w3, double &w4)
{
  w1 = statespace[12];
  w2 = statespace[13];
  w3 = statespace[14];
  w4 = statespace[15];

  return;
}

void QuadrotorModel::ResetSimulation(double time_,
                                     double x_, double y_, double z_,
                                     double roll_, double pitch_, double yaw_)
{
  // Reset the time and state
  time = time_;

  x = x_;
  y = y_;
  z = z_;
  phi = roll_;
  theta = pitch_;
  psi = yaw_;

  xdot = 0;
  ydot = 0;
  zdot = 0;
  wb1 = 0;
  wb2 = 0;
  wb3 = 0;

  // Reset the evolution of the ODE
  gsl_odeiv_evolve_reset(evolve);

  return;
}

void QuadrotorModel::ResetSimulation()
{
  time = 0;

  x = init_x;
  y = init_y;
  z = init_z;
  phi = init_phi;
  theta = init_theta;
  psi = init_psi;

  xdot = 0;
  ydot = 0;
  zdot = 0;
  wb1 = 0;
  wb2 = 0;
  wb3 = 0;

  // Reset the evolution of the ODE
  gsl_odeiv_evolve_reset(evolve);
}
