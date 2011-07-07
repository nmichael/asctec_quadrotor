#ifndef __ATTITUDE_CONTROLLER__
#define __ATTITUDE_CONTROLLER__

class AttitudeController
{
public:
  AttitudeController() :
    roll(0), pitch(0), yaw(0),
    wb1(0), wb2(0), wb3(0),
    thrust_des(0), roll_des(0), pitch_des(0), yaw_des(0),
    wb1_des(0), wb2_des(0), wb3_des(0),
    kp_roll(0), kp_pitch(0), kp_yaw(0),
    kd_roll(0), kd_pitch(0), kd_yaw(0)
  {

  }
  ~AttitudeController() {}

  void SetCurrentRotation(double roll_, double pitch_, double yaw_)
  {
    roll = roll_;
    pitch = pitch_;
    yaw = yaw_;
  }

  void SetCurrentAngularVelocity(double wb1_, double wb2_, double wb3_)
  {
    wb1 = wb1_;
    wb2 = wb2_;
    wb3 = wb3_;
  }

  void SetDesiredThrust(double thrust)
  {
    thrust_des = thrust;
  }

  void SetDesiredRotation(double roll, double pitch, double yaw)
  {
    roll_des = roll;
    pitch_des = pitch;
    yaw_des = yaw;
  }

  void SetDesiredAngularVelocity(double wb1_, double wb2_, double wb3_)
  {
    wb1_des = wb1_;
    wb2_des = wb2_;
    wb3_des = wb3_;
  }

  void SetRotationGain(double kp_roll_, double kp_pitch_, double kp_yaw_)
  {
    kp_roll = kp_roll_;
    kp_pitch = kp_pitch_;
    kp_yaw = kp_yaw_;
  }

  void SetAngularVelocityGain(double kd_roll_, double kd_pitch_, double kd_yaw_)
  {
    kd_roll = kd_roll_;
    kd_pitch = kd_pitch_;
    kd_yaw = kd_yaw_;
  }

  void GetControls(double &w1, double &w2, double &w3, double &w4)
  {
    double uf = thrust_des;
    double uM1 = -kp_roll*(roll - roll_des) - kd_roll*(wb1 - wb1_des);
    double uM2 = -kp_pitch*(pitch - pitch_des) - kd_pitch*(wb2 - wb2_des);
    double uM3 = -kp_yaw*(yaw - yaw_des) - kd_yaw*(wb3 - wb3_des);

    double kyaw = 0.45*propeller_radius*(kthrust - z_correction);
    double KMINV = 1.0/kyaw;
    double KFLINV = 1.0/((kthrust - z_correction)*length);

    double fb1_kF = (uf*length - 2.0*uM2)*KFLINV + uM3*KMINV;
    double fb2_kF = (uf*length - 2.0*uM1)*KFLINV - uM3*KMINV;
    double fb3_kF = (uf*length + 2.0*uM2)*KFLINV + uM3*KMINV;
    double fb4_kF = (uf*length + 2.0*uM1)*KFLINV - uM3*KMINV;

    w1 = fb1_kF > 0 ? 0.5*sqrt(fb1_kF) : 0;
    w2 = fb2_kF > 0 ? 0.5*sqrt(fb2_kF) : 0;
    w3 = fb3_kF > 0 ? 0.5*sqrt(fb3_kF) : 0;
    w4 = fb4_kF > 0 ? 0.5*sqrt(fb4_kF) : 0;
  }

  void SetLength(double l)
  {
    length = l;
  }

  void SetPropellerRadius(double r)
  {
    propeller_radius = r;
  }

  void SetCorrections(double z, double r, double p)
  {
    z_correction = z;
    r_correction = r;
    p_correction = p;
  }

  void SetThrustGain(double g)
  {
    kthrust = g;
  }

private:
  double roll, pitch, yaw;
  double wb1, wb2, wb3;
  double thrust_des, roll_des, pitch_des, yaw_des;
  double wb1_des, wb2_des, wb3_des;
  double kp_roll, kp_pitch, kp_yaw;
  double kd_roll, kd_pitch, kd_yaw;

  double propeller_radius;
  double kthrust;
  double z_correction, r_correction, p_correction;
  double length;
};
#endif
