#include <ros/ros.h>
#include <tf/tf.h>

#include <asctec_ll/LLStatus.h>
#include <asctec_ll/IMURawData.h>
#include <asctec_ll/IMUCalcData.h>
#include <asctec_ll/RCData.h>
#include <asctec_ll/CTRLOut.h>
#include <asctec_ll/GPSData.h>
#include <asctec_ll/GPSDataAdvanced.h>
#include <asctec_ll/SICmd.h>

#include "AscTecLLInterface.h"

AscTec asctec_ll_interface;

void cmd_callback(const ros::TimerEvent& e)
{
  asctec_ll_interface.SendCommand();
}

ros::Publisher ll_status_pub;
asctec_ll::LLStatus status_msg;

void ll_status_callback(float battery_voltage,
                        unsigned short cpu_load,
                        bool compass_enabled,
                        bool chksum_error,
                        bool flying,
                        bool motors_on,
                        short flight_mode,
                        unsigned short up_time)
{
  status_msg.battery_voltage = battery_voltage;
  status_msg.cpu_load = cpu_load;
  status_msg.compass_enabled = compass_enabled;
  status_msg.chksum_error = chksum_error;
  status_msg.flying = flying;
  status_msg.motors_on = motors_on;
  status_msg.flight_mode = flight_mode;
  status_msg.up_time = up_time;

  ll_status_pub.publish(status_msg);
  
  return;
}

ros::Publisher imu_raw_data_pub;
asctec_ll::IMURawData imu_raw_data_msg;

void imu_raw_data_callback(int pressure,
                           short gyro_x, 
                           short gyro_y, 
                           short gyro_z,
                           short mag_x, 
                           short mag_y, 
                           short mag_z,
                           short acc_x, 
                           short acc_y, 
                           short acc_z,
                           unsigned short temp_gyro,
                           unsigned int temp_ADC)
{
  imu_raw_data_msg.pressure = pressure;
  imu_raw_data_msg.gyro.x = gyro_x;
  imu_raw_data_msg.gyro.y = gyro_y;
  imu_raw_data_msg.gyro.z = gyro_z;
  imu_raw_data_msg.mag.x = mag_x;
  imu_raw_data_msg.mag.y = mag_y;
  imu_raw_data_msg.mag.z = mag_z;
  imu_raw_data_msg.acc.x = acc_x;
  imu_raw_data_msg.acc.y = acc_y;
  imu_raw_data_msg.acc.z = acc_z; 
  imu_raw_data_msg.temp_gyro = temp_gyro;
  imu_raw_data_msg.temp_ADC = temp_ADC;

  imu_raw_data_pub.publish(imu_raw_data_msg);
  return;
}

void RPYToQuat(float roll, float pitch, float yaw, 
               btQuaternion &quat)
{
  // Rot RZ(yaw)*RX(roll)*RY(pitch)
  btMatrix3x3 rot(cos(pitch)*cos(yaw) - sin(pitch)*sin(roll)*sin(yaw),
                  -(cos(roll)*sin(yaw)),
                  cos(yaw)*sin(pitch) + cos(pitch)*sin(roll)*sin(yaw),
                  cos(yaw)*sin(pitch)*sin(roll) + cos(pitch)*sin(yaw),
                  cos(roll)*cos(yaw),
                  -(cos(pitch)*cos(yaw)*sin(roll)) + sin(pitch)*sin(yaw),
                  -(cos(roll)*sin(pitch)),
                  sin(roll),
                  cos(pitch)*cos(roll));
  rot.getRotation(quat);
}

ros::Publisher imu_calc_data_pub;
asctec_ll::IMUCalcData imu_calc_data_msg;

void imu_calc_data_callback(float angle_roll, 
                            float angle_pitch, 
                            float angle_yaw,
                            float angvel_roll,
                            float angvel_pitch,
                            float angvel_yaw,
                            float acc_x_calib,
                            float acc_y_calib,
                            float acc_z_calib,
                            float acc_x,
                            float acc_y,
                            float acc_z,
                            float acc_angle_roll,
                            float acc_angle_pitch,
                            float acc_absolute_value,
                            int Hx, int Hy, int Hz,
                            float mag_heading,
                            int speed_x, int speed_y, int speed_z,
                            float height, float dheight,
                            float height_reference, 
                            float dheight_reference)
{
  btQuaternion quat;
  RPYToQuat(angle_roll, angle_pitch, angle_yaw, quat);
  tf::quaternionTFToMsg(quat, imu_calc_data_msg.orientation);

  imu_calc_data_msg.twist.linear.x = speed_x;
  imu_calc_data_msg.twist.linear.y = speed_y;
  imu_calc_data_msg.twist.linear.z = speed_z;
  imu_calc_data_msg.twist.angular.x = angvel_roll;
  imu_calc_data_msg.twist.angular.y = angvel_pitch;
  imu_calc_data_msg.twist.angular.z = angvel_yaw;

  imu_calc_data_msg.acc_calib.x = acc_x_calib;
  imu_calc_data_msg.acc_calib.y = acc_y_calib;
  imu_calc_data_msg.acc_calib.z = acc_z_calib;

  imu_calc_data_msg.acc.x = acc_x;
  imu_calc_data_msg.acc.y = acc_y;
  imu_calc_data_msg.acc.z = acc_z;

  imu_calc_data_msg.acc_angle.x = acc_angle_roll;
  imu_calc_data_msg.acc_angle.y = acc_angle_pitch;

  imu_calc_data_msg.mag.x = Hx;
  imu_calc_data_msg.mag.y = Hy;
  imu_calc_data_msg.mag.z = Hz;

  imu_calc_data_msg.mag_heading = mag_heading;

  imu_calc_data_msg.height = height;
  imu_calc_data_msg.dheight = dheight;
  imu_calc_data_msg.height_reference = height_reference;
  imu_calc_data_msg.dheight_reference = dheight_reference;

  imu_calc_data_pub.publish(imu_calc_data_msg);

  return;
}

ros::Publisher rc_data_pub;
asctec_ll::RCData rc_data_msg;

void rc_data_callback(const unsigned short (*channels_in)[8],
                      const unsigned short (*channels_out)[8],
                      bool lock)
{
  for (int i = 0; i < 8; i++)
    {
      rc_data_msg.channels_in[i] = *channels_in[i];
      rc_data_msg.channels_out[i] = *channels_out[i];
    }
  rc_data_msg.lock = lock;

  rc_data_pub.publish(rc_data_msg);

  return;
}

ros::Publisher ctrl_out_pub;
asctec_ll::CTRLOut ctrl_out_msg;

void ctrl_out_callback(int thrust, int roll, int pitch, int yaw)
{
  ctrl_out_msg.thrust = thrust;
  ctrl_out_msg.roll = roll;
  ctrl_out_msg.pitch = pitch;
  ctrl_out_msg.yaw = yaw;

  ctrl_out_pub.publish(ctrl_out_msg);

  return;
}

ros::Publisher gps_data_pub;
asctec_ll::GPSData gps_data_msg;

void gps_data_callback(double latitude,
                       double longitude,
                       float height,
                       float speed_x, float speed_y,
                       float heading,
                       float horizontal_accuracy,
                       float vertical_accuracy,
                       float speed_accuracy,
                       unsigned int numSV,
                       bool gps_fix)
{
  gps_data_msg.latitude = latitude;
  gps_data_msg.longitude = longitude;
  gps_data_msg.height = height;
  gps_data_msg.speed_x = speed_x;
  gps_data_msg.speed_y = speed_y;
  gps_data_msg.heading = heading;
  gps_data_msg.horizontal_accuracy = horizontal_accuracy;
  gps_data_msg.vertical_accuracy = vertical_accuracy;
  gps_data_msg.speed_accuracy = speed_accuracy;
  gps_data_msg.numSV = numSV;
  gps_data_msg.gps_fix = gps_fix;

  gps_data_pub.publish(gps_data_msg);

  return;
}

ros::Publisher gps_data_advanced_pub;
asctec_ll::GPSDataAdvanced gps_data_advanced_msg;

void gps_data_advanced_callback(double latitude,
                                double longitude,
                                float height,
                                float speed_x, float speed_y,
                                float heading,
                                float horizontal_accuracy,
                                float vertical_accuracy,
                                float speed_accuracy,
                                unsigned int numSV,
                                bool gps_fix,
                                double latitude_best_estimate,
                                double longitude_best_estimate,
                                double speed_x_best_estimate,
                                double speed_y_best_estimate)
{
  gps_data_advanced_msg.gps_data.latitude = latitude;
  gps_data_advanced_msg.gps_data.longitude = longitude;
  gps_data_advanced_msg.gps_data.height = height;
  gps_data_advanced_msg.gps_data.speed_x = speed_x;
  gps_data_advanced_msg.gps_data.speed_y = speed_y;
  gps_data_advanced_msg.gps_data.heading = heading;
  gps_data_advanced_msg.gps_data.horizontal_accuracy = horizontal_accuracy;
  gps_data_advanced_msg.gps_data.vertical_accuracy = vertical_accuracy;
  gps_data_advanced_msg.gps_data.speed_accuracy = speed_accuracy;
  gps_data_advanced_msg.gps_data.numSV = numSV;
  gps_data_advanced_msg.gps_data.gps_fix = gps_fix;
  gps_data_advanced_msg.latitude_best_estimate = latitude_best_estimate;
  gps_data_advanced_msg.longitude_best_estimate = longitude_best_estimate;
  gps_data_advanced_msg.speed_x_best_estimate = speed_x_best_estimate;
  gps_data_advanced_msg.speed_y_best_estimate = speed_y_best_estimate;

  gps_data_advanced_pub.publish(gps_data_advanced_msg);

  return;
}

void cmd_si_callback(const asctec_ll::SICmd::ConstPtr& msg)
{
  float thrust = msg->thrust;
  float roll = msg->roll;
  float pitch = msg->pitch;
  float yaw = msg->yaw;

  bool cmd_thrust = msg->cmd[0];
  bool cmd_roll = msg->cmd[1];
  bool cmd_pitch = msg->cmd[2];
  bool cmd_yaw = msg->cmd[3];

  ROS_DEBUG("%s: Setting SICmd: %f, %f, %f, %f, [%i, %i, %i, %i]",
            ros::this_node::getName().c_str(),
            thrust, roll, pitch, yaw, 
            cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw);

  asctec_ll_interface.SetSICommand(thrust, roll, pitch, yaw,
                                   cmd_thrust, cmd_roll,
                                   cmd_pitch, cmd_yaw);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "asctec_ll");
  ros::NodeHandle n("~");

  double cmd_rate;
  n.param("cmd_rate", cmd_rate, 20.0);

  double ll_status_rate;
  n.param("ll_status_rate", ll_status_rate, -1.0);
  asctec_ll_interface.SetLLStatusRate(ll_status_rate);
  if (ll_status_rate > 0)
    {
      ll_status_pub = 
        n.advertise<asctec_ll::LLStatus>("ll_status", 1, true);
      asctec_ll_interface.SetLLStatusCallback(ll_status_callback);
    }

  double imu_raw_data_rate;
  n.param("imu_raw_data_rate", imu_raw_data_rate, -1.0);
  asctec_ll_interface.SetIMURawDataRate(imu_raw_data_rate);
  if (imu_raw_data_rate > 0)
    {
      imu_raw_data_pub = 
        n.advertise<asctec_ll::IMURawData>("imu_raw_data", 1);
      asctec_ll_interface.SetIMURawDataCallback(imu_raw_data_callback);
    }

  double imu_calc_data_rate;
  n.param("imu_calc_data_rate", imu_calc_data_rate, -1.0);
  asctec_ll_interface.SetIMUCalcDataRate(imu_calc_data_rate);
  if (imu_calc_data_rate > 0)
    {
      imu_calc_data_pub = 
        n.advertise<asctec_ll::IMUCalcData>("imu_calc_data", 1);
      asctec_ll_interface.SetIMUCalcDataCallback(imu_calc_data_callback);
    }

  double rc_data_rate;
  n.param("rc_data_rate", rc_data_rate, -1.0);
  asctec_ll_interface.SetRCDataRate(rc_data_rate);
  if (rc_data_rate > 0)
    {
      rc_data_pub = 
        n.advertise<asctec_ll::RCData>("rc_data", 1);     
      asctec_ll_interface.SetRCDataCallback(rc_data_callback);
    }

  double ctrl_out_rate;
  n.param("ctrl_out_rate", ctrl_out_rate, -1.0);
  asctec_ll_interface.SetCTRLOutRate(ctrl_out_rate);
  if (ctrl_out_rate > 0)
    {
      ctrl_out_pub = 
        n.advertise<asctec_ll::CTRLOut>("ctrl_out", 1);     
      asctec_ll_interface.SetCTRLOutCallback(ctrl_out_callback);
    }

  double gps_data_rate;
  n.param("gps_data_rate", gps_data_rate, -1.0);
  asctec_ll_interface.SetGPSDataRate(gps_data_rate);
  if (gps_data_rate > 0)
    {
      gps_data_pub = 
        n.advertise<asctec_ll::GPSData>("gps_data", 1);          
      asctec_ll_interface.SetGPSDataCallback(gps_data_callback);
    }

  double gps_data_advanced_rate;
  n.param("gps_data_advanced_rate", gps_data_advanced_rate, -1.0);
  asctec_ll_interface.SetGPSDataAdvancedRate(gps_data_advanced_rate);
  if (gps_data_advanced_rate > 0)
    {
      gps_data_advanced_pub = 
        n.advertise<asctec_ll::GPSDataAdvanced>("gps_data_advanced", 1);
      asctec_ll_interface.SetGPSDataAdvancedCallback(gps_data_advanced_callback);
    }
  
  std::string port;
  n.param("port", port, std::string("/dev/ttyS0"));

  if (asctec_ll_interface.Connect(port.c_str()) != 0)
    {
      ROS_ERROR("%s: unable to open port %s", 
                ros::this_node::getName().c_str(),
                port.c_str());
      return -1; 
    }

  ros::Subscriber cmd_sub = n.subscribe("cmd_si", 10, cmd_si_callback);
  ros::Timer timer = n.createTimer(ros::Duration(1.0/cmd_rate), cmd_callback);
 
  while (n.ok())
    {
      asctec_ll_interface.Update();
      
      ros::spinOnce();
    }

  asctec_ll_interface.Disconnect();

  return 0;
}
