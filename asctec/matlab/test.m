imu = ipc_ros('ipc_sensor_msgs', 'sensor_msgs_Imu', 'imu', 'subscriber');
status = ipc_ros('asctec', 'asctec_Status', 'status', 'subscriber');
cmd = ipc_ros('asctec', 'asctec_Command', 'cmd', 'publisher');

for i = 1:5
    imu_msg = imu.read()
    status_msg = status.read()
end

m = 0.49;
g = 9.81;

msg = cmd.empty();
msg.thrust = m*g;
msg.kp_roll = 0.55;
msg.kp_pitch = 0.55;
msg.kp_yaw = 0.0;
msg.kd_roll = 0.25;
msg.kd_pitch = 0.25;
msg.kd_yaw = 0.15;
cmd.send(msg);

imu.disconnect();
status.disconnect();
cmd.disconnect();