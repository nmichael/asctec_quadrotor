addpath('~/Desktop/dynamics');

odom = ipc_ros('ipc_nav_msgs', 'nav_msgs_Odometry', ...
               'odom', 'subscriber');
cmd = ipc_ros('asctec', 'asctec_Command', 'cmd', 'publisher');

cmd_msg = cmd.empty();

m = 0.5;
g = 9.81;

scale_p = 3.24;
scale_d = 3.6;

scale_p = 1.0*scale_p;
scale_d = 1.0*scale_d;

t_z = 0.3; %rise time for z controller
xi_z = 1.0; %damping ratio for z controller
kp_z = scale_p/t_z^2*(1/m);
kd_z = scale_d*xi_z/t_z*(1/m);

t_xy = 0.7; %rise time for xy position control
xi_xy = 1.0; %damping factor for x and y controller
% kp_xy = scale_p/t_xy^2;
% kd_xy = scale_d*xi_xy/t_xy;
kp_xy = 1.0;
kd_xy = 1.0;

Ixx = 2.32e-3;
Iyy = 2.32e-3;
Izz = 4.41e-3;

t_attitude = 0.15; %rise time of the attitude controller
xi_attitude = 1; %damping ratio for attitude controller

cmd_msg.kp_roll = scale_p*Ixx/t_attitude^2;
cmd_msg.kd_roll = scale_d*xi_attitude*Ixx/t_attitude;

cmd_msg.kp_pitch = scale_p*Iyy/t_attitude^2;
cmd_msg.kd_pitch = scale_d*xi_attitude*Ixx/t_attitude;

t_yaw = 0.2; %rime time of yaw controller
xi_yaw = 1.0; %damping ratio for yaw controller
cmd_msg.kp_yaw = scale_p*Izz/t_yaw^2;
cmd_msg.kd_yaw = scale_d*xi_yaw*Izz/t_yaw;

max_asin = sin(40*pi/180);

x_des = 0.05;
y_des = 0.0;
z_des = 2.0;
psi_des = 0.0;

cmd_msg.z_correction = 0;

for i = 1:10000
    msg = odom.read();

    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    z = msg.pose.pose.position.z;

    xd = msg.twist.twist.linear.x;
    yd = msg.twist.twist.linear.y;
    zd = msg.twist.twist.linear.z;

    [~, ~, psi] = R_To_ZXY(Quat_To_R(msg.pose.pose.orientation.x, ...
                                     msg.pose.pose.orientation.y, ...
                                     msg.pose.pose.orientation.z, ...
                                     msg.pose.pose.orientation.w));

    th_cmd = m*g + kp_z*(z_des - z) - kd_z*zd;
    th_cmd = max(0, th_cmd);

    ux = kp_xy*(x_des - x) - kd_xy*xd;
    uy = kp_xy*(y_des - y) - kd_xy*yd;

    phides = ux*sin(psi) - uy*cos(psi);
    thetades = ux*cos(psi) + uy*sin(psi);

    phides = min(max(-max_asin, phides), max_asin);
    thetades = min(max(-max_asin, thetades), max_asin);

    cmd_msg.thrust = th_cmd;
    cmd_msg.roll = phides;
    cmd_msg.pitch = thetades;
    cmd_msg.yaw = psi_des;

    cmd.send(cmd_msg);
end

odom.disconnect();
cmd.disconnect();
