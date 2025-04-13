
clear all;
close all;
clc;
cd('/home/vigno/ws_new/Assignment_II');
pyenv('Version', '/usr/local/bin/python3.9');
rosshutdown;

%% PART 1: incorporate waypoints list

% Definition of variables
turtlebot3 = [
    0, 0, 0;
    5, 0, pi/2;
    5, 5, 5/4*pi;
    -5, -5, pi/2;
    -5, 5, 0;
    0, 0, 0;
    3, 3, 3/4*pi;
    -3, 0, 3/2*pi;
    0, -3, pi/4;
    3, 0, pi/2;
    0, 0, 3/2*pi
];

% Constraints
dist_max = 0.01;
ang_max = 0.05;
yaw_rate_max = 0.4;
v_max = 0.2;

% Gains
k_v = 3;
k_w_direction = 3;
k_w_heading = 1;

%% PART 2: plot waypoints

figure()
for ii = 1:size(turtlebot3, 1)
    L = 1;
    x_center = turtlebot3(ii, 1);
    y_center = turtlebot3(ii, 2);
    theta_rad = turtlebot3(ii, 3);

    x1 = x_center - L/2 * cos(theta_rad);
    y1 = y_center - L/2 * sin(theta_rad);
    x2 = x_center + L/2 * cos(theta_rad);
    y2 = y_center + L/2 * sin(theta_rad);

    plot([x1, x2], [y1, y2], '-o');
    axis equal;
    xlabel('x [m]', 'Interpreter', 'latex');
    ylabel('y [m]', 'Interpreter', 'latex');
    grid on;
    title('Waypoints');
    hold on;
    box on;
    xlim([-8 8]);
    ylim([-8 8]);
    text(x_center, y_center, num2str(ii), 'VerticalAlignment', 'bottom', ...
        'HorizontalAlignment', 'right');
    quiver(x_center, y_center, cos(theta_rad), sin(theta_rad), 0.5, ...
        'MaxHeadSize', 2, 'Color', 'r');
end


%% PART 3: simulation setup

rosshutdown
rosinit
f_cmd = 30;

% Initialise ROS node
ipaddress = 'localhost';
tbot = turtlebot(ipaddress);
% Reset initial position and time of the turtlebot in gazebo
pause(1)
resetService = rossvcclient('/gazebo/reset_world');
call(resetService);

% Set initial position of the turtlebot
setStateService = rossvcclient('/gazebo/set_model_state');
setModelStateMsg = rosmessage(setStateService);
setModelStateMsg.ModelState.ModelName = 'turtlebot3_burger';
setModelStateMsg.ModelState.Pose.Position.X = 0;
setModelStateMsg.ModelState.Pose.Position.Y = 0;
setModelStateMsg.ModelState.Pose.Position.Z = 0;
setModelStateMsg.ModelState.Pose.Orientation.X = 0;
setModelStateMsg.ModelState.Pose.Orientation.Y = 0;
setModelStateMsg.ModelState.Pose.Orientation.Z = 0;
setModelStateMsg.ModelState.Pose.Orientation.W = 1;
call(setStateService, setModelStateMsg);
% Omitting starting point

%% PART 4: simulate the basis request

warning('off', 'all')
ii = double(1);
new_waypoint = false;
%system('rosbag record -a -O /home/vigno/ws_new/Assignment_II/Bus_Ass_II.bag &');
sim('Assignment_II_simulink.slx');
%system('pkill -f "rosbag record"');

%% PART 5: from geometry node (python)

waypoints = zeros(1000, 5);
ii = double(1);
new_waypoint = false;
warning('off', 'all')
%system('rosbag record -a -O /home/vigno/ws_new/Assignment_II/Bus_Ass_II_extraA.bag &');
sim('Assignment_II_extraA.slx');
%system('pkill -f "rosbag record"');

%% PART 6: closed-loop posture regulation

% It could integrate the advantages of both approaches, but for the sake of
% simplicity we start from the original one.

% Definition of variables
turtlebot3 = [
    0, 0, 0;
    5, 0, pi/2;
    5, 5, 5/4*pi;
    -5, -5, pi/2;
    -5, 5, 0;
    0, 0, 0;
    3, 3, 3/4*pi;
    -3, 0, 3/2*pi;
    0, -3, pi/4;
    3, 0, pi/2;
    0, 0, 3/2*pi
];

% Constraints
dist_max = 0.01;
ang_max = 0.05;
yaw_rate_max = 0.4;
v_max = 0.2;

% Gains
k1 = 3;
k2 = 5;
k3 = 4;

warning('off', 'all')
ii = double(1);
new_waypoint = false;

%system('rosbag record -a -O /home/vigno/ws_new/Assignment_II/Bus_Ass_II_extraB.bag &');
sim('Assignment_II_extraB.slx');
%system('pkill -f "rosbag record"');

%% PART 7.1: plotting results (trivial case)

clear all;
clc;
max_v = 0.2;
max_w = 0.4;

turtlebot3 = [
    0, 0, 0;
    5, 0, pi/2;
    5, 5, 5/4*pi;
    -5, -5, pi/2;
    -5, 5, 0;
    0, 0, 0;
    3, 3, 3/4*pi;
    -3, 0, 3/2*pi;
    0, -3, pi/4;
    3, 0, pi/2;
    0, 0, 3/2*pi
];

% Loading ROS bag:
bagselect = rosbag('Bus_Ass_II.bag');

% Extracting odom:
bagselect_odom = select(bagselect, 'Time',...
    [bagselect.StartTime bagselect.EndTime], 'Topic', '/odom');
msg_odom = readMessages(bagselect_odom, 'DataFormat', 'Struct');

time_odom = double(bagselect_odom.MessageList.Time);

% Plotting
for ii = 1:length(msg_odom)
    xx(ii) = msg_odom{ii}.Pose.Pose.Position.X;
    yy(ii) = msg_odom{ii}.Pose.Pose.Position.Y;
end

figure()
plot(xx, yy, 'b-', linewidth = 2);
title('Trajectory');
hold on;
plot(xx(1),yy(1),'rx', linewidth = 2);
grid on;
box on;
xlabel('x [m]', 'Interpreter', 'latex');
ylabel('y [m]', 'Interpreter', 'latex');

for ii = 1:size(turtlebot3, 1)
    L = 0.5;
    x_center = turtlebot3(ii, 1);
    y_center = turtlebot3(ii, 2);
    theta_rad = turtlebot3(ii, 3);

    x1 = x_center - L/2 * cos(theta_rad);
    y1 = y_center - L/2 * sin(theta_rad);
    x2 = x_center + L/2 * cos(theta_rad);
    y2 = y_center + L/2 * sin(theta_rad);

    plot([x1, x2], [y1, y2], '-o');
    axis equal;
    xlabel('x [m]', 'Interpreter', 'latex');
    ylabel('y [m]', 'Interpreter', 'latex');
    grid on;
    hold on;
    box on;
    xlim([-7 7]);
    ylim([-7 7]);
    text(x_center, y_center, num2str(ii), 'VerticalAlignment', 'bottom', ...
        'HorizontalAlignment', 'right');
    quiver(x_center, y_center, cos(theta_rad), sin(theta_rad), 0.5, ...
        'MaxHeadSize', 2, 'Color', 'r');
end
hold off;

time_odom = time_odom - time_odom(1);

for ii = 1:length(msg_odom)
    v_odom(ii) = msg_odom{ii}.Twist.Twist.Linear.X;
    w_odom(ii) = msg_odom{ii}.Twist.Twist.Angular.Z;
end

% Velocity
figure()

subplot(2,1,1)
hold on
plot(time_odom, v_odom, 'b-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Linear velocity [m/s]')
ylim([0, 0.3]);
hold off

subplot(2,1,2)
hold on
plot(time_odom, w_odom, 'r-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Angular velocity [rad/s]')
yline(max_w, 'b--')
yline(-max_w, 'b--')
ylim([-1, 1])
hold off

linkaxes(findobj(gcf,'Type','axes'),'x');

% Command velocity
bagselect_cmd = select(bagselect, 'Time',...
    [bagselect.StartTime bagselect.EndTime], 'Topic', '/cmd_vel');
msg_cmd = readMessages(bagselect_cmd, 'DataFormat', 'Struct');

time_cmd = double(bagselect_cmd.MessageList.Time);

time_cmd = time_cmd - time_cmd(1);

for ii = 1:length(msg_cmd)
    v_cmd(ii) = msg_cmd{ii}.Linear.X;
    w_cmd(ii) = msg_cmd{ii}.Angular.Z;
end

% Velocity
figure()

subplot(2,1,1)
hold on
plot(time_cmd, v_cmd, 'b-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Command linear velocity [m/s]')
ylim([0, 0.3])
hold off

subplot(2,1,2)
hold on
plot(time_cmd, w_cmd, 'r-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Command angular velocity [rad/s]')
yline(max_w, 'b--')
yline(-max_w, 'b--')
ylim([-1, 1])
hold off

linkaxes(findobj(gcf,'Type','axes'),'x');

%% PART 7.2: plotting results (extra A)

close all

% This just differs by a constant at the beginning, needed to subscribe.
max_v = 0.2;
max_w = 0.4;

% Loading ROS bag:
bagselect = rosbag('Bus_Ass_II_extraA.bag');

% Extracting odom:
bagselect_odom = select(bagselect, 'Time',...
    [bagselect.StartTime bagselect.EndTime], 'Topic', '/odom');
msg_odom = readMessages(bagselect_odom, 'DataFormat', 'Struct');

time_odom = double(bagselect_odom.MessageList.Time);

% Plotting
for ii = 1:length(msg_odom)
    xx(ii) = msg_odom{ii}.Pose.Pose.Position.X;
    yy(ii) = msg_odom{ii}.Pose.Pose.Position.Y;
end

figure()
plot(xx, yy, 'b-', linewidth = 2);
title('Trajectory');
hold on;
plot(xx(1),yy(1),'rx', linewidth = 2);
grid on;
box on;
xlabel('x [m]', 'Interpreter', 'latex');
ylabel('y [m]', 'Interpreter', 'latex');

for ii = 1:size(turtlebot3, 1)
    L = 0.5;
    x_center = turtlebot3(ii, 1);
    y_center = turtlebot3(ii, 2);
    theta_rad = turtlebot3(ii, 3);

    x1 = x_center - L/2 * cos(theta_rad);
    y1 = y_center - L/2 * sin(theta_rad);
    x2 = x_center + L/2 * cos(theta_rad);
    y2 = y_center + L/2 * sin(theta_rad);

    plot([x1, x2], [y1, y2], '-o');
    axis equal;
    xlabel('x [m]', 'Interpreter', 'latex');
    ylabel('y [m]', 'Interpreter', 'latex');
    grid on;
    hold on;
    box on;
    xlim([-7 7]);
    ylim([-7 7]);
    text(x_center, y_center, num2str(ii), 'VerticalAlignment', 'bottom', ...
        'HorizontalAlignment', 'right');
    quiver(x_center, y_center, cos(theta_rad), sin(theta_rad), 0.5, ...
        'MaxHeadSize', 2, 'Color', 'r');
end
hold off;

time_odom = time_odom - time_odom(1);

for ii = 1:length(msg_odom)
    v_odom(ii) = msg_odom{ii}.Twist.Twist.Linear.X;
    w_odom(ii) = msg_odom{ii}.Twist.Twist.Angular.Z;
end

% Velocity
figure()

subplot(2,1,1)
hold on
plot(time_odom, v_odom, 'b-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Linear velocity [m/s]')
ylim([0, 0.3])
yline(max_v, 'r--');
hold off

subplot(2,1,2)
hold on
plot(time_odom, w_odom, 'r-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Angular velocity [rad/s]')
ylim([-1, 1])
yline(max_w, 'b--')
yline(-max_w, 'b--')
hold off

linkaxes(findobj(gcf,'Type','axes'),'x');

% Command velocity
bagselect_cmd = select(bagselect, 'Time',...
    [bagselect.StartTime bagselect.EndTime], 'Topic', '/cmd_vel');
msg_cmd = readMessages(bagselect_cmd, 'DataFormat', 'Struct');

time_cmd = double(bagselect_cmd.MessageList.Time);

time_cmd = time_cmd - time_cmd(1);

for ii = 1:length(msg_cmd)
    v_cmd(ii) = msg_cmd{ii}.Linear.X;
    w_cmd(ii) = msg_cmd{ii}.Angular.Z;
end

% Velocity
figure()

subplot(2,1,1)
hold on
plot(time_cmd, v_cmd, 'b-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylim([0, 0.3])
title('Command linear velocity [m/s]')
hold off

subplot(2,1,2)
hold on
plot(time_cmd, w_cmd, 'r-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Command angular velocity [rad/s]')
ylim([-1, 1])
yline(max_w, 'b--')
yline(-max_w, 'b--')
hold off

linkaxes(findobj(gcf,'Type','axes'),'x');

% They are not in phase due to the beginning of the pubblication of the
% message, since we have the time required for the simulation to read the
% Python node (during which the simulation is however running).

%% PART 7.3: plotting results (extra B)

close all;
max_v = 0.2;
max_w = 0.4;
% Loading ROS bag:
bagselect = rosbag('Bus_Ass_II_extraB.bag');

% Extracting odom:
bagselect_odom = select(bagselect, 'Time',...
    [bagselect.StartTime bagselect.EndTime], 'Topic', '/odom');
msg_odom = readMessages(bagselect_odom, 'DataFormat', 'Struct');

time_odom = double(bagselect_odom.MessageList.Time);

% Plotting
for ii = 1:length(msg_odom)
    xx(ii) = msg_odom{ii}.Pose.Pose.Position.X;
    yy(ii) = msg_odom{ii}.Pose.Pose.Position.Y;
end

figure()
plot(xx, yy, 'b-', linewidth = 2);
title('Trajectory');
hold on;
plot(xx(1),yy(1),'rx', linewidth = 2);
grid on;
box on;
xlabel('x [m]', 'Interpreter', 'latex');
ylabel('y [m]', 'Interpreter', 'latex');

for ii = 1:size(turtlebot3, 1)
    L = 0.5;
    x_center = turtlebot3(ii, 1);
    y_center = turtlebot3(ii, 2);
    theta_rad = turtlebot3(ii, 3);

    x1 = x_center - L/2 * cos(theta_rad);
    y1 = y_center - L/2 * sin(theta_rad);
    x2 = x_center + L/2 * cos(theta_rad);
    y2 = y_center + L/2 * sin(theta_rad);

    plot([x1, x2], [y1, y2], '-o');
    axis equal;
    xlabel('x [m]', 'Interpreter', 'latex');
    ylabel('y [m]', 'Interpreter', 'latex');
    grid on;
    hold on;
    box on;
    xlim([-12 12]);
    ylim([-12 12]);
    text(x_center, y_center, num2str(ii), 'VerticalAlignment', 'bottom', ...
        'HorizontalAlignment', 'right');
    quiver(x_center, y_center, cos(theta_rad), sin(theta_rad), 0.5, ...
        'MaxHeadSize', 2, 'Color', 'r');
end
hold off;

time_odom = time_odom - time_odom(1);

for ii = 1:length(msg_odom)
    v_odom(ii) = msg_odom{ii}.Twist.Twist.Linear.X;
    w_odom(ii) = msg_odom{ii}.Twist.Twist.Angular.Z;
end

% Velocity
figure()

subplot(2,1,1)
hold on
plot(time_odom, v_odom, 'b-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Linear velocity [m/s]')
ylim([0, 0.3])
yline(max_v, 'r--');
hold off

subplot(2,1,2)
hold on
plot(time_odom, w_odom, 'r-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Angular velocity [rad/s]')
ylim([-1, 1])
yline(max_w, 'b--')
yline(-max_w, 'b--')
hold off

linkaxes(findobj(gcf,'Type','axes'),'x');

% Command velocity
bagselect_cmd = select(bagselect, 'Time',...
    [bagselect.StartTime bagselect.EndTime], 'Topic', '/cmd_vel');
msg_cmd = readMessages(bagselect_cmd, 'DataFormat', 'Struct');

time_cmd = double(bagselect_cmd.MessageList.Time);

time_cmd = time_cmd - time_cmd(1);

for ii = 1:length(msg_cmd)
    v_cmd(ii) = msg_cmd{ii}.Linear.X;
    w_cmd(ii) = msg_cmd{ii}.Angular.Z;
end

% Velocity
figure()

subplot(2,1,1)
hold on
plot(time_cmd, v_cmd, 'b-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Command linear velocity [m/s]')
ylim([0, 0.3])
hold off

subplot(2,1,2)
hold on
plot(time_cmd, w_cmd, 'r-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Command angular velocity [rad/s]')
ylim([-1, 1])
yline(max_w, 'b--')
yline(-max_w, 'b--')
hold off

linkaxes(findobj(gcf,'Type','axes'),'x');



