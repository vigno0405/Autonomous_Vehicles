clear all;
close all;
clc;
cd('/home/vigno/ws_new/Assignment_III');
rosshutdown;
rosinit;

%% PART 1: definition of environment

% Communication object
gazebo = ExampleHelperGazeboCommunicator;

% Adding objects
redSphere = ExampleHelperGazeboModel("Ball");
red_color = [200, 48, 48, 255] * 1/255;
sphereLink1 = addLink(redSphere, "sphere", 0.1, "color", red_color);
spawnModel(gazebo, redSphere, [0, 6, 0]);

purpleSphere = ExampleHelperGazeboModel("Ball");
purple_color = [200, 0, 103, 255] * 1/255;
sphereLink2 = addLink(purpleSphere, "sphere", 0.2, "color", purple_color);
spawnModel(gazebo, purpleSphere, [6, 0, 0]);

blueSphere = ExampleHelperGazeboModel("Ball");
blue_color = [0, 0, 255, 255] * 1/255;
sphereLink3 = addLink(blueSphere, "sphere", 0.4, "color", blue_color);
spawnModel(gazebo, blueSphere, [-6, 0, 0]);

redSphereTest = ExampleHelperGazeboModel("Ball");
green_color = [200, 48, 48, 255] * 1/255;
sphereLink4 = addLink(redSphereTest, "sphere", 0.1, "color", red_color);
spawnModel(gazebo, redSphereTest, [0, -8, 0]);

%% PART 2: variables

max_area = 0;
yaw_rate_max = 0.4;
max_vel = 0.2;
k_w = 1;
f_cmd = 30;

%% PART 3: robot setup

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
setModelStateMsg.ModelState.ModelName = 'turtlebot3_waffle_pi';
setModelStateMsg.ModelState.Pose.Position.X = 0;
setModelStateMsg.ModelState.Pose.Position.Y = 0;
setModelStateMsg.ModelState.Pose.Position.Z = 0;
setModelStateMsg.ModelState.Pose.Orientation.X = 0;
setModelStateMsg.ModelState.Pose.Orientation.Y = 0;
setModelStateMsg.ModelState.Pose.Orientation.Z = 0;
setModelStateMsg.ModelState.Pose.Orientation.W = 1;
call(setStateService, setModelStateMsg);
% Omitting starting point

%% PART 4: Simulink control logic

warning('off', 'all')
%system('rosbag record -a -O /home/vigno/ws_new/Assignment_III/Bus_Ass_III.bag &');
out = sim('Simulink_ass_III.slx');
%system('pkill -f "rosbag record"');

%% PART 5: plotting robot trajectory and velocities

clear all;
clc;

% Bag extraction
bagselect = rosbag('Bus_Ass_III.bag');

% Extracting odom:
bagselect_odom = select(bagselect, 'Time', ...
    [bagselect.StartTime bagselect.EndTime], 'Topic', '/odom');
msg_odom = readMessages(bagselect_odom, 'DataFormat', 'Struct');

time_odom = double(bagselect_odom.MessageList.Time);
time_odom = time_odom;

% Robot trajectory
for ii = 1:length(msg_odom)
    xx(ii) = msg_odom{ii}.Pose.Pose.Position.X;
    yy(ii) = msg_odom{ii}.Pose.Pose.Position.Y;
end

% Plotting
for ii = 1:length(msg_odom)
    xx(ii) = msg_odom{ii}.Pose.Pose.Position.X;
    yy(ii) = msg_odom{ii}.Pose.Pose.Position.Y;
end

red_color = [200, 48, 48] * 1/255;
purple_color = [200, 0, 103] * 1/255;
blue_color = [0, 0, 255] * 1/255;
green_color = [0, 255, 0] * 1/255;

figure()
plot(xx, yy, 'b-', linewidth = 2);
title('Trajectory');
hold on;
plot(xx(1),yy(1),'rx', linewidth = 2);

% Coordinates and diameters of the spheres
sphere_positions = [0, 6; 6, 0; -6, 0; 0, -8]; % (x, y) positions
sphere_diameters = [0.1, 0.2, 0.4, 0.1]; % diameters
scatter(sphere_positions(:, 1), sphere_positions(:, 2), ...
    1000 * sphere_diameters.^2, ...
    [red_color; purple_color; blue_color; red_color], 'filled');

grid on;
box on;
legend('Trajectory', 'Starting Position', 'Spheres', 'Interpreter', ...
    'latex', 'Location', 'best');
xlim([-7, 7]);
ylim([-9, 7]);
xlabel('x [m]', 'Interpreter', 'latex');
ylabel('y [m]', 'Interpreter', 'latex');

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
title('Linear velocity (odom) [m/s]')
ylim([0, 0.3])
hold off

subplot(2,1,2)
hold on
plot(time_odom, w_odom, 'r-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Angular velocity (odom) [rad/s]')

hold off

linkaxes(findobj(gcf,'Type','axes'),'x');

%% PART 6: plotting

bagselect_covariance = select(bagselect, 'Time', ...
    [bagselect.StartTime bagselect.EndTime], 'Topic', '/covariance');
msg_covariance = readMessages(bagselect_covariance, 'DataFormat', 'Struct');

time_covariance = double(bagselect_covariance.MessageList.Time);
time_covariance = time_covariance;

for ii = 1:length(msg_covariance)
    x_center(ii) = msg_covariance{ii}.Covariance(1);
    y_center(ii) = msg_covariance{ii}.Covariance(2);
end

% Position
figure()

subplot(2,1,1)
hold on
plot(time_covariance, x_center, 'r-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('x', 'Interpreter', 'latex')
title('X center position (in camera) - 0 if nonexisting')
ylim([-10, 700])
hold off

subplot(2,1,2)
hold on
plot(time_covariance, y_center, 'b.', linewidth = 1)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('y', 'Interpreter', 'latex')
title('Y center position (in camera) - 0 if nonexisting')
hold off

linkaxes(findobj(gcf,'Type','axes'),'x');

% Command velocity
% Command velocity
bagselect_cmd = select(bagselect, 'Time',...
    [bagselect.StartTime bagselect.EndTime], 'Topic', '/cmd_vel');
msg_cmd = readMessages(bagselect_cmd, 'DataFormat', 'Struct');

time_cmd = double(bagselect_cmd.MessageList.Time);
time_cmd = time_cmd;

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
ylim([-0.1, 0.5])
hold off

linkaxes(findobj(gcf,'Type','axes'),'x');
