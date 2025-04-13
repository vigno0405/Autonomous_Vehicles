clear all;
close all;
clc;
cd('/home/vigno/ws_new/Assignment_I')
pyenv('Version', '/usr/local/bin/python3.9');
rosshutdown;

%% PART 1: minimum distance from obstacles

clear all;
close all;
clc;

% Loading ROS bag:
bagselect = rosbag('ex_A1.bag');

% Extracting odom and scan:
bagselect_odom = select(bagselect, 'Time',...
    [bagselect.StartTime bagselect.EndTime], 'Topic', '/odom');
bagselect_scan = select(bagselect, 'Time',...
    [bagselect.StartTime bagselect.EndTime], 'Topic', '/scan');
msg_odom = readMessages(bagselect_odom, 'DataFormat', 'Struct');
msg_scan = readMessages(bagselect_scan, 'DataFormat', 'Struct');

time_odom = double(bagselect_odom.MessageList.Time);
time_scan = double(bagselect_scan.MessageList.Time);

% Plotting
for ii = 1:length(msg_odom)
    xx(ii) = msg_odom{ii}.Pose.Pose.Position.X;
    yy(ii) = msg_odom{ii}.Pose.Pose.Position.Y;
    W = msg_odom{ii}.Pose.Pose.Orientation.W;
    Z = msg_odom{ii}.Pose.Pose.Orientation.Z;
    theta(ii) = 2 * atan2(Z, W);
end

figure()
plot(xx, yy, 'r', linewidth = 2);
title('Trajectory');
hold on;
plot(xx(1),yy(1),'rx', linewidth = 2);
legend('','starting position','Interpreter','latex','Location','best')
hold off;
grid on;
box on;
xlabel('x [m]', 'Interpreter', 'latex');
ylabel('y [m]', 'Interpreter', 'latex');

% Evaluate minimum distance:
angles = msg_scan{1}.AngleMin:msg_scan{1}.AngleIncrement:...
    msg_scan{1}.AngleMax;
ranges = zeros(length(msg_scan),length(angles));
min_distance = zeros(1, length(msg_scan));

for ii = 1:length(msg_scan)
    ranges(ii,:) = double(msg_scan{ii}.Ranges);
    min_distance(ii) = min(ranges(ii,:));
end

% Plotting distance
figure()
plot(time_scan, min_distance, linewidth = 2);
grid on;
box on;
xlabel('t [s]','Interpreter','latex');
ylabel('dist [m]','Interpreter','latex');
title('Minimum distance from obstacles');
hold off;

%% PART 2: input estimation

% Odometry:
for ii = 1:length(msg_odom)
    v_odom(ii) = msg_odom{ii}.Twist.Twist.Linear.X;
    w_odom(ii) = msg_odom{ii}.Twist.Twist.Angular.Z;
end

% Odometry frequency:
f_s = mean(1./diff(time_odom));

% Filtered signal:
v_odom_filtered = medfilt1(v_odom, 75);
w_odom_filtered = medfilt1(w_odom, 75);

% Plotting velocity and odometry:
figure()
hold on
plot(time_odom, v_odom, 'b--');
plot(time_odom, v_odom_filtered,'b');
grid on;
box on;
xlabel('t [s]','Interpreter','latex')
title('Provided linear velocity (odom)')
legend('provided v (odom) [m/s]','filtered v [m/s]',...
    'Interpreter','latex','Location','best')
axis tight
hold off

figure()
hold on
plot(time_odom, w_odom, 'r--');
plot(time_odom, w_odom_filtered, 'r');
grid on;
box on;
xlabel('t [s]','Interpreter','latex')
title('Provided angular velocity (odom)')
legend('provided $\omega$ (odom) [rad/s]',...
    'filtered $\omega$ [rad/s]','Interpreter','latex','Location','best')
axis tight
hold off

v_cmd = 0.01*round(v_odom_filtered/0.01);
w_cmd = 0.1*round(w_odom_filtered/0.1);

% Plotting the approximation:
figure()
hold on;
plot(time_odom, v_odom, 'b--')
plot(time_odom, v_cmd, 'b.')
grid on;
box on;
xlabel('t [s]', 'Interpreter', 'latex')
title('Command linear velocity')
legend('provided v (odom) [m/s]','generated v [m/s]', ...
    'Interpreter','latex','Location','best')
axis tight
hold off

figure()
hold on
plot(time_odom,w_odom,'r--')
plot(time_odom,w_cmd,'r.')
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Command angular velocity')
legend('provided $\omega$ (odom) [rad/s]','generated $\omega$ [rad/s]', ...
    'Interpreter','latex','Location','best')
axis tight
hold off

%% PART 3: plotting the environment shown by lidar

coherent_xx = interp1(time_odom, xx, time_scan);
coherent_yy = interp1(time_odom, yy, time_scan);
coherent_theta = interp1(time_odom, theta, time_scan);

% Finding not-null ranges and distances:
scanned_info = zeros(0, 2);
indices = [];
for ii = 1:length(msg_scan)
    valid = msg_scan{ii}.Ranges < inf;
    result = reshape([msg_scan{ii}.Ranges(valid)', angles(valid)], ...
        sum(valid), 2);
    indices = [indices, ones(1, sum(valid))*ii];
    scanned_info = [scanned_info; result];
end
scanned_info = round(scanned_info, 2);
[scanned_info, remaining] = unique(scanned_info, 'rows', 'stable');
indices = indices(remaining);
scanned_points = zeros(0, 2);

for ii = 1:length(msg_scan)
    result_xx = coherent_xx(ii) + ...
        scanned_info(indices == ii, 1).*cos(coherent_theta(ii) + ...
        scanned_info(indices == ii, 2));
    result_yy = coherent_yy(ii) + ...
        scanned_info(indices == ii, 1).*sin(coherent_theta(ii) + ...
        scanned_info(indices == ii, 2));
    result = reshape([result_xx, result_yy], size(result_xx, 1), 2);
    scanned_points = [scanned_points; result];
end

% Plotting scanned points:
figure()
axis equal;
plot(scanned_points(:, 1), scanned_points(:, 2), '.');
hold on;
grid on;
plot(xx, yy, 'r', linewidth = 2);
xlabel('x [m]','Interpreter','latex');
ylabel('y [m]','Interpreter','latex');
title('Path inferred by scanner');
hold off;

% This means that there is a rotation at the beginning (since it doesnt't
% move in x and y at the beginning, it 'looks around').

%% PART 4: Simulink (bringing the turtlebot to the beginning)

rosshutdown
rosinit
time_cmd = time_odom;

v_ts = timeseries(v_cmd, time_cmd);
w_ts = timeseries(w_cmd, time_cmd);
null = timeseries(zeros(1, length(msg_odom)), time_cmd);
% null for the ones which are not used

% Establish command frequency
f_cmd = round(f_s);

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
setModelStateMsg.ModelState.Pose.Position.X = ...
    msg_odom{1}.Pose.Pose.Position.X;
setModelStateMsg.ModelState.Pose.Position.Y = ...
    msg_odom{1}.Pose.Pose.Position.Y;
setModelStateMsg.ModelState.Pose.Position.Z = ...
    msg_odom{1}.Pose.Pose.Position.Z;
setModelStateMsg.ModelState.Pose.Orientation.X = ...
    msg_odom{1}.Pose.Pose.Orientation.X;
setModelStateMsg.ModelState.Pose.Orientation.Y = ...
    msg_odom{1}.Pose.Pose.Orientation.Y;
setModelStateMsg.ModelState.Pose.Orientation.Z = ...
    msg_odom{1}.Pose.Pose.Orientation.Z;
setModelStateMsg.ModelState.Pose.Orientation.W = ...
    msg_odom{1}.Pose.Pose.Orientation.W;

call(setStateService, setModelStateMsg);

% Simulate
warning('off', 'all')
system('rosbag record -a -O /home/vigno/ws_new/Assignment_I/Bus_Ass_I.bag &');
out = sim('Assignment_I_simulink.slx');
system('pkill -f "rosbag record"');
save('data.mat')

%% PART 5: graphical representation

clear all;
clc;
load('data.mat');
% Plot the results of the simulation:
t_sim = out.tout;
x_sim = out.x_sim;
y_sim = out.y_sim;
v_sim = out.v_sim;
w_sim = out.w_sim;

figure()
hold on
plot(xx(1), yy(1), 'bx')
plot(xx, yy, 'b--')
plot(x_sim, y_sim, 'b')
grid on
box on
xlabel('y[m]', 'Interpreter','latex')
ylabel('x[m]', 'Interpreter', 'latex')
title('Trajectory');
legend('starting position','provided','simulated','Interpreter', ...
    'latex','Location','best')
hold off;

% Difference in the trajectory could be caused by different sampling
% frequency, or by not perfectly accurate evaluation of the commands.

%% PART 6: plotting velocities

% Linear velocity
figure()

subplot(3,1,1)
hold on
plot(time_odom, v_odom, 'b-');
grid on
box on
xlabel('t [s]', 'Interpreter', 'latex')
title('Provided linear velocities (odom)')
hold off

subplot(3,1,2)
hold on
plot(time_cmd,v_cmd,'b-')
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Simulation linear velocities (command)')
hold off

subplot(3,1,3)
hold on
plot(t_sim, v_sim, 'b-', linewidth = 2)
plot(time_odom, v_odom, 'c-')
legend('simulation','provided','Interpreter','latex','Location','best')
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Simulation linear velocities (odom)')
hold off

% Angular velocity
figure()

subplot(3,1,1)
hold on
plot(time_odom, w_odom, 'r-');
grid on
box on
xlabel('t [s]', 'Interpreter', 'latex')
title('Provided angular velocities (odom)')
hold off

subplot(3,1,2)
hold on
plot(time_cmd,w_cmd,'r-')
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Simulation angular velocities (command)')
ylim([-0.25, 0.15])
hold off

subplot(3,1,3)
hold on
plot(t_sim,w_sim,'r-', linewidth = 2)
plot(time_odom, w_odom, 'm-')
legend('simulation','provided','Interpreter','latex','Location','best')
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Simulation angular velocities (odom)')
hold off

%% PART 7: understanding from the bag

clear all;
clc;

bagselect_prov = rosbag('ex_A1.bag');
bagselect_odom_prov = select(bagselect_prov, 'Time',...
    [bagselect_prov.StartTime bagselect_prov.EndTime], 'Topic', '/odom');
msg_odom_prov = readMessages(bagselect_odom_prov, 'DataFormat', 'Struct');
time_prov = double(bagselect_odom_prov.MessageList.Time);
time_prov = time_prov - time_prov(1);

bagselect_cmd = rosbag('Bus_Ass_I.bag');
bagselect_odom_cmd = select(bagselect_cmd, 'Time',...
    [bagselect_cmd.StartTime bagselect_cmd.EndTime], 'Topic', '/odom');
msg_odom_cmd = readMessages(bagselect_odom_cmd, 'DataFormat', 'Struct');
time_cmd = double(bagselect_odom_cmd.MessageList.Time);
time_cmd = time_cmd - time_cmd(1);

for ii = 1:length(msg_odom_prov)
    xx_prov(ii) = msg_odom_prov{ii}.Pose.Pose.Position.X;
    yy_prov(ii) = msg_odom_prov{ii}.Pose.Pose.Position.Y;
end

for ii = 1:length(msg_odom_cmd)
    xx_cmd(ii) = msg_odom_cmd{ii}.Pose.Pose.Position.X;
    yy_cmd(ii) = msg_odom_cmd{ii}.Pose.Pose.Position.Y;
end

for ii = 1:length(msg_odom_prov)
    v_odom_prov(ii) = msg_odom_prov{ii}.Twist.Twist.Linear.X;
    w_odom_prov(ii) = msg_odom_prov{ii}.Twist.Twist.Angular.Z;
end

for ii = 1:length(msg_odom_cmd)
    v_odom_cmd(ii) = msg_odom_cmd{ii}.Twist.Twist.Linear.X;
    w_odom_cmd(ii) = msg_odom_cmd{ii}.Twist.Twist.Angular.Z;
end

figure()
hold on
plot(xx_prov(1), yy_prov(1), 'bx')
plot(xx_prov, yy_prov, 'b--')
plot(xx_cmd, yy_cmd, 'b')
grid on
box on
xlabel('y[m]', 'Interpreter','latex')
ylabel('x[m]', 'Interpreter', 'latex')
title('Trajectory');
legend('starting position','provided','simulated','Interpreter', ...
    'latex','Location','best')
hold off;

% Linear velocity
figure()

subplot(2,1,1)
hold on
plot(time_cmd, v_odom_cmd, 'b-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Simulation linear velocity')
hold off

subplot(2,1,2)
hold on
plot(time_prov, v_odom_prov, 'r-', linewidth = 2)
legend('simulation','provided','Interpreter','latex','Location','best')
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Provided linear velocity (odom)')
hold off

linkaxes(findobj(gcf,'Type','axes'),'x');

% Angular velocity
figure()

subplot(2,1,1)
hold on
plot(time_cmd, w_odom_cmd, 'b-', linewidth = 2)
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Simulation angular velocity')
hold off

subplot(2,1,2)
hold on
plot(time_prov, w_odom_prov, 'r-', linewidth = 2)
legend('simulation','provided','Interpreter','latex','Location','best')
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Provided angular velocity (odom)')
hold off

linkaxes(findobj(gcf,'Type','axes'),'x');

% Results are actually very similar (good reverse engineering)

%% Distances

% Loading ROS bag:
bagselect_ex = rosbag('ex_A1.bag');
bagselect_mine = rosbag('Bus_Ass_I.bag');

% Extracting odom and scan:
bagselect_scan_ex = select(bagselect_ex, 'Time', ...
    [bagselect_ex.StartTime bagselect_ex.EndTime], 'Topic', '/scan');
bagselect_scan_mine = select(bagselect_mine, 'Time', ...
    [bagselect_mine.StartTime bagselect_mine.EndTime], 'Topic', '/scan');

msg_scan_ex = readMessages(bagselect_scan_ex, 'DataFormat', 'Struct');
msg_scan_mine = readMessages(bagselect_scan_mine, 'DataFormat', 'Struct');
time_prov = bagselect_scan_ex.MessageList.Time;
time_cmd = bagselect_scan_mine.MessageList.Time;

min_distance_ex = zeros(1, length(msg_scan_ex));
min_distance_mine = zeros(1, length(msg_scan_mine));

for ii = 1:length(msg_scan_ex)
    ranges(ii,:) = double(msg_scan_ex{ii}.Ranges);
    min_distance_ex(ii) = min(ranges(ii,:));
end

for ii = 1:length(msg_scan_mine)
    ranges(ii,:) = double(msg_scan_mine{ii}.Ranges);
    min_distance_mine(ii) = min(ranges(ii,:));
end

% Plotting
figure()
subplot(2,1,1)
plot(time_prov, min_distance_ex, 'LineWidth', 2, 'Color', 'r');
grid on;
box on;
xlabel('t [s]','Interpreter','latex');
ylabel('dist [m]','Interpreter','latex');
title('Minimum distance from obstacles - Provided');
legend('Provided','Interpreter','latex','Location','southwest');

subplot(2,1,2)
plot(time_cmd, min_distance_mine, 'LineWidth', 2, 'Color', 'b');
grid on;
box on;
xlabel('t [s]','Interpreter','latex');
ylabel('dist [m]','Interpreter','latex');
title('Minimum distance from obstacles - Simulation');
legend('Simulation','Interpreter','latex','Location','southwest');

