clear
close all
clc

%% Creation of the map

n_pixel=5000;   % new rescaled dimension
[BW,BW_beforeClear]=ProcessMapGRAY_RRT('house_map.pgm',n_pixel,n_pixel);

%% Choose start and goal positions

% Ask positions to the user

Ngoals=input('How many goal points do you want to input?   ');
points=nan(Ngoals+1,2);
fig=figure;
imshow(BW)
hold on
for ii=1:(Ngoals+1)
    if ii==1
        title('Select the start point:');
    elseif ii==Ngoals+1
        title('Select last goal point:');
    else
        title(['Select goal point ', num2str(ii-1), ':']);
    end
    [x,y]=ginput(1);
    if ii==1
        plot(x,y,'ro','MarkerSize',10,'LineWidth',2);
    elseif ii==Ngoals+1
        plot(x,y,'go','MarkerSize',10,'LineWidth',2);
        pause(2)
    else
        plot(x,y,'go','MarkerSize',10,'LineWidth',2);
    end
    points(ii,:)=[x,y];
end
close(fig)

start_i=round(points(1,2));
start_j=round(points(1,1));
goal_i=round(points(2:end,2));
goal_j=round(points(2:end,1));

% Make sure index is within the borders

start_i=min(max(start_i,1),n_pixel);    
start_j=min(max(start_j,1),n_pixel);
goal_i=min(max(goal_i,1),n_pixel);
goal_j=min(max(goal_j,1),n_pixel);

% Start and goal in terms of indices (i,j)

start=[start_i start_j];
goal=[goal_i goal_j];

%% RRT algorithm to find the trajectory

% Parameters

max_iter=5e6;
step=n_pixel/5000*100;
patience=500;
dyn_plot=true;   % visualise evolution of the algorithm

% Path definition

tic
pathOutput=RRT_star_informed(BW,start,goal(1,:),max_iter,step,0.2,patience,true,dyn_plot);
for k=2:Ngoals
    pause(1)
    path_k=RRT_star_informed(BW,goal(k-1,:),goal(k,:),max_iter,step,0.2,patience,true,dyn_plot);
    pathOutput=[pathOutput; path_k(2:end,:)];
end
pause(1)
i_goals=goal(1:(end-1),:);
goal=goal(end,:);
toc

% Resample the trajectory

x_path=csaps(1:length(pathOutput),pathOutput(1:end,1),0.8,linspace(1,length(pathOutput),10001*length(pathOutput)));
y_path=csaps(1:length(pathOutput),pathOutput(1:end,2),0.8,linspace(1,length(pathOutput),10001*length(pathOutput)));

x_path=interp1(1:length(x_path),x_path(1:end),linspace(1,length(x_path),round(0.04*length(x_path))));
y_path=interp1(1:length(y_path),y_path(1:end),linspace(1,length(y_path),round(0.04*length(y_path))));

% Show the result

figure
hold on
PlotMap(BW,[0.7 0.7 0.7])
PlotMap(BW_beforeClear,'k')
plot(pathOutput(:,2),pathOutput(:,1),'r','LineWidth',2)
splot=plot(start(2),start(1),'bo','MarkerFaceColor','b');
gplot=plot(goal(2),goal(1),'go','MarkerFaceColor','g');
igplot=plot(i_goals(:,2),i_goals(:,1),'mo','MarkerFaceColor','m');
resamp=plot(y_path,x_path,'b');
xlabel('jj')
ylabel('ii')
title('RRT solution path')
axis tight
if Ngoals>1
    legend([splot gplot igplot resamp],'start','goal','intermediate goals','resampled traj','location','southwest')
else
    legend([splot gplot resamp],'start','goal','resampled traj','location','southwest')
end
hold off

%% Simulink initialization

% Change coordinates from BW to GAZEBO references

[path,res]=BW2GAZEBO([x_path' y_path'],n_pixel);
dist=sqrt(sum(diff(path).^2,2));   % distances between subcessive points of the reference trajectory for Simulink
reach_toll=70*mean(dist);          % tolerance for reaching the points of the reference trajectory

rosshutdown
setenv('ROS_HOSTNAME', 'localhost');  % set to local hostname
setenv('ROS_IP', '127.0.0.1');        % set to local IP
rosinit

% Set initial position of the turtlebot

call(rossvcclient('/gazebo/reset_world'), rosmessage(rossvcclient('/gazebo/reset_world')));  % reset world

setStateService=rossvcclient('/gazebo/set_model_state');
setModelStateMsg=rosmessage(setStateService);

angle=atan2(path(2,2)-path(1,2),path(2,1)-path(1,1));
angle=[angle,0,0];
quaternions=eul2quat(angle);

setModelStateMsg.ModelState.ModelName='turtlebot3';
setModelStateMsg.ModelState.Pose.Position.X=path(1,1);
setModelStateMsg.ModelState.Pose.Position.Y=path(1,2);
setModelStateMsg.ModelState.Pose.Orientation.W=quaternions(1);
setModelStateMsg.ModelState.Pose.Orientation.X=quaternions(2);
setModelStateMsg.ModelState.Pose.Orientation.Y=quaternions(3);
setModelStateMsg.ModelState.Pose.Orientation.Z=quaternions(4);
setModelStateMsg.ModelState.ReferenceFrame='world';

call(setStateService, setModelStateMsg);

%% Simulation

clc
% simNumber=input('Which number do you want to save in?   ');
out=sim('PID_houseFAST.slx');
% save(['path_RRT_',num2str(simNumber)],'out','path','BW_beforeClear','n_pixel','BW','pathOutput','start','goal','i_goals')

%% Simulation results

% Choose file to load

simNumber=input('Which file do you want to see the results of?   ');
load(['path_RRT_',num2str(simNumber)])

% Extract simulation results

t=out.tout;
x=out.x;
x_ref=out.x_ref;
y=out.y;
y_ref=out.y_ref;
theta=out.theta;
theta_ref=out.theta_ref;
v_cmd=out.v_cmd;
w_cmd=out.w_cmd;
v=out.v;
w=out.w;

% Remove first "ficticious" zeros in the vectors

x(x==0)=x(find(x~=0,1));
y(y==0)=y(find(y~=0,1));
x_ref(x_ref==0)=x_ref(find(x_ref~=0,1));
y_ref(y_ref==0)=y_ref(find(y_ref~=0,1));

% Convert trajectory from GAZEBO reference to BW reference for plot

path_sim=GAZEBO2BW([x,y],n_pixel);
path_ref=GAZEBO2BW([x_ref,y_ref],n_pixel);

% Trajectory

figure
hold on
PlotMap(BW,[0.7 0.7 0.7])
PlotMap(BW_beforeClear,'k')
ref=plot(path_ref(:,2),path_ref(:,1),'r--');
sim=plot(path_sim(:,2),path_sim(:,1),'c');
splot=plot(start(2),start(1),'bo','MarkerFaceColor','b');
gplot=plot(goal(2),goal(1),'go','MarkerFaceColor','g');
igplot=plot(i_goals(:,2),i_goals(:,1),'mo','MarkerFaceColor','m');
grid on
box on
xlabel('x [m]','Interpreter','latex')
ylabel('y [m]','Interpreter','latex')
title('Simulation - trajectory')
ytks=yticks;
xtks=xticks;
ytks=flip(ytks);
res_BW=0.05*384/n_pixel;  
xtks=xtks*res_BW-10;
ytks=ytks*res_BW-12;   
xtks=round(xtks,1);
ytks=round(ytks,1);
set(gca,'XAxisLocation','bottom','YTickLabels',ytks,'XTickLabels',xtks)
axis tight
if isempty(i_goals)
    legend([splot gplot ref sim],'start','goal','reference trajectory','actual trajectory','location','southwest')
else
    legend([splot gplot igplot ref sim],'start','goal','intermediate goals','reference trajectory','actual trajectory','location','southwest')
end
hold off

% Reference vs actual

figure

subplot(311)
hold on
plot(t,x)
plot(t,x_ref,'--')
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('x [m]','Interpreter','latex')
title('x position')
legend('actual x','$x_{ref}$','interpreter','latex','Location','best')
axis tight
hold off

subplot(312)
hold on
plot(t,y)
plot(t,y_ref,'--')
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('y [m]','Interpreter','latex')
title('y position')
legend('actual y','$y_{ref}$','interpreter','latex','Location','best')
axis tight
hold off

subplot(313)
hold on
plot(t,theta)
plot(t,theta_ref,'--')
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('$\theta$ [rad]','Interpreter','latex')
title('Angle')
legend('actual $\theta$','$\theta_{ref}$','interpreter','latex','Location','best')
axis tight
hold off

figure

subplot(211)
hold on
plot(t,v)
plot(t,v_cmd,'--')
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('v [m/s]','Interpreter','latex')
title('Linear velocity')
legend('actual v','$v_{cmd}$','interpreter','latex','Location','best')
axis tight
hold off

subplot(212)
hold on
plot(t,w)
plot(t,w_cmd,'--')
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('$\omega$ [rad/s]','Interpreter','latex')
title('Angular velocity')
legend('actual $\omega$','$\omega_{cmd}$','interpreter','latex','Location','best')
axis tight
hold off