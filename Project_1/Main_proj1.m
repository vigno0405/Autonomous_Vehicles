clear
close all
clc

%% Load image and build map

n=10000;        % new rescaled dimension [px]
real_dim=300;   % real map dimension [m]

choose_map='map_1_d.png';
[map,start,goal,~]=MapElaboration(choose_map,n,n,0.99);

%% RRT algorithm

max_iter=1e6;    % max number of iterations
step=200;        % max step distance for any new node of the tree
bias=0.2;        % biasing the growth direction of the tree

[niter,nnodes,path_length,elatime]=RRT(map,start,goal,max_iter,step,bias,true,true);
path_length=300*path_length/n;

fprintf('RRT:\n\titerations: %d\n\tnodes: %d\n\tpath length: %.2f [m]\n\ttime: %.4f [s]\n\n',niter,nnodes,path_length,elatime)
pause(2)
 
%% RRT-connect algorithm

max_iter=1e6;    % max number of iterations
step=200;        % max step distance for any new node of the tree
bias=0.2;        % biasing the growth direction of the tree
greedy=2.5;      % greedy behaviour in connecting the trees

[niter,nnodes,path_length,elatime]=RRT_connect(map,start,goal,max_iter,step,bias,greedy,true);
path_length=300*path_length/n;

fprintf('RRT connected:\n\titerations: %d\n\tnodes: %d\n\tpath length: %.2f [m]\n\ttime: %.4f [s]\n\n',niter,nnodes,path_length,elatime)
pause(2)

%% Informed RRT* algorithm

max_iter=1e6;    % max number of iterations
step=200;        % max step distance for any new node of the tree
bias=0.2;        % biasing the growth direction of the tree
patience=500;    % tolerance on the refinement of the solution
informed=true;   % informed RRT*

[niter,nnodes,path_length,elatime]=RRT_star_informed(map,start,goal,max_iter,step,bias,patience,informed,true);
path_length=300*path_length/n;

fprintf('RRT star:\n\titerations: %d\n\tnodes: %d\n\tpath length: %.2f [m]\n\ttime: %.4f [s]\n\n',niter,nnodes,path_length,elatime)
pause(2)

%% A star (for comparison)

n=1000;   % new rescaled dimension
[map,start,goal,N_free]=MapElaboration(choose_map,n,n,0.99);
map=~map;   % invert 0 and 1 (matlab function works like this)
mapObj=binaryOccupancyMap(map,1);

tic
planner=plannerAStarGrid(mapObj,"HCost","Euclidean");
[pathOutput,info]=plan(planner,start,goal);
path_length=300*info.PathCost/n;
elatime=toc;

fprintf('A star:\n\texplored nodes: %g/%d\n\tpath length: %.2f [m]\n\ttime: %.4f [s]\n\n',info.NumNodesExplored,N_free,path_length,elatime)
figure
show(planner)
legend