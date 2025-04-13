function [map_out,start,goal,nfree]=MapElaboration(map_in,n,m,BW_tresh)
%
% Elaborates the provided NxMx3 RGB map.
%
%   [map_out,start,goal,nfree]=MapElaboration(map_in,n,m,BW_tresh)
%
% Inputs:
%   map_in........provided map: NxMx3 RGB with gray and black (obstacles),
%                 white (free), green (start) and red (goal) pixels
%   n,m...........new dimensions
%   BW_tresh......grey scale treshold for distinguishing obstacles from
%                 free nodes
%
% Outputs:
%   map_out.......binary output rescaled matrix: nxm binary matrix with 1
%                 (free nodes) and 0 (not free nodes)
%   start,goal....positions of start and final point: (ii,jj) indices in a
%                 vector [ii jj]
%   nfree.........number of free nodes


% Find start/goal coordinates

map_RGB=imread(map_in);     % RGB: (N,M,3) matrix
map_HSV=rgb2hsv(map_RGB);   % HSV: (N,M,3) matrix

N=size(map_HSV,1);          % original number of pixels
M=size(map_HSV,2);         

H=map_HSV(:,:,1);           % extract hue values

[start_ii,start_jj]=find(H>0.2 & H<0.5);       % coordinates in original image (red)
[goal_ii,goal_jj]=find(H>0.9 | H<0.1 & H~=0);  % coordinates in original image (green)

map_RGB(start_ii,start_jj,:)=255;   % set to white (free) the pixels of the starting positions
map_RGB(goal_ii,goal_jj,:)=255;     % set to white (free) the pixels of the goal positions

start_ii=round(mean(start_ii)*n/N);  % rescale coordinates
start_jj=round(mean(start_jj)*m/M);
goal_ii=round(mean(goal_ii)*n/N);
goal_jj=round(mean(goal_jj)*m/M);

start_ii=min(max(start_ii,1),n);     % make sure index is within the borders
start_jj=min(max(start_jj,1),m);
goal_ii=min(max(goal_ii,1),n);
goal_jj=min(max(goal_jj,1),m);

start=[start_ii start_jj];
goal=[goal_ii goal_jj];

% Create nodes

map_resized=imresize(map_RGB,[n m],'Method','nearest');     % resize to (n,m,3)
map_BW=im2gray(map_resized);                                % convert in gray scale: (n,m) matrix
map_bin=imbinarize(map_BW,BW_tresh);                        % logic: 1 if node (ii,jj) is free, 0 if obstacle; (n,m) matrix

% Assign outputs

map_out=map_bin;
nfree=sum(sum(map_bin));

end