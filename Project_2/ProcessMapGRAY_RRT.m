function [BW,not_clearanced]=ProcessMapGRAY_RRT(pgm_file,n,m)
%
% Elaborates the provided gray scale map.
%
% Inputs:
%   pgm_file.........provided map: NxM gray scale image with gray and black 
%                    (obstacles), white (free)
%   n,m..............new dimensions of the map
%
% Outputs:
%   BW...............binary output rescaled matrix: nxm binary matrix with 1
%                    (free nodes) and 0 (not free nodes)
%   not_clearanced...BW map before adding clearance around obstacles


% Resize given map and assign free/occupied nodes

map_pgm=imread(pgm_file);   % gray scale image: (N,M) matrix

BW_tresh=0.99;
map_resized=imresize(map_pgm,[n m],'Method','nearest');     % resize to (n,m,3), with n=m=30
map_BW=im2gray(map_resized);                                % convert in gray scale: (n,m) matrix
BW=imbinarize(map_BW,BW_tresh);                             % logic: 1 if node (ii,jj) is free, 0 if obstacle; (n,m) matrix
not_clearanced=BW;

% Add clearance

se=strel('disk',round(6/384*max(n,m)));  % "radius" of the turtlebot is more or less 3 pixels on 384 (or 6 on 768), so set a clearance of 6/384
BW=imdilate(~BW,se);
BW=~BW;

end