function [BW,not_clearanced,nodes,edges]=ProcessMapGRAY(pgm_file,n,m)
%
% Elaborates the provided gray scale map.
%
%   [BW,not_clearanced,nodes,edges]=ProcessMapGRAY(pgm_file,n,m)
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
%   nodes............contains the list of free nodes; i-th contains the i-th 
%                    free node
%   edges............contains the edges, i.e. valid connections between nodes;
%                    each row is associated to a certain edge and contains the
%                    indices of the two connected nodes


elatime=0;
tic
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

% Create nodes

[ii,jj]=find(BW==1);
nodes=[ii,jj];

% Edges creation:
%    1. build a distance matrix containing the distances between all nodes
%    2. divide this matrix into blocks that fit in RAM memory
%    3. process one block at the time, with a for loop, saving edges; use
%       GPU for speeding up large matrices operations

block_size=5000;
edges=[];
tot_adv=length(1:block_size:length(nodes));

% nodes=gpuArray(nodes);
for ii_start=1:block_size:length(nodes)

    clc
    adv=(ii_start-1)/block_size/tot_adv*100;
    fprintf('Generating graph: %.1f %% \n',adv)

    ii_end=min(ii_start+block_size-1,length(nodes));
    for jj_start=ii_start:block_size:length(nodes)
        jj_end=min(jj_start+block_size-1,length(nodes));

        % Compute distances between nodes in these blocks and set to zero
        % for nodes that are distant (distance > sqrt(2))
        distances=pdist2(nodes(ii_start:ii_end,:),nodes(jj_start:jj_end,:));
        distances(distances > sqrt(2))=0;
        distances=sparse(distances);
        % distances=gather(distances);        % GPU

        % Find indices of connected nodes and add it to "edges"
        [row,col]=find(distances>0);
        row=row+ii_start-1;       % global indices
        col=col+jj_start-1;       % global indices
        if jj_start==ii_start
            edges=[edges; [row,col]];
        else
            edges=[edges; [row,col]; [col,row]];
        end
    end

end
% nodes=gather(nodes);
clc
fprintf('Generating graph: 100 %% \n\n')
clc

elatime=elatime+toc;
fprintf(['Grid generated in ',num2str(elatime),' seconds\n\n'])

end