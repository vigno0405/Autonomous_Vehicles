function [parent,cost]=Rewire(tree,parent,cost,x_new,step,n_iter,map,meas)
%
% Rewiring function for RRT* with gamma calculated based on Karaman and Frazzoli.
%
%   [parent,cost,updated_edges]=Rewire(tree,parent,cost,x_new,step,n_iter,map)
%
% Inputs:
%   tree............Nx2 matrix of all nodes in the tree
%   parent..........Nx1 vector of parent indices for each node
%   cost............Nx1 vector of cost-to-come for each node
%   x_new...........1x2 vector representing the newly added node
%   step............max step size
%   n_iter..........current number of iterations
%   map.............binary map where 1 represents free space and 0 represents obstacles
%   meas............measure of the free space (for ex. area, for a 2D map) - it can be  
%                   omitted, but it helps speeding up the code
% 
% Outputs:
%   parent..........updated parent vector after rewiring
%   cost............updated cost vector after rewiring


% Gamma based on Karaman and Frazzoli formula

d=2;                        % dimension of the space (2D in our case)
zeta_d=pi;                  % volume of a unit ball (in 2D in our case)
if nargin<8
    mu_free=sum(map(:));    % measure of the free space (area in our case)
else
    mu_free=meas;
end
gamma=2*sqrt(1+1/d)*sqrt(mu_free/zeta_d);  % original formula: gamma=2*[(1+1/d)^(1/d)]*(mu_free/zeta_d)^(1/d); in our case d=2

% Rewiring radius r(n) for the current iteration. It decreases as more 
% nodes are added (RRT* optimality criterion)

radius=max(gamma*(log(n_iter)/n_iter)^(1/d),step);

% Calculate distances and find neighbors within the radius

distances=vecnorm(tree-x_new,2,2);
neighbors=find(distances<=radius);

% Get the index of the newly added node

x_new_idx=length(tree);

% Connect x_new to the node that guarantees the minimum cost-to-come. The 
% algorithm checks all neighbors within the radius to find the node that 
% minimizes the cost to reach x_new. This guarantees the locally optimal 
% connection for x_new

xmin_idx=parent(x_new_idx);                        % initialise the index of the new parent of x_new as the current parent of x_new
cmin=cost(xmin_idx)+norm(tree(xmin_idx,:)-x_new);  % initialise the minimum cost as the current cost to x_new

for ii=1:length(neighbors)
    neighbor_idx=neighbors(ii);
    neighbor=tree(neighbor_idx,:);

    if EdgeValidation(map,neighbor,x_new)
        temp_cost=cost(neighbor_idx)+norm(neighbor-x_new);  % new tentative cost
        if temp_cost<cmin
            xmin_idx=neighbor_idx;
            cmin=temp_cost;
        end
    end

end

% Update the parent and cost of x_new

parent(x_new_idx)=xmin_idx;
cost(x_new_idx)=cmin;

% Rewire neighbors to minimize cost-to-come through x_new.
% For each neighbor within the radius, check if connecting to x_new
% reduces total cost-to-come. If this happens, old parent is changed
% to x_new (with new cost recomputed). Then the tree remains optimal
% at least around x_new.

for ii=1:length(neighbors)
    neighbor_idx=neighbors(ii);
    neighbor=tree(neighbor_idx,:);

    if EdgeValidation(map,x_new,neighbor)
        potential_cost=cost(x_new_idx)+norm(x_new-neighbor); % tentative cost
        if potential_cost<cost(neighbor_idx)
            parent(neighbor_idx)=x_new_idx;
            cost(neighbor_idx)=potential_cost;
        end
    end
    
end

end
