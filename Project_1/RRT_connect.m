function [niter,nnodes,path_length,elatime]=RRT_connect(map,start,goal,max_iter,step,bias,greedy,dyn_plot)
%
% RRT-connect algorithm. Generates two trees from start and goal positions
% and extends them untill they meet. "Greedy" heuristic behaviour for
% connecting the trees can be enabled.
%
%   [niter,nnodes,path_length,elatime]=RRT_connect(map,start,goal,max_iter,step,bias,greedy,dyn_plot)
%
% Inputs:
%   map...........binary matrix: nxm binary matrix with 1
%                 (free nodes) and 0 (not free nodes)
%   start,goal....positions of start and final point: (ii,jj) indices in a
%                 vector [ii jj]
%   max_iter......max number of iterations
%   step..........max length of each edge of the tree
%   bias..........enforcing the probability of extending the trees straight
%                 towards the goal/start - between 0 and 1
%   greedy........enables "greedy" behaviour: when extending the tree, it
%                 tries to create a longer edge (up to a maximum distance of
%                 "greedy*step", or when it meets an obstacle or the randomly
%                 sampled point) - set "greedy=0" for no greedy behaviour
%   dyn_plot......visualise growth of the tree
%
% Outputs:
%   niter.........number of iterations
%   nnodes........number of nodes
%   path_length...length of the solution path
%   elatime.......running time of the algorithm


% Open figure

figure
imshow(map)
hold on
plot(start(2),start(1),'go','MarkerFaceColor','g')
plot(goal(2),goal(1),'ro','MarkerFaceColor','r')
title('RRT-connect algorithm')

elatime=0;
tic

% Trees initialisation

tree_start=start; % forward tree (from start)
parent_start=0;   % parent indices for start tree

tree_goal=goal;   % backward tree (from goal)
parent_goal=0;    % parent indices for goal tree

trees={tree_start,tree_goal};
parents={parent_start,parent_goal};

% Iterations

rng('shuffle')
[n,m]=size(map);
connect_dist=-1;

for niter=1:max_iter

    % 0. Select the tree for the niter-th iteration

    k=2-mod(niter,2);    % k=1 associated to tree start, k=2 to tree goal

    % 1. Generate random point x_rand. "goal" or "start" are taken every
    % once in a while, according to the bias

    if k==1 && rand()<bias
        x_rand=goal;
    elseif  k==2 && rand()<bias
        x_rand=start;
    else
        x_rand=[randi(n),randi(m)];
    end

    % 2. Find the closest point of the current tree to x_rand

    distances=vecnorm(trees{k}-x_rand,2,2);   % euclidean distances
    [~,ind_near]=min(distances);
    x_near=trees{k}(ind_near,:);    

    % 3. Extend the tree by choosing a new node x_new in the direction of
    % x_rand and: at a maximum distance "step", if no greedy beahaviour; at
    % a maximum distance of "greedy*step" or up to an obstacle, otherwise

    if norm(x_rand-x_near)<step || (greedy>0 && norm(x_rand-x_near)<greedy*step)
        x_new=x_rand;
    elseif greedy>0
        dir=(x_rand-x_near)/norm(x_rand-x_near);  % direction (as a versor)
        x_ii=x_near;
        while x_ii(1)>0 && x_ii(1)<n && x_ii(2)>0 && x_ii(2)<m && map(x_ii(1),x_ii(2))==1 && norm(x_ii-x_near)<greedy*step
            x_new=x_ii;
            x_ii=x_new+step*dir;
            x_ii=round(x_ii);
        end
    else
        dir=(x_rand-x_near)/norm(x_rand-x_near);  % direction (as a versor)
        x_new=x_near+dir*step;
    end
    x_new=round(x_new);

    % 4. Validation of the new point (i.e. if it belongs to C_free: inside
    % the map and not on obstacles) and of the new edge (i.e. if it does
    % not cross obstacles)

    if x_new(1)>0 && x_new(2)>0 && x_new(1)<=n && x_new(2)<=m && map(x_new(1),x_new(2))==1 && EdgeValidation(map,x_new,x_near)
       
        % Add x_new to the current tree and update parent
        trees{k}=[trees{k}; x_new];
        parents{k}=[parents{k}; ind_near];
        
        % Update the tree plot
        if dyn_plot==true
            elatime=elatime+toc;
            if k==1
                col='b';
            else
                col='c';
            end
            plot([x_near(2),x_new(2)],[x_near(1),x_new(1)],'Color',col,'Marker','.')
            tic
        end

        % Try to connect x_new to the closest point of the OTHER tree. If
        % "greedy" heuristic behaviour is enabled, take "greedy*step" as the
        % maximum distance for connecting the trees
        distances=vecnorm(trees{3-k}-x_new,2,2);        % euclidean distances
        [~,ind_near]=min(distances);
        x_near=trees{3-k}(ind_near,:);
        if EdgeValidation(map,x_new,x_near) 
            if min(distances)<=step || (greedy>0 && min(distances)<=greedy*step)
                elatime=elatime+toc;
                plot([x_near(2) x_new(2)],[x_near(1) x_new(1)],'r.-','LineWidth',1.5)
                tic
                connect_dist=norm(x_near-x_new);
                break
            end
        end

    end

    if dyn_plot==true
        elatime=elatime+toc;
        drawnow
        pause(0.001)
        tic
    end

end

% Reconstruct final path, if one was found within max_iter, and assign
% nnodes

uniqueA=unique(trees{1},'rows');
uniqueB=unique(trees{2},'rows');
nnodes=length(uniqueA)+length(uniqueB);

if connect_dist==-1
   
    fprintf('RRT_connect failed to find a path within the maximum iterations.\n\n');
    path_length=0;
    elatime=elatime+toc;
    if dyn_plot==false
        for ii=2:length(parents{1})
            par=parents{1}(ii);
            plot([trees{1}(par,2) trees{1}(ii,2)],[trees{1}(par,1) trees{1}(ii,1)],'b.-')
        end
        for ii=2:length(parents{2})
            par=parents{2}(ii);
            plot([trees{2}(par,2) trees{2}(ii,2)],[trees{2}(par,1) trees{2}(ii,1)],'c.-')
        end
    end
    hold off

else

    if dyn_plot==false
        elatime=elatime+toc;
        for ii=2:length(parents{1})
            par=parents{1}(ii);
            plot([trees{1}(par,2) trees{1}(ii,2)],[trees{1}(par,1) trees{1}(ii,1)],'b.-')
        end
        for ii=2:length(parents{2})
            par=parents{2}(ii);
            plot([trees{2}(par,2) trees{2}(ii,2)],[trees{2}(par,1) trees{2}(ii,1)],'c.-')
        end
        tic
    end
    
    % First, compute the path along tree "k": it was the selected tree when
    % the connection was established, so its last point is for sure our 
    % starting point to start reconstruction the backwards path in tree "k"
    idx=length(trees{k});
    path_length=0;
    while idx>1
        prev_node=trees{k}(parents{k}(idx),:);
        path_length=path_length+norm(trees{k}(idx,:)-prev_node);
    
        elatime=elatime+toc;
        plot([trees{k}(idx,2),prev_node(2)],[trees{k}(idx,1),prev_node(1)],'ro-','MarkerSize',2,'LineWidth',1.5)
        tic

        idx=parents{k}(idx);
    end

    % Then, compute the same on the other tree ("3-k"): in this case, the
    % starting point for the backwards reconstruction is "x_near", that
    % needs to be located inside the tree
    idx=find(all(trees{3-k}==x_near,2));   % finds the index of x_near inside the tree
    idx=idx(1);
    while idx>1
        prev_node=trees{3-k}(parents{3-k}(idx),:);
        path_length=path_length+norm(trees{3-k}(idx,:)-prev_node);
    
        elatime=elatime+toc;
        plot([trees{3-k}(idx,2),prev_node(2)],[trees{3-k}(idx,1),prev_node(1)],'ro-','MarkerSize',2,'LineWidth',1.5)
        tic

        idx=parents{3-k}(idx);
    end

    % Finally, add the connecting edge to path_length
    path_length=path_length+connect_dist;

    elatime=elatime+toc;
    hold off

end

end