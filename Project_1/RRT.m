function [niter,nnodes,path_length,elatime]=RRT(map,start,goal,max_iter,step,bias,dyn_plot,functioning)
%
% Basic RRT algorithm.
%
%   [niter,nnodes,path_length,elatime]=RRT(map,start,goal,max_iter,step,bias,dyn_plot,functioning)
%
% Inputs:
%   map...........binary matrix: nxm binary matrix with 1
%                 (free nodes) and 0 (not free nodes)
%   start,goal....positions of start and final point: (ii,jj) indices in a
%                 vector [ii jj]
%   max_iter......max number of iterations
%   step..........max length of each edge of the tree
%   bias..........enforcing the probability of extending the trees straight
%                 towards the goal - between 0 and 1
%   dyn_plot......visualise growth of the tree
%   functioning...visualise the functioning principle of the algorithm -
%                 requires dyn_plot to be active
%
% Outputs:
%   niter.........number of iterations
%   nnodes........number of nodes
%   path_length...length of the solution path
%   elatime.......running time of the algorothm


% Open figure

figure
imshow(map)
hold on
plot(start(2),start(1),'go','MarkerFaceColor','g')
plot(goal(2),goal(1),'ro','MarkerFaceColor','r')
title('RRT algorithm')

elatime=0;
tic

% Tree initialization

tree=start;   % will contain (as rows) the points [ii jj] of the tree
parent=0;     % k-th element is the index (inside "tree") of the parent node of the k-th point of "tree"

% Iterations

rng('shuffle')
[n,m]=size(map);

for niter=1:max_iter

    not_valid=false;

    % 1. Generate a random point x_rand. "goal" is taken every once in a
    % while, according to the bias

    if rand()<bias
        x_rand=goal;
    else
        x_rand=[randi(n),randi(m)];
    end
    
    % 2. Find the closest point of the tree to x_rand

    distances=vecnorm(tree-x_rand,2,2);  % euclidean distances
    [~,ind_near]=min(distances);
    x_near=tree(ind_near,:);

    if (length(tree)>50 && length(tree)<54) && dyn_plot==true && functioning==true
        elatime=elatime+toc;
        pause(1)
        h_rand=plot([x_rand(2) x_near(2)],[x_rand(1) x_near(1)],'ro--');
        tic
    end
    
    % 3. Extend the tree by choosing a new node x_new in the direction of
    % x_rand and at a maximum distance "step"

    if norm(x_rand-x_near)<step                   % if x_rand is already within "step", keep it as x_new
        x_new=x_rand;
    else
        dir=(x_rand-x_near)/norm(x_rand-x_near);  % direction (as a versor)
        x_new=x_near+step*dir;
        x_new=round(x_new);
    end

    if (length(tree)>50 && length(tree)<54) && dyn_plot==true && functioning==true
        elatime=elatime+toc;
        pause(1)
        h_new=plot(x_new(2),x_new(1),'rx');
        pause(1)
        delete(h_rand)
        tic
    end

    % 4. Validation of the new point (i.e. if it belongs to C_free: inside
    % the map and not on obstacles) and of the new edge (i.e. if it does
    % not cross any obstacle)

    if x_new(1)>0 && x_new(2)>0 && x_new(1)<=n && x_new(2)<=m && map(x_new(1),x_new(2))==1 && EdgeValidation(map,x_new,x_near)
       
        % Add x_new to the tree and update parent vector
        tree=[tree; x_new];
        parent=[parent; ind_near];
        
        % Update the tree plot
        if dyn_plot==true
            elatime=elatime+toc;
            plot([x_near(2),x_new(2)],[x_near(1),x_new(1)],'b.-')
            tic
        end
        
        % Stop if the node is close enough to the goal and their connection is valid
        if norm(x_new-goal)<step && EdgeValidation(map,x_new,goal)
            tree=[tree; goal];
            parent=[parent; length(tree)-1];
            break
        end

    else

        not_valid=true;

    end
    
    if dyn_plot==true
        elatime=elatime+toc;
        drawnow
        pause(0.001)
        if ((length(tree)>51 && length(tree)<55) || ((length(tree)>50 && length(tree)<54) && not_valid==true)) && functioning==true
            delete(h_new)
        end
        tic
    end

end

% Reconstruct final path, if one was found within max_iter, and assign
% nnodes

nnodes=length(tree);

if tree(end,:)~=goal
   
    fprintf('RRT failed to find a path within the maximum iterations.\n\n');
    path_length=0;
    elatime=elatime+toc;
    if dyn_plot==false
        for ii=2:length(parent)
            par=parent(ii);
            plot([tree(par,2) tree(ii,2)],[tree(par,1) tree(ii,1)],'b.-')
        end
    end
    hold off

else

    if dyn_plot==false
        elatime=elatime+toc;
        for ii=2:length(parent)
            par=parent(ii);
            plot([tree(par,2) tree(ii,2)],[tree(par,1) tree(ii,1)],'b.-')
        end
        tic
    end
    idx=length(tree);
    path_length=0;
    while idx>1
        prev_node=tree(parent(idx),:);
        path_length=path_length+norm(tree(idx,:)-prev_node);
        
        elatime=elatime+toc;
        plot([tree(idx,2),prev_node(2)],[tree(idx,1),prev_node(1)],'ro-','LineWidth',1.5,'MarkerSize',2,'MarkerFaceColor','r')
        tic

        idx=parent(idx);
    end
    elatime=elatime+toc;
    hold off
    
end

end