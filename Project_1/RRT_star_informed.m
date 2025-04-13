function [niter,nnodes,path_length,elatime]=RRT_star_informed(map,start,goal,max_iter,step,bias,PATIENCE,ellipse,dyn_plot)
%
% RRT* Algorithm. Informed behaviour can be enabled.
%
%   [niter,nnodes,path_length,elatime]=RRT_star_informed(map,start,goal,max_iter,step,bias,PATIENCE,ellipse,dyn_plot)
%
% Inputs:
%   map...........binary output rescaled matrix: nxm binary matrix with 1
%                 (free nodes) and 0 (obstacle nodes)
%   start,goal....positions of start and final point: (ii,jj) indices in a
%                 vector [ii jj]
%   max_iter......max number of iterations
%   step..........max length of each edge of the tree
%   bias..........enforcing the probability of extending the trees straight
%                 towards the goal - between 0 and 1
%   PATIENCE......to manage stagnation
%   ellipse.......enables informed behaviour
%   dyn_plot......visualise growth of the tree
%
% Outputs:
%   niter.........number of iterations
%   nnodes........number of nodes (in the final tree)
%   path_length...length of the optimal solution path
%   elatime.......running time of the algorithm


% Open figure

figure
imshow(map)
hold on
plot(start(2),start(1),'go','MarkerFaceColor','g')
plot(goal(2),goal(1),'ro','MarkerFaceColor','r')
tree_plot=plot(nan,nan,'c.-');
sol_plot=plot(nan,nan,'r.-','LineWidth',2);
if ellipse==true
    ell_plot=plot(nan,nan,'--','Color',[0 0 1],'LineWidth',2);
end
title('RRT* algorithm')

elatime=0;
tic

% Initialization

tree=start;          % initialise the tree
parent=0;            % initialise parent nodes
cost=0;              % initialise cost to reach each node from start
cost_best=inf;       % cost of the best solution path
flag=false;          % flag on finding the first solution path
stagnation_count=0;  % stagnation counter
area=sum(map(:));    % measure of the free space in our 2D map

rng('shuffle')
[n,m]=size(map);

for niter = 1:max_iter

    % 1. Generate a random point x_rand. "goal" is taken every once in a
    % while, according to the bias. Sample x_rand in the entire space if we
    % still need to find the first solution or if informed behaviour is
    % disabled; sample x_rand inside the "informed ellipse" otherwise
    
    if cost_best==inf || ellipse==false
        if rand()<bias
            x_rand=goal;
        else
            x_rand=[randi(n),randi(m)];
        end
    else                                                  
        x_rand=InformedSample(start,goal,cost_best,n,m);
    end

    % 2. Find the closest point of the tree to x_rand

    distances=vecnorm(tree-x_rand,2,2);  % euclidean distances
    [~,ind_near]=min(distances);
    x_near=tree(ind_near,:);

    % 3. Extend the tree by choosing a new node x_new in the direction of
    % x_rand and at a maximum distance "step"
    
    if norm(x_rand-x_near)<step                   % if x_samp is already within "step", keep it as x_new
        x_new=x_rand;
    else
        dir=(x_rand-x_near)/norm(x_rand-x_near);  % direction (as a versor)
        x_new=x_near+step*dir;
        x_new=round(x_new);
    end

    % 4. Validation of the new point (i.e. if it belongs to C_free: inside
    % the map and not on obstacles) and of the new edge (i.e. if it does
    % not cross any obstacle)

    if x_new(1)>0 && x_new(2)>0 && x_new(1)<=n && x_new(2)<=m && map(x_new(1),x_new(2))==1 && EdgeValidation(map,x_new,x_near)

        % Add x_new to the tree and update parent and cost
        tree=[tree; x_new];
        parent=[parent; ind_near];
        cost=[cost; cost(ind_near)+norm(x_new-x_near)];

        % Plot tree before rewiring
        if dyn_plot==true
            elatime=elatime+toc;
            x_coords=nan(1,3*(length(parent)-1));
            y_coords=nan(1,3*(length(parent)-1));
            for ii=2:length(parent)
                par=parent(ii);
                x_coords(3*(ii-2)+1:3*(ii-2)+3)=[tree(par,2) tree(ii,2) nan];   % nan to interrupt the line
                y_coords(3*(ii-2)+1:3*(ii-2)+3)=[tree(par,1) tree(ii,1) nan];   % parent, child from the tree, and nan
            end
            set(tree_plot,'XData',x_coords,'YData',y_coords);                   % before drawing
            drawnow
            pause(0.001)
            tic
        end

        % Rewire around x_new to optimize costs        
        [parent,cost]=Rewire(tree,parent,cost,x_new,step,niter,map,area);

        % Plot new "rewired" tree
        if dyn_plot==true
            elatime=elatime+toc;
            x_coords=nan(1,3*(length(parent)-1));
            y_coords=nan(1,3*(length(parent)-1));
            for ii=2:length(parent)
                par=parent(ii);
                x_coords(3*(ii-2)+1:3*(ii-2)+3)=[tree(par,2) tree(ii,2) nan];
                y_coords(3*(ii-2)+1:3*(ii-2)+3)=[tree(par,1) tree(ii,1) nan];
            end
            set(tree_plot,'XData',x_coords,'YData',y_coords);
            drawnow
            pause(0.001)
            tic
        end

        % Check if the goal is reached (this block is run only one time,
        % when we connect the goal)
        if norm(x_new-goal)<step && flag==false && EdgeValidation(map,x_new,goal)

            flag=true;
            tree=[tree; goal];
            parent=[parent; length(tree)-1];
            cost=[cost; cost(end)+norm(x_new-goal)];
            cost_best=cost(end);
            initial_path_length=cost_best;
            idx=length(tree);
            id_goal=idx;

            if dyn_plot==true
                elatime=elatime+toc;
                plot([x_new(2),goal(2)],[x_new(1),goal(1)],'c')
                x_coords=[];
                y_coords=[];
                while idx>1            % plot first best solution
                    prev_node=tree(parent(idx),:);
                    x_coords=[x_coords tree(idx,2) prev_node(2) nan];
                    y_coords=[y_coords tree(idx,1) prev_node(1) nan];
                    idx=parent(idx);
                end
                set(sol_plot,'XData',x_coords,'YData',y_coords);

                drawnow
                tic
            end

        end
        
        % Compute updated goal cost (this block is run at every step, but 
        % only after finding a solution for the first time)
        if flag==true

            idx=id_goal;             % known from previous block
            cost_goal=0;
            while idx>1              % compute new cost of goal
                prev_node=tree(parent(idx),:);
                cost_goal=cost_goal+norm(tree(idx,:)-prev_node);
                idx=parent(idx);
            end

            if cost_goal<cost_best   % if it is lower than the previous one, change best solution...
                cost_best=cost_goal;
                stagnation_count=0;  % ...and reset the stagnation counter
                if dyn_plot==true    % plot updated best solution
                    elatime=elatime+toc; 
                    idx=id_goal;
                    x_coords=[];
                    y_coords=[];
                    while idx>1            
                        prev_node=tree(parent(idx),:);
                        x_coords=[x_coords tree(idx,2) prev_node(2) nan];
                        y_coords=[y_coords tree(idx,1) prev_node(1) nan];
                        idx=parent(idx);
                    end
                    set(sol_plot,'XData',x_coords,'YData',y_coords);
                    if ellipse==true
                        delete(ell_plot)
                        ell_plot=PlotEllipse(start,goal,cost_best);
                    end
                    drawnow
                    tic
                end
            else
                stagnation_count=stagnation_count+1;
            end

        end

        % Stop if stagnation limit is reached
        if stagnation_count>=PATIENCE
            break
        end
    end

end

% Assign outputs

uniq=unique(tree,'rows');   % just to have nnodes
nnodes=length(uniq);

if cost_best<inf

    if dyn_plot==false   % plot tree
        elatime=elatime+toc;
        for ii=2:length(parent)
            par=parent(ii);
            plot([tree(par,2) tree(ii,2)],[tree(par,1) tree(ii,1)],'c.-')
        end
        drawnow
        tic
    end

    idx=id_goal;
    path_length=0;
    while idx>1
        prev_node=tree(parent(idx),:);
        path_length=path_length+norm(tree(idx,:)-prev_node);
        if dyn_plot==false  % plot solution
            elatime=elatime+toc;
            plot([tree(idx,2),prev_node(2)],[tree(idx,1),prev_node(1)],'ro-','MarkerSize',2,'LineWidth',2,'MarkerFaceColor','r');
            tic
        end
        idx=parent(idx);
    end

    elatime=elatime+toc;
    title(['RRT*: path_{best}=', num2str(path_length), ' path_{best,not informed}=', num2str(initial_path_length)])

else

    fprintf('RRT* failed to find a path within the maximum iterations.\n\n');
    path_length=0;
    elatime=elatime+toc;
    if dyn_plot==false
        for ii=2:length(parent)
            par=parent(ii);
            plot([tree(par,2) tree(ii,2)],[tree(par,1) tree(ii,1)],'c.-')
        end
    end

end

hold off

end