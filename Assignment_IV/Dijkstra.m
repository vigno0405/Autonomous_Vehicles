function points = Dijkstra(BW, G, start_pos, goal_pos, num_image, diag)

% Plotting
plot_map(BW)
hold on;
plot(goal_pos(1), goal_pos(2), 'ro', 'MarkerFaceColor', 'r');
plot(start_pos(1), start_pos(2), 'go',...
'MarkerFaceColor', 'g');
    
% Linearize start and goal position
start = (start_pos(2) - 1)*size(BW, 1) + start_pos(1);
goal = (goal_pos(2) - 1)*size(BW, 1) + goal_pos(1);

% Initialize variables
dist = inf*ones(size(BW, 1)*size(BW, 2), 1); % C(q)
prec = inf*ones(size(BW, 1)*size(BW, 2), 1); % parenting
nodelist = -1*ones(size(BW, 1)*size(BW, 2), 1); % Q (open queue to visit)

% Initialize starting node
dist(start) = 0;
act_node = start;
[~, con_nodes]=find(G(act_node,:) > 0); % defining connected nodes
nodelist(con_nodes,1) = 1; % add: to visit (all directly reachable)
nodelist(act_node,1) = 0; % (already) visited

while any(nodelist(:,1)==1) && act_node~=goal
    i_con=length(con_nodes);
    
    while i_con > 0
    [row_act, col_act] = ind2sub(size(BW), act_node); % actual
    [row_con, col_con] = ind2sub(size(BW), con_nodes(i_con)); % to visit
    
        % Diagonal movement
        if abs(row_act - row_con) == 1 && abs(col_act - col_con) == 1
            if dist(con_nodes(i_con)) > dist(act_node) + sqrt(2)
                dist(con_nodes(i_con)) = dist(act_node) + sqrt(2);
                prec(con_nodes(i_con)) = act_node;
            end
        % Othogonal movement (the other case)
        else
            if dist(con_nodes(i_con)) > dist(act_node) + 1
                dist(con_nodes(i_con)) = dist(act_node) + 1;
                prec(con_nodes(i_con)) = act_node;
            end
        end
        i_con = i_con - 1;
    end
    
    % Plot run-time (in the graph)
    plot_runtime(act_node,BW)
    
    % Evaluate new candidate node & new neighbours
    % The following node in the alive set is the one with the minimum
    % distance (Dijkstra says so)
    alive = find(nodelist==1); % alive points (to be visited)
    [~, act_index] = min(dist(nodelist==1)); % 
    act_node = alive(act_index);
    nodelist(act_node,1)=0; % visited
    
    [~, con_nodes] = find(G(act_node,:) > 0); % def connected vertex
    i_con = length(con_nodes);
    while i_con > 0
        if nodelist(con_nodes(i_con),1) ~= 0 % if not visited
            nodelist(con_nodes(i_con),1) = 1; % add to visit
        end
        i_con = i_con-1;
    end
end

% Shortest path
if dist(goal) < inf
    sol_id = goal;
    path = [];
    while(sol_id ~= start)
        path = [sol_id path];
        sol_id = prec(sol_id);        
    end
    path = [sol_id path];
    points = zeros(length(path), 2);

    % Plot shortest path
    for i = 1:length(path)
        dr_i = 0;
        dr_j = path(i);
        while size(BW,1)<dr_j
            dr_i = dr_i+1;
            dr_j = dr_j-size(BW,1);
        end
        dr_i = (dr_i)+1;
        plot(dr_j,dr_i,'or','MarkerFaceColor','r');
        points(i,:) = [dr_j, dr_i];
    end
else 
    disp('no solution found')
    points = [];
end

total_distance = dist(goal);
total_nodes_analyzed = sum(nodelist==0);

if diag
    title(['DNav: image ', num2str(num_image), ' (length: ',...
    num2str(total_distance), ', ', num2str(total_nodes_analyzed),'/',num2str(sum(sum(BW))),...
    ' nodes)'], 'Interpreter', 'latex');
else
    title(['DVanilla: image ', num2str(num_image), ' (length: ',...
    num2str(total_distance), ', ', num2str(total_nodes_analyzed),'/',num2str(sum(sum(BW))),...
    ' nodes)'], 'Interpreter', 'latex');
end
hold off;

end