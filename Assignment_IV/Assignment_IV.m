
clear all;
close all;
clc;
cd('/home/vigno/ws_new/Assignment_IV');

%% PART 1: first image

clear all;
close all;
clc;

[BW1, G1, start_pos1, goal_pos1] = process_map('map_1_d.png', 30, 30, false);
points_dijkstra1_nodiag = Dijkstra(BW1, G1, start_pos1, goal_pos1, 1, false);
points_astar1_nodiag = Astar(BW1, G1, start_pos1, goal_pos1, 1, false);

[BW1, G1, start_pos1, goal_pos1] = process_map('map_1_d.png', 30, 30, true);
points_dijkstra1_diag = Dijkstra(BW1, G1, start_pos1, goal_pos1, 1, true);
tic
points_astar1_diag = Astar(BW1, G1, start_pos1, goal_pos1, 1, true);
toc

%% PART 2: second image

clear all;
close all;
clc;

[BW2, G2, start_pos2, goal_pos2] = process_map('map_2_d.png', 30, 30, false);
points_dijkstra2_nodiag = Dijkstra(BW2, G2, start_pos2, goal_pos2, 2, false);
points_astar2_nodiag = Astar(BW2, G2, start_pos2, goal_pos2, 2, false);

[BW2, G2, start_pos2, goal_pos2] = process_map('map_2_d.png', 30, 30, true);
points_dijkstra2_diag = Dijkstra(BW2, G2, start_pos2, goal_pos2, 2, true);
tic
points_astar2_diag = Astar(BW2, G2, start_pos2, goal_pos2, 2, true);
toc

%% PART 3: third image

clear all;
close all;
clc;

[BW3, G3, start_pos3, goal_pos3] = process_map('map_3_d.png', 30, 30, false);
points_dijkstra3_nodiag= Dijkstra(BW3, G3, start_pos3, goal_pos3, 3, false);
points_astar3_nodiag = Astar(BW3, G3, start_pos3, goal_pos3, 3, false);

[BW3, G3, start_pos3, goal_pos3] = process_map('map_3_d.png', 30, 30, true);
points_dijkstra3_diag = Dijkstra(BW3, G3, start_pos3, goal_pos3, 3, true);
tic
points_astar3_diag = Astar(BW3, G3, start_pos3, goal_pos3, 3, true);
toc

%% PART 4: fourth image

clear all;
close all;
clc;

[BW4, G4, start_pos4, goal_pos4] = process_map('map_4_d.png', 30, 30, false);
points_dijkstra4_nodiag = Dijkstra(BW4, G4, start_pos4, goal_pos4, 4, false);
points_astar4_nodiag = Astar(BW4, G4, start_pos4, goal_pos4, 4, false);

[BW4, G4, start_pos4, goal_pos4] = process_map('map_4_d.png', 30, 30, true);
points_dijkstra4_diag = Dijkstra(BW4, G4, start_pos4, goal_pos4, 4, true);
tic
points_astar4_diag = Astar(BW4, G4, start_pos4, goal_pos4, 4, true);
toc



