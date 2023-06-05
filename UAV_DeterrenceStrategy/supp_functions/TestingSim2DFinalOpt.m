%% Simulation code to test the optimized 2D strategy over the whole map
clc; clear; close all;
tic

%% NO PLOTTING VERSION

%% Set up the simulation world (circle is not working)
%                           shape         x    y     grid size
world               = World('rectangle', [1000+400,800+400], 10);

%% Set up agent
%                              typ x     y    z   ro pc yaw           ve  max_ve  min_ve max_yawrate
agent              = Aircraft(2, 200,  200, 50, 0, 0, deg2rad(45), 0,  10,     0,     deg2rad(45));
agents              = [agent];


%% Set up targets
% initialise target
%                            fligt x    y     z   heading       vel snse  max_vel  yaw_rate
target             = Target(false, 350, 600,  20, deg2rad(270), 5,  800,  12,      deg2rad(45));
targets             = [target];

% initialise targets with initial interests
for k = 1:size(targets,1)
    targets(k)      = initInterests(targets(k), world);
end

%% simulation parameters
t_init              = 0;
t                   = t_init; % seconds
dt                  = 1; % seconds
t_total             = 150;
cumulation          = 1;
iterations          = ceil(t_total/dt)+1;
SimParam = [t_init t dt t_total cumulation iterations];

% expand matrices to speed up code
world               = expandArrays(world, iterations);
for i = 1:size(agents,1)
    agents(i)       = expandArrays(agents(i), world, iterations);
end
for k = 1:size(targets,1)
    targets(k)      = expandArrays(targets(k), world, iterations);
end

% set q parameter
qN = 1.1;
qE = 2.6;
qS = 2.8;
qW = 3.8;

% number of configurations to test
N = 5;



% define results table
% varTypes2D = ["double", "double", "logical", "logical", "logical", "logical", "double", "double", "double"];
% varNames2D = ["InitialPos_X", "InitialPos_Y", "Deterred", "ValidTrajectory", "DesiredEdge", "Efficient", "q", "TimeOfDeterrence", "LengthTraveled"];
% results2D = table('Size', [length(Q) 9],'VariableTypes',varTypes2D, 'VariableNames',varNames2D);

% stats
% statsTime2D = zeros(N, 1);
% statsLpath2D = zeros(N, 1);
% statsEff2D = zeros(N, 1);

% random locations for target
target_x_ini = randi([210 1190], 1, N);
target_y_ini = randi([210 990], 1, N);



% loop to test on random initial positions of target
for j = 1:N
    target = Target(false, target_x_ini(j), target_y_ini(j),  20, deg2rad(270), 5,  350,  12,      deg2rad(45));
    targets             = [target];
    targets(1)      = expandArrays(targets(1), world, iterations);
    edge = closestEdge(world, targets(1), cumulation+1);
    if edge == 'N'
        q = qN;
    elseif edge == 'E'
        q = qE;
    elseif edge == 'S'
        q = qS;
    elseif edge == 'W'
        q = qW;
    end
    % 2 dimensions strategy
    [deterred valid desired_edge efficient t_deter l_path] = simLoop2D(world, agents, targets, SimParam, q);
%     results2D(i, :)          = {target_x_ini(j), target_y_ini(j), deterred, valid, desired_edge, efficient, q, t_deter, l_path};
    statsTime2D(j)     = t_deter;
    statsLpath2D(j)    = l_path;
    statsEff2D(j)      = efficient;
    j
end


mean_eff = mean(statsEff2D, 'omitnan')*100;
mean_t = mean(statsTime2D, 'omitnan');
med_t = median(statsTime2D, 'omitnan');
mean_l = mean(statsLpath2D, 'omitnan');
med_l = median(statsLpath2D, 'omitnan');


% results2D;

varTypes2D = ["double", "double", "double", "double", "double"];
varNames2D = ["Av_Efficiency", "Av_TimeOfDeterrence", "Med_TimeOfDeterrence", "Av_LengthTraveled", "Med_LengthTraveled"];
stats_table2D = table('Size', [1, 5],'VariableTypes',varTypes2D, 'VariableNames',varNames2D);

stats_table2D(1, :) = {mean_eff, mean_t, med_t, mean_l, med_l};
stats_table2D


% Plotting
% figure(1)
% [t,s] = title(tcl,['2D strategy'], [num2str(N),' iterations']);
% t.FontSize = 14;
% t.FontWeight = 'bold';
% s.FontAngle = 'italic';
% % efficiency 
% nexttile
% bar(Q, mean_eff)
% title('Average Efficiency')
% xticks(Q)
% ylim([0 100])
% % time
% nexttile
% bar(Q, mean_t)
% title('Average deterrence time (normalized)')
% xticks(Q)
% ylim([0.5 1])
% % length 
% nexttile
% bar(Q, mean_l)
% title('Average length traveled (normalized)')
% xticks(Q)
% xlabel('q') 
% ylim([0.5 1])

% plotting tested points
% figure(2)
hold on;
% drawBoundary;
axis equal
drawRectangleColor(world.world_centre, world.size_x-400, world.size_y-400);
plot(target_x_ini,target_y_ini, "o", 'color', 'r', MarkerSize=3)
xlim([200, 1200])
ylim([200 1000])
title('Initial positions of target')

% results2D

% Elapsed time
toc