%% Simulation code to test the hybridization of the 2D and 3D strategies
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

%set k parameter
k = 0.6;

% number of configurations to test
N = 5;

% stats
statsTime = zeros(N, 1);
statsLpath = zeros(N, 1);
statsEff = zeros(N, 1);

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
        % 2D strategy
        q = qN;
        [deterred valid desired_edge efficient t_deter l_path] = simLoop2D(world, agents, targets, SimParam, q);
        x_2D(j) = target_x_ini(j);
        y_2D(j) = target_y_ini(j);
        strat = '2D';
    elseif edge == 'E'
        % 2D strategy
        q = qE;
        [deterred valid desired_edge efficient t_deter l_path] = simLoop2D(world, agents, targets, SimParam, q);
        x_2D(j) = target_x_ini(j);
        y_2D(j) = target_y_ini(j);
        strat = '2D';
    elseif edge == 'S'
        % 3D strategy
        [deterred desired_edge efficient t_deter l_path] = simLoop3D(world, agents, targets, SimParam, k);
        x_3D(j) = target_x_ini(j);
        y_3D(j) = target_y_ini(j);
        strat = '3D';
    elseif edge == 'W'
        % 3D strategy
        [deterred desired_edge efficient t_deter l_path] = simLoop3D(world, agents, targets, SimParam, k);
        x_3D(j) = target_x_ini(j);
        y_3D(j) = target_y_ini(j);
        strat = '3D';
    end
    statsTime(j)     = t_deter;
    statsLpath(j)    = l_path;
    statsEff(j)      = efficient;
    j
end

%% calculate stats
mean_eff = mean(statsEff, 'omitnan')*100;
mean_t = mean(statsTime, 'omitnan');
med_t = median(statsTime, 'omitnan');
mean_l = mean(statsLpath, 'omitnan');
med_l = median(statsLpath, 'omitnan');

Av_Efficiency = [mean_eff];
Av_TimeOfDeterrence = [mean_t];
Med_TimeOfDeterrence = [med_t];
Av_LengthTraveled = [mean_l];
Med_LengthTraveled = [med_l];

stats_table = table( Av_Efficiency, Av_TimeOfDeterrence, Med_TimeOfDeterrence, Av_LengthTraveled, Med_LengthTraveled)

% save data to excel
% writetable(stats_table,'combination2d3d.xlsx')

% plotting tested points
hold on;
axis equal
drawRectangleColor(world.world_centre, world.size_x-400, world.size_y-400);
% Red dots for 2D strategy
plot(x_2D, y_2D, "o", 'color', 'r', MarkerSize=3)
% Blue dots for 3D strategy
plot(x_3D, y_3D, "o", 'color', 'b', MarkerSize=3)
xlim([200, 1200])
ylim([200 1000])
title('Initial positions of target')

% Elapsed time
toc