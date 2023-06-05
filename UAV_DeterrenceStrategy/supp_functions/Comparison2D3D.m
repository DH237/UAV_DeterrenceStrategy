%% Simulation code used to compare the 2D and 3D strategies

clc; clear; close all;
tic

%% Set up the simulation world
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

% edge tested
testing_edge = 'W';

if testing_edge == 'N'
    q = qN;
elseif testing_edge == 'E'
    q = qE;
elseif testing_edge == 'S'
    q = qS;
elseif testing_edge == 'W'
    q = qW;
end

% stats
statsTime2D = zeros(N, 1);
statsLpath2D = zeros(N, 1);
statsEff2D = zeros(N, 1);

statsTime3D = zeros(N, 1);
statsLpath3D = zeros(N, 1);
statsEff3D = zeros(N, 1);

% random locations for target
% target_x_ini = randi([210 1190], 1, N);
% target_y_ini = randi([210 990], 1, N);

for i = 1:N
    [x y] = randEdge(world, testing_edge);
    target_x_ini(i) = x;
    target_y_ini(i) = y;
end

% loop to test on random initial positions of target
for j = 1:N
    target = Target(false, target_x_ini(j), target_y_ini(j),  20, deg2rad(270), 5,  350,  12,      deg2rad(45));
    targets             = [target];
    targets(1)      = expandArrays(targets(1), world, iterations);
    
    % 2 dimensions strategy
    [deterred valid desired_edge efficient t_deter l_path] = simLoop2D(world, agents, targets, SimParam, q);
    statsTime2D(j)     = t_deter;
    statsLpath2D(j)    = l_path;
    statsEff2D(j)      = efficient;
    
    % 3 dimensions strategy
    [deterred desired_edge efficient t_deter l_path] = simLoop3D(world, agents, targets, SimParam, k);
    statsTime3D(j)     = t_deter;
    statsLpath3D(j)    = l_path;
    statsEff3D(j)      = efficient;
    j
end

%% 2 dimension stats
mean_eff2D = mean(statsEff2D, 'omitnan')*100;
mean_t2D = mean(statsTime2D, 'omitnan');
med_t2D = median(statsTime2D, 'omitnan');
mean_l2D = mean(statsLpath2D, 'omitnan');
med_l2D = median(statsLpath2D, 'omitnan');

%% 3 dimension stats
mean_eff3D = mean(statsEff3D, 'omitnan')*100;
mean_t3D = mean(statsTime3D, 'omitnan');
med_t3D = median(statsTime3D, 'omitnan');
mean_l3D = mean(statsLpath3D, 'omitnan');
med_l3D = median(statsLpath3D, 'omitnan');

Strategy = ['2D'; '3D'];
Av_Efficiency = [mean_eff2D; mean_eff3D];
Av_TimeOfDeterrence = [mean_t2D; mean_t3D];
Med_TimeOfDeterrence = [med_t2D; med_t3D];
Av_LengthTraveled = [mean_l2D; mean_l3D];
Med_LengthTraveled = [med_l2D; med_l3D];

stats_table = table(Strategy, Av_Efficiency, Av_TimeOfDeterrence, Med_TimeOfDeterrence, Av_LengthTraveled, Med_LengthTraveled)

% save data in excel
% writetable(stats_table,'comparison2d3d.xlsx')

% plotting tested points
hold on;
axis equal
drawRectangleColor(world.world_centre, world.size_x-400, world.size_y-400);
plot(target_x_ini,target_y_ini, "o", 'color', 'r', MarkerSize=3)
xlim([200, 1200])
ylim([200 1000])
title('Initial positions of target')

% Elapsed time
toc