%% Simulation code to test the 2D strategy
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
Q = [1.5, 2, 3, 4, 5];
% Q = [1.1, 1.3, 1.5, 1.7, 1.9];
% Q = [2.6, 2.8, 3.0, 3.2, 3.4];
Q = [3.6, 3.8, 4.0, 4.2, 4.4];

% number of configurations to test
N = 5;

% edge tested
testing_edge = 'W';

% define results table
varTypes2D = ["double", "double", "logical", "logical", "logical", "logical", "double", "double", "double"];
varNames2D = ["InitialPos_X", "InitialPos_Y", "Deterred", "ValidTrajectory", "DesiredEdge", "Efficient", "q", "TimeOfDeterrence", "LengthTraveled"];
results2D = table('Size', [length(Q) 9],'VariableTypes',varTypes2D, 'VariableNames',varNames2D);

% stats
statsTime2D = zeros(N, length(Q));
statsLpath2D = zeros(N, length(Q));
statsEff2D = zeros(N, length(Q));

% random locations for target
% target_x_ini = randi([200 1200], 1, N);
% target_y_ini = randi([200 1000], 1, N);

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
    for i = 1:length(Q)
        % 2 dimensions strategy
        [deterred valid desired_edge efficient t_deter l_path] = simLoop2D(world, agents, targets, SimParam, Q(i));
        results2D(i, :)          = {target_x_ini(j), target_y_ini(j), deterred, valid, desired_edge, efficient, Q(i), t_deter, l_path};
        statsTime2D(j, i)     = t_deter;
        statsLpath2D(j, i)    = l_path;
        statsEff2D(j, i)      = efficient;
    end
    % normalize length of path and time
    statsLpath2D(j, :) = statsLpath2D(j, :)/max(statsLpath2D(j, :));
    statsTime2D(j, :) = statsTime2D(j, :)/max(statsTime2D(j, :));
    results2D;
    j
end

varTypes2D = ["double", "double", "double", "double"];
varNames2D = ["q","Av_Efficiency", "Av_TimeOfDeterrence", "Av_LengthTraveled"];
stats_table2D = table('Size', [length(Q), 4],'VariableTypes',varTypes2D, 'VariableNames',varNames2D);

for i = 1:length(Q)
    mean_eff(i) = mean(statsEff2D(:, i))*100;
    mean_t(i) = mean(statsTime2D(:, i), 'omitnan');
    mean_l(i) = mean(statsLpath2D(:, i), 'omitnan');
    stats_table2D(i, :) = {Q(i), mean_eff(i), mean_t(i), mean_l(i)};
end

stats_table2D;

% calculation of q_opt
alpha = 0.2;        % weight of mean efficiency
beta = 0.5;         % weight of mean time of deterrence
gamma = 0.3;        % weight of mean lenght traveled
calcu = alpha*mean_eff + beta*(1-mean_t) + gamma*(1-mean_l);
[~, index] = max(alpha*mean_eff + beta*(1-mean_t) + gamma*(1-mean_l));
q_opt = Q(index);

% Plotting
figure(1)
tcl = tiledlayout(3,1);
[t,s] = title(tcl,['2D strategy - ', testing_edge, ' edge - N = ',num2str(N),' iterations'], ['q_o_p_t = ', num2str(q_opt)]);
t.FontSize = 14;
t.FontWeight = 'bold';
s.FontAngle = 'italic';
% efficiency 
nexttile
bar(Q, mean_eff)
title('Average Efficiency')
xticks(Q)
ylim([0 100])
% time
nexttile
bar(Q, mean_t)
title('Average deterrence time (normalized)')
xticks(Q)
ylim([0.5 1])
% length 
nexttile
bar(Q, mean_l)
title('Average length traveled (normalized)')
xticks(Q)
xlabel('q') 
ylim([0.5 1])

% plotting tested points
figure(2)
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