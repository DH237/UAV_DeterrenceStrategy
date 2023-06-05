%% Simulation code to test the 3D strategy
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

% set k parameter
K = [0.2, 0.4, 0.6, 0.8];

% define results table
varTypes3D = ["double", "double", "logical", "logical", "logical", "double", "double", "double"];
varNames3D = ["InitialPos_X", "InitialPos_Y", "Deterred", "DesiredEdge", "Efficient", "k", "TimeOfDeterrence", "LengthTraveled"];
results3D = table('Size', [length(K) 8],'VariableTypes',varTypes3D, 'VariableNames',varNames3D);

% number of configurations to test
N = 500;

% stats
statsTime3D = zeros(N, length(K));
statsLpath3D = zeros(N, length(K));
statsEff3D = zeros(N, length(K));

% random locations for target
target_x_ini = randi([300 1100], 1, N);
target_y_ini = randi([300 900], 1, N);


% loop to test on random initial positions of target
for j = 1:N
    target = Target(false, target_x_ini(j), target_y_ini(j),  20, deg2rad(270), 5,  350,  12,      deg2rad(45));
    targets             = [target];
    targets(1)      = expandArrays(targets(1), world, iterations);
    for i = 1:length(K)
        % 2 dimensions strategy
        [deterred desired_edge efficient t_deter l_path] = simLoop3D(world, agents, targets, SimParam, K(i));
        results3D(i, :)          = {target_x_ini(j), target_y_ini(j), deterred, desired_edge, efficient, K(i), t_deter, l_path};
        statsTime3D(j, i)     = t_deter;
        statsLpath3D(j, i)    = l_path;
        statsEff3D(j, i)      = efficient;
        % 3 dimensions strategy
    end
    % normalize length of path and time
    statsLpath3D(j, :) = statsLpath3D(j, :)/max(statsLpath3D(j, :));
    statsTime3D(j, :) = statsTime3D(j, :)/max(statsTime3D(j, :));
    results3D;
    j
end

varTypes3D = ["double", "double", "double", "double"];
varNames3D = ["q","Av_Efficiency", "Av_TimeOfDeterrence", "Av_LengthTraveled"];
stats_table3D = table('Size', [length(K), 4],'VariableTypes',varTypes3D, 'VariableNames',varNames3D);

for i = 1:length(K)
    mean_eff(i) = mean(statsEff3D(:, i))*100;
    mean_t(i) = mean(statsTime3D(:, i), 'omitnan');
    mean_l(i) = mean(statsLpath3D(:, i), 'omitnan');
    stats_table3D(i, :) = {K(i), mean_eff(i), mean_t(i), mean_l(i)};
end

stats_table3D

% calculating k_opt
alpha = 0.4;        % weight of mean efficiency
beta = 0.3;         % weight of mean time of deterrence
gamma = 0.3;        % weight of mean lenght traveled
[~, index] = max(alpha*mean_eff + beta*(1-mean_t) + gamma*(1-mean_l));
k_opt = K(index)

% Plotting
figure(1)
tcl = tiledlayout(3,1);
[t,s] = title(tcl,['3D strategy - N = ',num2str(N),' iterations'], ['k_o_p_t = ', num2str(k_opt)]);
t.FontSize = 14;
t.FontWeight = 'bold';
s.FontAngle = 'italic';
% efficiency 
nexttile
bar(K, mean_eff)
title('Average Efficiency')
xticks(K)
ylim([0 100])
% time
nexttile
bar(K, mean_t)
title('Average deterrence time (normalized)')
xticks(K)
ylim([0.5 1])
% length 
nexttile
bar(K, mean_l)
title('Average length traveled (normalized)')
xticks(K)
xlabel('k') 
ylim([0.5 1])

saveas(gcf, 'stats3d')

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

% results3D

% Elapsed time
toc