% ===================================================================
% This is the main function of the UAV deterrence project for the 2D
% strategy
%
% Created by: Zihao Wang
% Created date: 4 July 2017
% 
% Modified by: Dylan Hauss
% Last modified: 5 june 2023
% ===================================================================
clc; clear; close all;
animation_f         = false;
videoWrite_f        = false;

if videoWrite_f
    video_saver     = VideoWriter('simulation.avi');
    video_saver.FrameRate = 5;
    open(video_saver);
end

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
target             = Target(false, 400, 500,  20, deg2rad(270), 5,  350,  12,      deg2rad(45));
targets             = [target];

% initialise targets with initial interests
for k = 1:size(targets,1)
    targets(k)      = initInterests(targets(k), world);
end

%% plotting world
aircraft_scale      = 30;
multirotor_scale    = 80;
target_scale        = 90;
color_pallete       = ['r';'y';'k';'m'];
color_pallete_agent = ['g'];
plotWorld;

%% simulation parameters
t_init              = 0;
t                   = t_init; % seconds
dt                  = 1; % seconds
t_total             = 140;
cumulation          = 1;
iterations          = ceil(t_total/dt)+1;

% expand matrices to speed up code
world               = expandArrays(world, iterations);
for i = 1:size(agents,1)
    agents(i)       = expandArrays(agents(i), world, iterations);
end
for k = 1:size(targets,1)
    targets(k)      = expandArrays(targets(k), world, iterations);
end

% skip initial condition before the loop starts
cumulation          = cumulation+1;
t                   = t+dt;

%% Initiate Bezier flight path from initial conditions for first flight mode: guided trajectory
% distance under which target will move because i < i_threshold
FID = world.inter_threshold*world.inter_alert_radius;

% edge to chase target through
edge = closestEdge(world, targets(1), cumulation);

% set q parameter depending on targeted edge
qN = 1.1;
qE = 2.6;
qS = 2.8;
qW = 3.8;

if edge == 'N'
    q = qN;
elseif edge == 'E'
    q = qE;
elseif edge == 'S'
    q = qS;
elseif edge == 'W'
    q = qW;
end

% decide whether to plot bezier curves or not
plotting = 1;

% compute Bezier curve
[b, curve_length] = bezierPath(world, agents(1), targets(1), edge, q, 1);

l_path = 0;
outofBounds = 0;

%% main loop
while t <= t_total && outofBounds == 0
    % =============================================================================================
    % update target
    for k = 1:size(targets,1)
        agent_flight_mode = agents(1).flight_mode;
        outofBounds = outofBoundaries(targets(k), cumulation);
        if outofBounds == 1
            % collects data at the end of the loop
            deterrence = true;
            target_last_pos = [targets(k).x(cumulation-1) targets(k).y(cumulation-1)];
            deter_time = t
            edge_deter = closestEdge(world, targets(1), cumulation)
        elseif t == t_total
            % if the bird has not been deterred at the end of the total
            % time
            deterrence = false;
            target_last_pos = [targets(k).x(cumulation-1) targets(k).y(cumulation-1)];
            deter_time = t
        end
        % % update interests
        switch agent_flight_mode
            case 1
                targets(k)  = updateInterests(targets(k), agents, world, cumulation);
            case 2
                targets(k)  = updateInterests(targets(k), agents, world, cumulation);
            case 3
                targets(k)  = updateInterestsDive(targets(k), agents, world, cumulation);
        end
        % move target
        for i=1:1:length(agents)
            dist_ag_tar(i) = euclideanDistance(targets(k).x(cumulation-1), targets(k).y(cumulation-1), agents(i).x(cumulation-1), agents(i).y(cumulation-1));
        end
        target_interest = targets(i).inter_map(ceil(targets(i).y(cumulation-1)/world.grid_size), ceil(targets(i).x(cumulation-1)/world.grid_size), cumulation-1);
        if min(dist_ag_tar) <= FID || target_interest < world.inter_threshold
            agent_alert = true;     % meaning there is an agent within the FID of the bird
        else
            agent_alert = false;
        end
        targets(k)  = moveTarget(targets(k), world, cumulation, dt, agent_alert);
    end % end target loop
    
    % update global interests
    world           = updateGlobalInterests(world, targets, cumulation);
    
    % =============================================================================================
    % control agents
    for i = 1:size(agents,1)
        % move agents
        % experimental: get another agents coordinates
        other_agents_coordinates = [0,0];
        agents(i)   = moveAgentGC(agents(i), world, cumulation, dt, targets(1), b);
    end % end agent loop
    l_path = l_path + euclideanDistance(agents(1).x(cumulation),agents(1).y(cumulation),agents(1).x(cumulation-1),agents(1).y(cumulation-1));
    
    % =============================================================================================
    
    % ++++++++++++++++++++ debug figure +++++++++++++++++++
    figure(h2);
%     subplot(1,2,1);
    for j = 1:size(agents,1)
        delete(agents(j).figure_handle_debug);
        switch agents(j).type
            case 1
                agents(j).figure_handle_debug = uav(agents(j).x(cumulation), agents(j).y(cumulation), agents(j).z(cumulation),...
                         'roll', rad2deg(agents(j).roll(cumulation)), 'pitch', rad2deg(agents(j).pitch(cumulation)),...
                         'yaw', rad2deg(agents(j).yaw(cumulation)), 'scale', aircraft_scale, 'color', color_pallete_agent(j),...
                         'wing', color_pallete_agent(j), 'linestyle', 'none');
            case 2
                agents(j).figure_handle_debug = quadrotor(agents(j).x(cumulation), agents(j).y(cumulation), agents(j).z(cumulation),...
                         'roll', rad2deg(agents(j).roll(cumulation)), 'pitch', rad2deg(agents(j).pitch(cumulation)),...
                         'yaw', rad2deg(agents(j).yaw(cumulation)), 'scale', multirotor_scale, 'body', color_pallete_agent(j),...
                         'boom', color_pallete_agent(j), 'prop', color_pallete_agent(j), 'linestyle', 'none');
            otherwise
                agents(j).figure_handle_debug = uav(agents(j).x(cumulation), agents(j).y(cumulation), agents(j).z(cumulation),...
                         'roll', rad2deg(agents(j).roll(cumulation)), 'pitch', rad2deg(agents(j).pitch(cumulation)),...
                         'yaw', rad2deg(agents(j).yaw(cumulation)), 'scale', aircraft_scale, 'color', color_pallete_agent(j),...
                         'wing', color_pallete_agent(j), 'linestyle', 'none');
        end
        % draw position setpoint
        delete(agents(j).figure_handle_waypoint);
        agents(j).figure_handle_waypoint = plot3(agents(j).x_setpoint(cumulation), agents(j).y_setpoint(cumulation), agents(j).z(cumulation), ...
                '+', 'markersize', 8, 'color', color_pallete_agent(j),'linewidth',2);
    end
    for m = 1:size(targets,1)
        delete(targets(m).figure_handle_debug);
        targets(m).figure_handle_debug = birds(targets(m).x(cumulation), targets(m).y(cumulation), targets(m).z(cumulation), ...
                    'body', color_pallete(m), 'yaw', rad2deg(targets(m).heading(cumulation)), 'scale', target_scale);
        delete(targets(m).figure_handle_waypoint);
        targets(m).figure_handle_waypoint = plot3(targets(m).x_setpoint(cumulation), targets(m).y_setpoint(cumulation), targets(m).z(cumulation), ...
                    '^', 'markersize', 8, 'color', color_pallete(m));
    end
    delete(debug_map_handle);
%     debug_map_handle = surf(world.meshgrid_XX,world.meshgrid_YY,world.prob_global(:,:,cumulation-1), 'facealpha', 1, 'linestyle', '-');
    debug_map_handle = surf(world.meshgrid_XX,world.meshgrid_YY,world.inter_global(:,:,cumulation-1), 'facealpha', 0.5', 'linestyle', 'none');
%     debug_map_handle = surf(world.meshgrid_XX,world.meshgrid_YY,agents(1).cost, 'facealpha', 1, 'linestyle', '-');
%     debug_map_handle = surf(world.meshgrid_XX,world.meshgrid_YY,world.inter_global(:,:,cumulation-1),'facealpha', 1, 'linestyle', '-');
%     debug_map_handle = surf(world.meshgrid_XX,world.meshgrid_YY,targets(1).cost(:,:,cumulation-1), 'facealpha', 1, 'linestyle', 'none');
    title(['Interest map  -  t = ',num2str(t),'s'],'FontSize',30,'FontName','Times','FontWeight','Bold');
    colorbar;
%     subplot(1,2,2);
%     plot(cumulation, sum(sum(world.prob_global(:,:,cumulation-1)))/world.number_of_tiles,'--or','markersize',10,'linewidth',2);
%     hold on;
%     title('Normalised global probability vs Time');
%     xlabel('Time (s)'); ylabel('Global probability');
    pbaspect([1 1 1]);
    set(gcf, 'Position', [100 100 1200 800]);
    set(gca, 'FontSize', 12);
    
%     plot trails
    plot3(agents(1).x(cumulation), agents(1).y(cumulation), 30, '+','Color','g','linewidth',2,'MarkerSize',3); hold on;
    plot3(targets(1).x(cumulation), targets(1).y(cumulation), 30, '+','Color','r','linewidth',2,'MarkerSize',3); hold on;
    
    % plot boundaries
    drawRectangleColor(world.world_centre, world.size_x-400, world.size_y-400);

    if videoWrite_f
        current_frame = getframe(gcf);
        writeVideo(video_saver, current_frame);
    end
    

    % ******************** debug figure ********************
    
    % increment time step
    t               = t + dt;
    cumulation      = cumulation + 1;
end % end simulation loop

if videoWrite_f
    close(video_saver);
end
% animation;