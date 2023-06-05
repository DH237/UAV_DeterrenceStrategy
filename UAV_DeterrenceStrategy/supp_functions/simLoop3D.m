%% Runs the simulation loop for the 3D strategy
% gives back whether the bird was deterred, deterrence time and length of
% path traveled by the agent

function [deterred desired_edge efficient t_deter l_path] = simLoop3D(world, agents, targets, SimParam, k)
%% simulation parameters
t_init              = SimParam(1);
t                   = SimParam(2);
dt                  = SimParam(3);
t_total             = SimParam(4);
cumulation          = SimParam(5);
iterations          = SimParam(6);
% skip initial condition before the loop starts
cumulation          = cumulation+1;
t                   = t+dt;

%% Initiate Bezier flight path from initial conditions for first flight mode: guided trajectory
% distance under which target will move because i < i_threshold
FID = world.inter_threshold*world.inter_alert_radius;

% edge to chase target through
edge = closestEdge(world, targets(1), cumulation);

% diving parameters
inter_mult_dive = 2;
offset_mult = 0.4;
closing_dist = k*FID;

% compute Bezier curve
[b_dive, ~] = bezierPathdive(world, agents(1), targets(1), edge, inter_mult_dive, offset_mult, 0);
% [~, curve_length_dive] = bezierPathdive3d(world, agents(1), targets(1), edge, inter_mult_dive, offset_mult);

outofBounds = 0;
deterred = 0;
t_deter = NaN;
l_path = NaN;
desired_edge = false;
efficient = false;

%% main loop
while t <= t_total && outofBounds == 0
    %% update target
    for k = 1:size(targets,1)
        agent_flight_mode = agents(1).flight_mode;
        outofBounds = outofBoundaries(targets(1), cumulation);
        if outofBounds == 1
            % collects data at the end of the loop
            deterred = true;
            target_last_pos = [targets(1).x(cumulation-1) targets(1).y(cumulation-1)];
            t_deter = t;
            edge_deter = closestEdge(world, targets(1), cumulation);
            if edge_deter == edge
                desired_edge = true;
            else
                desired_edge = false;
            end
            efficient = deterred*desired_edge;
        elseif t == t_total
            % if the bird has not been deterred at the end of the total
            % time
            deterred = false;
            target_last_pos = [targets(1).x(cumulation-1) targets(1).y(cumulation-1)];
            t_deter = t;
            edge_deter = closestEdge(world, targets(1), cumulation);
            if edge_deter == edge
                desired_edge = true;
            else
                desired_edge = false;
            end
            efficient = deterred*desired_edge;
        end

        % update interests depending on flight mode
        switch agent_flight_mode
            case 1
                targets(1)  = updateInterests(targets(1), agents, world, cumulation);
            case 2
                targets(1)  = updateInterests(targets(1), agents, world, cumulation);
            case 3
                targets(1)  = updateInterestsDive(targets(1), agents, world, cumulation);
        end

        % move target
        for i=1:1:length(agents)
            dist_ag_tar(i) = euclideanDistance(targets(1).x(cumulation-1), targets(1).y(cumulation-1), agents(i).x(cumulation-1), agents(i).y(cumulation-1));
        end
        target_interest = targets(i).inter_map(ceil(targets(i).y(cumulation-1)/world.grid_size), ceil(targets(i).x(cumulation-1)/world.grid_size), cumulation-1);
        if min(dist_ag_tar) <= FID || target_interest < world.inter_threshold
            agent_alert = true;     % meaning there is an agent within the FID of the bird
        else
            agent_alert = false;
        end
        targets(1)  = moveTarget(targets(1), world, cumulation, dt, agent_alert);
    end % end target loop

    % update global interests
    world           = updateGlobalInterests(world, targets, cumulation);

    %% control agents
    for i = 1:size(agents,1)
        % move agents
        % agents(i)   = moveAgentGC(agents(i), world, cumulation, dt, targets(1), b);
        agents(i)   = moveAgentGDC(agents(i), world, cumulation, dt, targets(1), b_dive, closing_dist);

    end % end agent loop

    % update l_path of the agent
     l_path = sum([l_path euclideanDistance(agents(1).x(cumulation),agents(1).y(cumulation),agents(1).x(cumulation-1),agents(1).y(cumulation-1))], 'omitnan');

    % increment time step
    t               = t + dt;
    cumulation      = cumulation + 1;
end % end simulation loop
end