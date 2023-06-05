classdef Target
    % ===============================================================
    % Class - Target
    % This class contains all the properties of the target object
    %
    % Methods: Protected
    %           [obj] = initInterests(obj, world)
    %           [obj] = integrateVelocity(obj, cumulation, dt)
    %           [obj] = integrateHeading(obj, cumulation, dt)
    %           [obj] = integratePosition(obj, cumulation, dt)
    %           [obj] = updateInterests(obj, agents, world, cumulation)
    %           [obj] = moveTarget(obj, world, cumulation, dt)
    %          Private
    %           [obj] = expandArrays(obj, world, iter)
    %
    % Created by: Zihao Wang
    % Last updated: 19-07-2018
    % ===============================================================
    properties
        % states
        x                           % x position
        y                           % y position
        z                           % z position
        heading                     % heading
        velocity                    % speed
        flight                      % flying or landed
        
        % performance
        sensor_range                % how far can they sense agents
        max_yawrate                 % maximum rate of heading change (rad/s)
        max_velocity                % maximum velocity
        
        % interests map
        inter_map                   % the target's interested location
        
        % figure handles
        figure_handle_uncertainty   % figure handle for uncertainty
        figure_handle_probability   % probability figure handle
        figure_handle_interests     % interests figure handle
        figure_handle_waypoint      % waypoint plot handle
        figure_handle_debug         % figure handle for debug
        interests_map_handle        % interests map handle
        
        % system dynamics
        velocity_setpoint           % velocity setpoint
        heading_setpoint            % heading setpoint
        x_setpoint                  % position x setpoint
        y_setpoint                  % position y setpoint
        % gains
        pos_p               % position error to velocity setpoint gain
        vel_p               % velocity error to acceleration setpoint gain
        yaw_p               % heading error to yaw speed setpoint gain
        % cost map (experimental)
        cost
        % grid target distance (experimental)
        grid_target_dist

        alert_counter       % counts for how many iteration bird has been in alert because of agent

    end
    
    methods
        %% constructor
        function obj = Target(flight,x,y,z,heading,velocity,sensor_range,max_velocity,max_yawrate)
            if nargin > 0
                obj.flight      = flight;
                obj.x           = x;
                obj.y           = y;
                obj.z           = z;
                obj.heading     = heading;
                obj.velocity    = velocity;
                obj.sensor_range= sensor_range;
                obj.max_velocity= max_velocity;
                obj.max_yawrate = max_yawrate;
                obj.x_setpoint  = x;
                obj.y_setpoint  = y;
                
            % assign default values if no values are given
            else
                obj.flight      = true;
                obj.x           = 0;
                obj.y           = 0;
                obj.z           = 0;
                obj.heading     = 0;
                obj.velocity    = 10;
                obj.sensor_range= 300;
                obj.max_velocity= 10;
                obj.max_yawrate = deg2rad(20);
                obj.x_setpoint  = 0;
                obj.y_setpoint  = 0;
            end
            
            % gains
            obj.pos_p = 0.1;
            obj.vel_p = 0.5;
            obj.yaw_p = 0.5;
        end
        
        %% protected methods
        % ========================
        % initialise interests map
        % ========================
        function [obj] = initInterests(obj, world)
            obj.inter_map           = ones(world.number_of_tiles_y, world.number_of_tiles_x).*world.inter0;
        end
        
        % ========================
        % integrate velocity
        % ========================
        function [obj] = integrateVelocity(obj, cumulation, dt)
            % velocity error = setpoint - current velocity
            velocity_error          = obj.velocity_setpoint(cumulation) - obj.velocity(cumulation-1);
            % integrate velocity
            delta_velocity          = velocity_error*obj.vel_p;
            obj.velocity(cumulation)= min(obj.velocity(cumulation-1)+delta_velocity*dt,obj.max_velocity);
        end
        
        % ========================
        % integrate heading
        % ========================
        function [obj] = integrateHeading(obj, cumulation, dt)
            % extract coordinates to make code look cleaner
%             target_x                = obj.x(cumulation-1);
%             target_y                = obj.y(cumulation-1);

            % heading error = setpoint - current heading
            heading_error           = obj.heading_setpoint-obj.heading(cumulation-1);
%             if [target_x target_y] == [obj.x(cumulation) obj.y(cumulation)]
%                 heading_error = 0;
%             end
            % constrain heading error to the maximum yaw rate according to direction
            if heading_error > 0
                delta_yaw           = min(obj.yaw_p*heading_error,obj.max_yawrate);
            elseif heading_error < 0
                delta_yaw           = max(obj.yaw_p*heading_error,-obj.max_yawrate);
            else
                delta_yaw           = 0;
            end
            obj.heading(cumulation) = wrapTo2Pi(obj.heading(cumulation-1)+delta_yaw*dt);
        end
        
        % ========================
        % integrate position
        % ========================
        function [obj] = integratePosition(obj, cumulation, dt)
            % distance error = setpoint - current position
            distance_error          = sqrt((obj.x_setpoint(cumulation)-obj.x(cumulation-1))^2+(obj.y_setpoint(cumulation)-obj.y(cumulation-1))^2);
            % velocity setpoint = position gain * distance error
            % constrain velocity setpoint to the maximum
            obj.velocity_setpoint(cumulation) = min(obj.pos_p*distance_error,obj.max_velocity);
            % integrate velocity
            [obj]                   = integrateVelocity(obj, cumulation, dt);
            % heading setpoint = angle between position setpoint and current position in global coordinates
            obj.heading_setpoint    = wrapTo2Pi(atan2((obj.y_setpoint(cumulation)-obj.y(cumulation-1)),obj.x_setpoint(cumulation)-obj.x(cumulation-1)));
            % integrate heading
            [obj]                   = integrateHeading(obj, cumulation, dt);
            % integrate position
            obj.x(cumulation)       = obj.x(cumulation-1) + obj.velocity(cumulation-1)*cos(obj.heading(cumulation-1))*dt;
            obj.y(cumulation)       = obj.y(cumulation-1) + obj.velocity(cumulation-1)*sin(obj.heading(cumulation-1))*dt;
        end
        
        % ========================
        % update interests
        % ========================
        function [obj] = updateInterests(obj, agents, world, cumulation)
            % load the last global interests into a temporary variable
            data                = world.inter_global(:,:,cumulation-1);

            % drive interests value towards nominal
            data                = data.*world.inter_tau + world.inter_nom*(1-world.inter_tau);
            
            % extract coordinates to make code look cleaner
            target_x            = obj.x(cumulation-1);
            target_y            = obj.y(cumulation-1);

            for i = 1:size(agents,1)
                % birds do not react to fixed wing type UAV
                if agents(i).type == 1
                    continue;
                end

                % extract coordinates to make code look cleaner
                agent_x             = agents(i).x(cumulation-1);
                agent_y             = agents(i).y(cumulation-1);

                % calculate distance between target and agent
                target_agent_dist   = euclideanDistance(agent_x,agent_y,target_x,target_y);

                % if agent is not in target's sensor range, continue
                if target_agent_dist > obj.sensor_range
                    continue;
                end

                % calculate distance between all grids and agent
                grid_agent_dist     = euclideanDistance(agent_x,agent_y,world.grid_centre_x,world.grid_centre_y);

                % all tiles in the alert radius have a decreased interests proportional to distance
                intersection_alert  = grid_agent_dist < world.inter_alert_radius;
                % minimum of the existing value or new value
                data(intersection_alert) = min(data(intersection_alert), grid_agent_dist(intersection_alert)./world.inter_alert_radius);

                % the rest of the tiles in the nogo radius have 0 interests
                intersection_nogo   = grid_agent_dist < world.inter_nogo_radius;
                data(intersection_nogo) = world.inter_nogo;
            end % end agents for loop

            % update target
            obj.inter_map(:,:,cumulation) = data;
        end
        
        % ========================
        %% update interests DIVE VERSION
        % ========================
        function [obj] = updateInterestsDive(obj, agents, world, cumulation)
            % load the last global interests into a temporary variable
            data                = world.inter_global(:,:,cumulation-1);

            % drive interests value towards nominal
            data                = data.*world.inter_tau_dive + world.inter_nom*(1-world.inter_tau_dive);
            
            % extract coordinates to make code look cleaner
            target_x            = obj.x(cumulation-1);
            target_y            = obj.y(cumulation-1);

            for i = 1:size(agents,1)
                % birds do not react to fixed wing type UAV
                if agents(i).type == 1
                    continue;
                end

                % extract coordinates to make code look cleaner
                agent_x             = agents(i).x(cumulation-1);
                agent_y             = agents(i).y(cumulation-1);

                % calculate distance between target and agent
                target_agent_dist   = euclideanDistance(agent_x,agent_y,target_x,target_y);

                % if agent is not in target's sensor range, continue
                if target_agent_dist > obj.sensor_range
                    continue;
                end

                % calculate distance between all grids and agent
                grid_agent_dist     = euclideanDistance(agent_x,agent_y,world.grid_centre_x,world.grid_centre_y);
                
                %% Parameters of the cones
                % parameters of the 1st cone
                lambda1 = world.inter_alert_radius;      % arbitrary equals alert radius
                rho1 = deg2rad(10);
                delta1 = lambda1*tan(rho1);

                % second cone
                rho2 = deg2rad(20);
                lambda2 = (lambda1 + delta1)/(1+tan(rho2));
                delta2 = lambda2*tan(rho2);

                % third cone
                rho3 = deg2rad(30);
                lambda3 = (lambda1 + delta1)/(1+tan(rho3));
                delta3 = lambda3*tan(rho3);

                % coordinates of M1
                yaw = agents(i).yaw(cumulation-1);
                x_M1 = agent_x + lambda1*cos(yaw);
                y_M1 = agent_y + lambda1*sin(yaw);

                % coodrinates of M2
                x_M2 = agent_x + lambda2*cos(yaw);
                y_M2 = agent_y + lambda2*sin(yaw);

                % coodrinates of M3
                x_M3 = agent_x + lambda3*cos(yaw);
                y_M3 = agent_y + lambda3*sin(yaw);

                % vector from agent to M
                v_am = [x_M1-agent_x, y_M1-agent_y];

                % setting boundaries to speed up the for loop
                i_inf = ceil(agent_x - lambda1 - delta1);
                i_sup = ceil(agent_x + lambda1 + delta1);
                if i_inf <= 0
                    i_inf = 1;
                end
                if i_sup > world.size_x
                    i_sup = world.size_x;
                end

                j_inf = ceil(agent_y - lambda1 - delta1);
                j_sup = ceil(agent_y + lambda1 + delta1);
                if j_inf <= 0
                    j_inf = 1;
                end
                if j_sup > world.size_y
                    j_sup = world.size_y;
                end

                % 3-cone shaped interest during dive mode
                for i = i_inf:i_sup
                    for j = j_inf:j_sup
                        % heading from agent to (i,j) cell
                        v_ac = [i-agent_x,j-agent_y];
                        psi_ac = acos(dot(v_am,v_ac) / (norm(v_am) * norm(v_ac)));
                        if psi_ac <= rho1
                            % 1st cone
                            if euclideanDistance(i,j,agent_x,agent_y) <= lambda1
                                data_i = ceil(i/world.grid_size);
                                data_j = ceil(j/world.grid_size);
                                data(data_j,data_i) = 0.1;
                            elseif euclideanDistance(i,j,x_M1,y_M1) <= delta1
                                data_i = ceil(i/world.grid_size);
                                data_j = ceil(j/world.grid_size);
                                data(data_j,data_i) = 0.1;
                            end
                        elseif psi_ac <= rho2
                            % 2nd cone
                            if euclideanDistance(i,j,agent_x,agent_y) <= lambda2
                                data_i = ceil(i/world.grid_size);
                                data_j = ceil(j/world.grid_size);
                                data(data_j,data_i) = 0.2;
                            elseif euclideanDistance(i,j,x_M2,y_M2) <= delta2
                                data_i = ceil(i/world.grid_size);
                                data_j = ceil(j/world.grid_size);
                                data(data_j,data_i) = 0.2;
                            end
                        elseif psi_ac <= rho3
                            % 3rd cone
                            if euclideanDistance(i,j,agent_x,agent_y) <= lambda3
                                data_i = ceil(i/world.grid_size);
                                data_j = ceil(j/world.grid_size);
                                data(data_j,data_i) = 0.3;
                            elseif euclideanDistance(i,j,x_M3,y_M3) <= delta3
                                data_i = ceil(i/world.grid_size);
                                data_j = ceil(j/world.grid_size);
                                data(data_j,data_i) = 0.3;
                            end
                        end
                    end
                end
                

%                 % all tiles in the alert radius have a decreased interests proportional to distance
                intersection_alert  = grid_agent_dist < world.inter_alert_radius_dive;
%                 % minimum of the existing value or new value
                data(intersection_alert) = min(data(intersection_alert), grid_agent_dist(intersection_alert)./world.inter_alert_radius_dive);

                % the rest of the tiles in the nogo radius have 0 interests
                intersection_nogo   = grid_agent_dist < world.inter_nogo_radius;
                data(intersection_nogo) = world.inter_nogo;
            end % end agents for loop

            % update target
            obj.inter_map(:,:,cumulation) = data;
        end
        % ========================
        % move target
        % ========================
        function [obj] = moveTarget(obj, world, cumulation, dt, agent_alert)
            % extract coordinates to make code look cleaner
            target_x                = obj.x(cumulation-1);
            target_y                = obj.y(cumulation-1);

            % weights depending on flight property
            if obj.flight == true
                alpha                   = 0.7; % interests weight when flying
                beta                    = 0.2; % distance weight when flying
                gamma                   = 0.1; % heading weight when flying
            else
                alpha                   = 0.8; % interests weight when landed
                beta                    = 0.2; % distance weight when landed
                gamma                   = 0.0; % heading weight when landed
            end
            
            if agent_alert      % = if there is an agent within alert radius of bird : move bird
                obj.alert_counter = obj.alert_counter+1;
                % update flight property of bird
                if obj.alert_counter > 1   % we consider that   before that many iteration, bird can still move as if landed because of low speed
                    obj.flight = true;
                end

                % calculate distance between target and grid centres
                grid_target_dist        = euclideanDistance(target_x,target_y,world.grid_centre_x,world.grid_centre_y);

                % find all grids unreachable by the target using receding horizon
                unreachable_grids       = ~(grid_target_dist < (obj.max_velocity*(world.RHC_steps)*dt));

                % calculate relative heading change between target and grid centres
                grid_target_yaw         = wrapTo2Pi(atan2((world.grid_centre_y-target_y),(world.grid_centre_x-target_x)));

                % calculate cost to reach each grid centres
                
                % interests cost
                interests_cost          = alpha*(1-world.inter_global(:,:,cumulation-1));
                % distance cost
                distance_cost           = beta*grid_target_dist./max(max(grid_target_dist));
                % heading cost
                heading_difference      = abs(grid_target_yaw-obj.heading(cumulation-1));
                heading_difference(heading_difference>=pi)=2*pi-heading_difference(heading_difference>=pi);
                heading_cost            = gamma.*heading_difference./pi;
                % unreachable grids
                unreachable_cost        = 10*unreachable_grids;
                % total cost
                obj.cost                = unreachable_cost + ...
                    interests_cost + ...
                    distance_cost + ...
                    heading_cost;

                % find the optimum waypoint
                [temp,waypoint_x]       = min(obj.cost);
                [~,waypoint_y]          = min(temp);
                waypoint_x              = waypoint_x(waypoint_y);

            else    % if no agent within alert radius of bird : next waypoint is current location of bird, no movement
                obj.flight = false;
                waypoint_x = ceil(target_y/world.grid_size);
                waypoint_y = ceil(target_x/world.grid_size);
            end
            
            % assign the optimum waypoint to the position set point
            obj.x_setpoint(cumulation) = world.grid_centre_x(waypoint_x,waypoint_y);
            obj.y_setpoint(cumulation) = world.grid_centre_y(waypoint_x,waypoint_y);

            % integrate
            obj                     = integratePosition(obj, cumulation, dt);
        end
        
        %% private methods
        % ========================
        % expand arrays
        % ========================
        function [obj] = expandArrays(obj, world, iter)
            obj.x(2:iter,:)         = ones(iter-1,1);
            obj.y(2:iter,:)         = ones(iter-1,1);
            obj.z(2:iter,:)         = ones(iter-1,1)*obj.z(1);
            obj.heading(2:iter,:)   = ones(iter-1,1);
            obj.velocity(2:iter,:)  = ones(iter-1,1);
            obj.flight(2:iter,:)    = ones(iter-1,1);
            
            obj.x_setpoint(2:iter,:)= ones(iter-1,1);
            obj.y_setpoint(2:iter,:)= ones(iter-1,1);
            
            obj.inter_map(:,:,2:iter) = ones(world.number_of_tiles_y, world.number_of_tiles_x, iter-1);
        end
    end
end