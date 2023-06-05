classdef Aircraft
    % ===============================================================
    % Class - Aircraft
    % This class contains all the properties of the Aircraft
    %
    % Methods: Protected
    %           [obj] = initProbability(obj, world)
    %           [obj] = initUncertainty(obj, world)
    %           [obj] = integrateVelocity(obj, cumulation, dt)
    %           [obj] = getAcceleration(obj, cumulation, dt)
    %           [obj] = integrateYaw(obj, cumulation, dt)
    %           [obj] = getYawRate(obj, cumulation, dt)
    %           [obj] = getYawAcceleration(obj, cumulation, dt)
    %           [obj] = integratePosition(obj, cumulation, dt)
    %           [obj] = updateUncertainty(obj, world, cumulation)
    %           [obj] = updateProbability(obj, targets, world, cumulation)
    %           [obj] = moveAgent(obj, world, cumulation, dt)
    %           [obj] = getMultirotorWaypoint(obj, world, unreachable_grids, grid_agent_yaw, cumulation)
    %           [obj] = getFixedwingWaypoint(obj, world, unreachable_grids, grid_agent_yaw, cumulation)
    %          Private
    %           [obj] = expandArrays(obj, world, iter)
    %
    % Created by: Zihao Wang
    % Last updated: 25-07-2018
    % ===============================================================
    properties
        % type
        % type 1 = fixed wing
        % type 2 = multirotor
        type
        
        % states
        x                       % x-position
        y                       % y-position
        z                       % z-position
        roll                    % roll angle
        pitch                   % pitch angle
        yaw                     % heading angle
        yaw_rate                % heading change rate
        yaw_acceleration        % heading acceleration
        velocity                % velocity
        acceleration            % acceleration

        % fixed wing performance
        max_yawrate             % maximum yaw rate
        max_velocity            % maximum velocity
        max_velocity_dive       % maximum velocity when diving
        min_velocity            % minimum velocity
        max_acceleration        % maximum acceleration
        max_yaw_acceleration    % maximum yaw acceleration

        % sensor properties
        camera_fov              % camera field of view
        camera_range            % distance covered by the camera

        % probability grid and maps
        prob_map                % target probability map
        prob_map_sum            % sum of target probability map
        uncert_map              % environment uncertainty map
        uncert_map_sum          % sum of environment uncertainty map
        sensor_map              % a logic map represents all tiles in the sensor range
        detect_map              % a logic map represents all tiles marked as target detected
        nodetect_map            % a logic map represents all tiles marked as target not detected

        % figure handles
        figure_handle_uncertainty   % aircraft figure handle for uncertainty map
        figure_handle_probability   % aircraft figure handle for probability map
        figure_handle_interests     % aircraft figure handle for interests map
        figure_handle_debug         % aircraft figure handle for debug
        figure_handle_waypoint      % waypoint plot handle
        uncertainty_map_handle      % uncertainty map handle
        probability_map_handle      % probability map handle
        
        % system dynamics
        velocity_setpoint           % velocity setpoint
        acceleration_setpoint       % acceleration setpoint
        yaw_setpoint                % yaw (heading) setpoint
        yaw_rate_setpoint           % yaw rate setpoint
        yaw_acceleration_setpoint   % yaw acceleration setpoint
        x_setpoint                  % position x setpoint
        y_setpoint                  % position y setpoint
        % gains
        pos_p                       % position error to velocity setpoint gain
        vel_p                       % velocity error to acceleration setpoint gain
        yaw_p                       % heading error to yaw speed setpoint gain
        yaw_rate_p                  % yaw speed error to yaw acceleration setpoint gain
        yaw_acl_p                   % yaw acceleration error to yaw acceleration gain
        acl_p                       % acceleration error to acceleration gain
        % cost map (experimental)
        cost

        % flight mode
        flight_mode                 % either 1 for guided trajectory along a path or 2 for chase mode to assigned target, 3 for dive mode
        chase_mode_counter          % if >= 1, chase mode has already been engaged in the past
        
    end

    methods
        % constructor
        function obj = Aircraft(type,x,y,z,roll,pitch,yaw,velocity,max_velocity,min_velocity,max_yawrate)
            if nargin > 0
                obj.type            = type;
                obj.x               = x;
                obj.y               = y;
                obj.z               = z;
                obj.roll            = roll;
                obj.pitch           = pitch;
                obj.yaw             = yaw;
                obj.velocity        = velocity;
                obj.max_velocity    = max_velocity;
                obj.min_velocity    = min_velocity;
                obj.max_yawrate     = max_yawrate;
                            
            % assign default values if no values are given
            else
                obj.type            = 2;
                obj.x               = 0;
                obj.y               = 0;
                obj.z               = 0;
                obj.roll            = 0;
                obj.pitch           = 0;
                obj.yaw             = 0;
                obj.velocity        = 0;
                obj.max_velocity    = 10;
                obj.max_velocity_dive    = obj.max_velocity + 5;
                obj.min_velocity    = 0;
                obj.max_yawrate     = deg2rad(45);
            end
            
            % no acceleration at initial condition
            obj.acceleration        = 0;
            obj.yaw_rate            = 0;
            obj.yaw_acceleration    = 0;
            
            % temp max acceleration
            obj.max_acceleration    = 5;
            % temp max yaw acceleration
            obj.max_yaw_acceleration= deg2rad(45);
            
            % fixed parameters
            obj.camera_fov          = deg2rad(120);
            obj.camera_range        = 200;
            
            % gains
            % fixed wing gains
            if obj.type == 1
                obj.pos_p       = 0.7;
                obj.vel_p       = 0.8;
                obj.yaw_p       = 0.5;
                obj.yaw_rate_p  = 0.5;
                obj.yaw_acl_p   = 0.5;
                obj.acl_p       = 0.3;
            % multirotor gains
            else
                obj.pos_p       = 0.7;
                obj.vel_p       = 0.8;
                obj.yaw_p       = 0.5;
                obj.yaw_rate_p  = 0.5;
                obj.yaw_acl_p   = 0.5;
                obj.acl_p       = 0.5;
            end
            
            obj.flight_mode         = 1;
            obj.chase_mode_counter  = 0;
        end
        
        %% protected methods
        % ========================
        % initialise probability
        % ========================
        function [obj] = initProbability(obj, world)
            obj.prob_map            = ones(world.number_of_tiles_y, world.number_of_tiles_x).*world.prob0;
            obj.prob_map_sum        = sum(sum(obj.prob_map));
        end
        
        % ========================
        % initialise uncertainty
        % ========================
        function [obj] = initUncertainty(obj, world)
            obj.uncert_map          = ones(world.number_of_tiles_y, world.number_of_tiles_x).*world.uncert0;
            obj.uncert_map_sum      = sum(sum(obj.uncert_map));
        end
        
        % ========================
        % integrate velocity
        % ========================
        function [obj] = integrateVelocity(obj, cumulation, dt)
            % velocity error = setpoint - velocity
            velocity_error          = obj.velocity_setpoint(cumulation) - obj.velocity(cumulation-1);
            % acceleration setpoint = velocity gain * velocity error
            % limit acceleration setpoint according to direction
            if velocity_error >= 0
                obj.acceleration_setpoint(cumulation) = min(obj.max_acceleration, obj.vel_p*velocity_error);
            else
                obj.acceleration_setpoint(cumulation) = max(-obj.max_acceleration, obj.vel_p*velocity_error);
            end
            % get acceleration
            [obj]                   = getAcceleration(obj, cumulation, dt);
            delta_velocity          = obj.acceleration(cumulation)*dt;
            % integrate velocity
            obj.velocity(cumulation)= obj.velocity(cumulation-1)+delta_velocity;
        end
        
        % ========================
        % get acceleration
        % ========================
        function [obj] = getAcceleration(obj, cumulation, dt)
            % acceleration error = setpoint - acceleration
            acceleration_error      = obj.acceleration_setpoint(cumulation) - obj.acceleration(cumulation-1);
            % change in accleration = accleration gain * acceleration error * dt
            delta_acceleration      = obj.acl_p*acceleration_error*dt;
            % get acceleration
            obj.acceleration(cumulation) = obj.acceleration(cumulation) + delta_acceleration;
            % limit acceleration according to direction
            if obj.acceleration(cumulation) >= 0
                obj.acceleration(cumulation) = min(obj.max_acceleration, obj.acceleration(cumulation));
            else
                obj.acceleration(cumulation) = max(-obj.max_acceleration, obj.acceleration(cumulation));
            end
        end
        
        % ========================
        % integrate yaw (heading)
        % ========================
        function [obj] = integrateYaw(obj, cumulation, dt)
            % yaw error = yaw setpoint - current yaw
            yaw_error               = obj.yaw_setpoint(cumulation)-obj.yaw(cumulation-1);
            % normalise yaw error
            if yaw_error > pi
                yaw_error           = yaw_error - 2*pi;
            elseif yaw_error < -pi
                yaw_error           = yaw_error + 2*pi;
            end
            % yaw rate setpoint = yaw gain * yaw error
            % limit yaw rate to max yaw rate according to direction
            if yaw_error >= 0
                obj.yaw_rate_setpoint(cumulation) = min(obj.yaw_p*yaw_error,obj.max_yawrate);
            else
                obj.yaw_rate_setpoint(cumulation) = max(obj.yaw_p*yaw_error,-obj.max_yawrate);
            end
            % get yaw rate
            [obj]                   = getYawRate(obj, cumulation, dt);
            % integrate yaw
            delta_yaw               = obj.yaw_rate(cumulation) * dt;
            obj.yaw(cumulation)     = wrapTo2Pi(obj.yaw(cumulation-1)+delta_yaw);
        end
        
        % ========================
        % get yaw rate
        % ========================
        function [obj] = getYawRate(obj, cumulation, dt)
            % yaw rate error = yaw rate setpoint - current yaw rate
            yaw_rate_error          = obj.yaw_rate_setpoint(cumulation) - obj.yaw_rate(cumulation-1);
            % yaw acceleration setpoint = yaw rate gain * yaw rate error
            % limit yaw acceleration to max yaw acceleration according to direction
            if yaw_rate_error >= 0
                obj.yaw_acceleration_setpoint(cumulation) = min(obj.yaw_rate_p*yaw_rate_error, obj.max_yaw_acceleration);
            else
                obj.yaw_acceleration_setpoint(cumulation) = max(obj.yaw_rate_p*yaw_rate_error, -obj.max_yaw_acceleration);
            end
            % get yaw acceleration
            [obj]                   = getYawAcceleration(obj, cumulation, dt);
            % integrate yaw rate
            delta_yaw_rate          = obj.yaw_acceleration(cumulation) * dt;
            obj.yaw_rate(cumulation)= obj.yaw_rate(cumulation-1) + delta_yaw_rate;
        end
            
        % ========================
        % get yaw acceleration
        % ========================
        function [obj] = getYawAcceleration(obj, cumulation, dt)
            % yaw acceleration error = setpoint - yaw acceleration
            yaw_acceleration_error  = obj.yaw_acceleration_setpoint(cumulation) - obj.yaw_acceleration(cumulation-1);
            % change in yaw acceleration = yaw acceleration gain * yaw acceleration error * dt
            delta_yaw_acceleration  = obj.yaw_acl_p*yaw_acceleration_error*dt;
            % get yaw acceleration
            obj.yaw_acceleration(cumulation) = obj.yaw_acceleration(cumulation) + delta_yaw_acceleration;
            % limit yaw acceleration according to direction
            if obj.yaw_acceleration(cumulation) >= 0
                obj.yaw_acceleration(cumulation) = min(obj.max_yaw_acceleration, obj.yaw_acceleration(cumulation));
            else
                obj.yaw_acceleration(cumulation) = max(-obj.max_yaw_acceleration, obj.yaw_acceleration(cumulation));
            end
        end
            
        % ========================
        % integrate position
        % ========================
        function [obj] = integratePosition(obj, cumulation, dt)
            % distance error = position setpoint - current position
            distance_error                      = sqrt((obj.x_setpoint(cumulation)-obj.x(cumulation-1))^2+(obj.y_setpoint(cumulation)-obj.y(cumulation-1))^2);
            % velocity setpoint = position gain * distance error
            % limit velocity setpoint to maximum velocity
            obj.velocity_setpoint(cumulation)   = min(obj.pos_p*distance_error,obj.max_velocity);
            % constrain velocity setpoint to minimum velocity
            obj.velocity_setpoint(cumulation)   = max(obj.velocity_setpoint(cumulation), obj.min_velocity);
            % integrate velocity
            [obj]                               = integrateVelocity(obj, cumulation, dt);
            % yaw setpoint = the angle between position setpoint and current position in global coordinates
            obj.yaw_setpoint(cumulation)        = wrapTo2Pi(atan2((obj.y_setpoint(cumulation)-obj.y(cumulation-1)),obj.x_setpoint(cumulation)-obj.x(cumulation-1)));
            % integrate yaw
            [obj]                               = integrateYaw(obj, cumulation, dt);
            % integrate position
            obj.x(cumulation)                   = obj.x(cumulation-1) + obj.velocity(cumulation)*cos(obj.yaw(cumulation))*dt;
            obj.y(cumulation)                   = obj.y(cumulation-1) + obj.velocity(cumulation)*sin(obj.yaw(cumulation))*dt;
        end
        
        % ========================
        % update uncertainty map
        % ========================
        function [obj] = updateUncertainty(obj, world, cumulation)
            % load the last global uncertainty into a temporary variable
            data                    = world.uncert_global(:,:,cumulation-1);
            
            % drive uncertainty towards nominal
            data                    = data.*world.uncert_tau + world.uncert_nom*(1-world.uncert_tau);
            
            % extract some states to make code look cleaner
            agent_x                 = obj.x(cumulation-1);
            agent_y                 = obj.y(cumulation-1);
            agent_yaw               = obj.yaw(cumulation-1);
            
            % calculate relative distance and angle between all tiles and agent
            gird_agent_dist         = euclideanDistance(agent_x,agent_y,world.grid_centre_x,world.grid_centre_y);
            grid_agent_angle        = wrapTo2Pi(atan2((world.grid_centre_y-agent_y), (world.grid_centre_x-agent_x)));
            
            % find all tiles inside the sensor radius and field of view
            intersection1           = gird_agent_dist < obj.camera_range;
            intersection2           = grid_agent_angle > wrapTo2Pi(agent_yaw-obj.camera_fov/2);
            intersection3           = grid_agent_angle < wrapTo2Pi(agent_yaw+obj.camera_fov/2);
            if (agent_yaw >= obj.camera_fov/2) && (agent_yaw <= (2*pi-obj.camera_fov/2))
                intersection_sensor = intersection1&(intersection2&intersection3);
            else
                intersection_sensor = intersection1&(intersection2|intersection3);
            end
            
            % uncertainty within sensor range become 0
            data(intersection_sensor)= 0;
            
            % update object
            obj.uncert_map(:,:,cumulation) = data;
            obj.uncert_map_sum(cumulation) = sum(sum(data));
            obj.sensor_map          = intersection_sensor;
        end
        
        % ========================
        % update probability map
        % ========================
        function [obj] = updateProbability(obj, targets, world, cumulation)
            % load the last global probability into a temporary variable
            data                    = world.prob_global(:,:,cumulation-1);
            
            % drive probability towards nominal
            data                    = data.*world.prob_tau + world.prob_nom*(1-world.prob_tau);
            
            % extract coordinates to make code look cleaner
            agent_x                 = obj.x(cumulation-1);
            agent_y                 = obj.y(cumulation-1);
            agent_yaw               = obj.yaw(cumulation-1);
            
            % initialise the detection logic map
            obj.detect_map          = false(world.number_of_tiles_y, world.number_of_tiles_x);
            obj.nodetect_map         = obj.sensor_map;
            
            for k = 1:size(targets,1)
                % this flag indicates if target is in sensor range
                target_in_sensor    = true;
                
                % extract coordinates to make code look cleaner
                target_x            = targets(k).x(cumulation-1);
                target_y            = targets(k).y(cumulation-1);
                
                % calculate distance between target and agent
                target_agent_dist   = euclideanDistance(target_x, target_y, agent_x, agent_y);
                
                % calculate angular distance between target and agent
                target_agent_yaw    = wrapTo2Pi(atan2((target_y-agent_y),(target_x-agent_x)));
                
                % if target is not in the sensor range, negate flag
                if target_agent_dist > obj.camera_range
                    target_in_sensor= false;
                end
                
                % if target is not in the sensor field of view, continue
                if target_in_sensor
                    if (agent_yaw >= obj.camera_fov/2) && (agent_yaw <= (2*pi-obj.camera_fov/2))
                        if (target_agent_yaw<wrapTo2Pi(agent_yaw-obj.camera_fov/2)) || (target_agent_yaw>wrapTo2Pi(agent_yaw+obj.camera_fov/2))
                            target_in_sensor = false;
                        end
                    else
                        if ~((target_agent_yaw>wrapTo2Pi(agent_yaw-obj.camera_fov/2)) || (target_agent_yaw<wrapTo2Pi(agent_yaw+obj.camera_fov/2)))
                            target_in_sensor = false;
                        end
                    end
                end
                
                % calculate distance between all tiles and target
                targets(k).grid_target_dist    = euclideanDistance(target_x,target_y,world.grid_centre_x,world.grid_centre_y);
                
                % find all tiles with no detection and detection
                if target_in_sensor
                    % detection = tiles around the target
                    intersection_detect     = targets(k).grid_target_dist < world.prob_range;
                    % no detection = tiles covered by the agent's sensor - tiles around the target
                    intersection_nodetect   = obj.sensor_map & (~intersection_detect);
                    
                else
                    % detection = no tiles
                    intersection_detect     = false(world.number_of_tiles_y, world.number_of_tiles_x);
                    % no detection = all tiles covered by the agent's sensor
                    intersection_nodetect   = obj.sensor_map;
                end
                
                % drive probability in the area marked as detected towards nominal
                data(intersection_detect)   = data(intersection_detect).*world.prob_detect_tau + world.prob_detect*(1-world.prob_detect_tau);
                
                % drive probability in the area marked as no detect towards nominal
                data(intersection_nodetect) = data(intersection_nodetect).*world.prob_nodetect_tau + world.prob_nodetect*(1-world.prob_detect_tau);
                
                % update agent
                obj.detect_map              = obj.detect_map | intersection_detect;
                obj.nodetect_map            = obj.nodetect_map | intersection_nodetect;
            end
            
            % update agent
            obj.prob_map(:,:,cumulation)    = data;
            obj.prob_map_sum(cumulation)    = sum(sum(data));        
        end
        
        % ========================
        % move agent
        % ========================
        function [obj] = moveAgent(obj, world, cumulation, dt, other_agents_coordinates)
            % extract coordinates to make code look cleaner
            agent_x                 = obj.x(cumulation-1);
            agent_y                 = obj.y(cumulation-1);
            
            % calculate distance between agent and grid
            grid_agent_dist         = euclideanDistance(world.grid_centre_x,world.grid_centre_y,agent_x,agent_y);
            
            % find all grids unreachable by the target using receding horizon
            unreachable_grids       = ~(grid_agent_dist < (obj.max_velocity*world.RHC_steps*dt));
            
            % calculate relative heading change between target and grid centres
            grid_agent_yaw          = wrapTo2Pi(atan2((world.grid_centre_y-agent_y),(world.grid_centre_x-agent_x)));
            
            % find optimal waypoint for agent
            if obj.type == 1
                obj                 = getFixedwingWaypoint(obj, world, unreachable_grids, grid_agent_yaw, grid_agent_dist, cumulation);
            else
                obj                 = getMultirotorWaypoint(obj, world, unreachable_grids, grid_agent_yaw, grid_agent_dist, cumulation, other_agents_coordinates);
            end
            
            % integrate
            obj                     = integratePosition(obj, cumulation, dt);
        end
        
        % ========================
        % get multirotor optimal waypoint
        % ========================
        function [obj] = getMultirotorWaypoint(obj, world, unreachable_grids, grid_agent_yaw, grid_agent_dist, cumulation, other_agents_coordinates)
            % calculate cost to reach each grid centres
            % weights
            alpha = 0.4; % probability weight
            beta = 0.1;  % distance weight
            gamma = 0.5; % heading weight
            % probability cost
            probability_cost = alpha*(1-world.prob_global(:,:,cumulation-1));
            % distance cost
            distance_cost   = beta*grid_agent_dist./world.RHC_steps/obj.max_velocity; %forgot dt
            % heading cost
            heading_difference = abs(grid_agent_yaw-obj.yaw(cumulation-1));
            heading_difference(heading_difference>=pi)=2*pi-heading_difference(heading_difference>=pi);
            heading_cost = gamma.*heading_difference./pi;
            % unreachable grids
            unreachable_cost = 10*unreachable_grids;
            % add cost around other agents
            grid_other_agent_dist = euclideanDistance(world.grid_centre_x,world.grid_centre_y,other_agents_coordinates(1),other_agents_coordinates(2));
            otheragents_grids = (grid_other_agent_dist < (130));
            otheragents_cost = 10*otheragents_grids;
            % total cost
            obj.cost = unreachable_cost + ...
                        probability_cost + ...
                        distance_cost + ...
                        heading_cost + ...
                        otheragents_cost;
                        
            % find the optimum waypoint
            [temp, waypoint_x]      = min(obj.cost);
            [~, waypoint_y]         = min(temp);
            waypoint_x              = waypoint_x(waypoint_y);
            
            % assign the optimum waypoint to the position set point
            obj.x_setpoint(cumulation) = world.grid_centre_x(waypoint_x, waypoint_y);
            obj.y_setpoint(cumulation) = world.grid_centre_y(waypoint_x, waypoint_y);
        end
        
        % ==============================================
        %% NEW VERSION: GUIDED --> CHASE move agent
        % moves the agent depending on phase 1/2 of flight towards assigned target
        % ==============================================
        function [obj] = moveAgentGC(obj, world, cumulation, dt, target_assigned, b)
            % extract coordinates to make code look cleaner
            agent_x                 = obj.x(cumulation-1);
            agent_y                 = obj.y(cumulation-1);
            target_x                = target_assigned.x(cumulation-1);
            target_y                = target_assigned.y(cumulation-1);

            % FID : distance under which target will move because i < i_threshold
            FID = world.inter_threshold*world.inter_alert_radius;

            % find optimal waypoint for agent
            if obj.type == 1
                obj                 = getFixedwingWaypoint(obj, world, unreachable_grids, grid_agent_yaw, grid_agent_dist, cumulation);
            else
                if (euclideanDistance(agent_x, agent_y, target_x, target_y) <= FID) || (obj.chase_mode_counter > 0)
                    % Phase 2 flight : chasing mode
%                     disp('Phase 2 : chase mode');
                    obj                 = getTargetWaypoint(obj, world, target_assigned, cumulation);
                    obj.chase_mode_counter = obj.chase_mode_counter +1;
                    obj.flight_mode = 2;
                else
                    % Phase 1 flight : guided trajectory mode
%                     disp('Phase 1 : guided trajectory mode');
                    obj                 = getBezierWaypoint(obj, world, b, cumulation);
                end
            end

            % integrate
            obj                     = integratePosition(obj, cumulation, dt);
        end
        % ==============================================
        %% NEW VERSION: GUIDED --> DIVE --> CHASE move agent
        % moves the agent depending on phase 1/2/3 of flight towards assigned target
        % ==============================================
        function [obj] = moveAgentGDC(obj, world, cumulation, dt, target_assigned, b_dive, closing_dist)
            % find optimal waypoint for agent
            if obj.type == 1
                obj                 = getFixedwingWaypoint(obj, world, unreachable_grids, grid_agent_yaw, grid_agent_dist, cumulation);
            else
                if (obj.flight_mode == 2) || (obj.chase_mode_counter > 0)
                    % Chase mode
%                     disp('Phase 2 : chase mode');
                    obj.flight_mode         = 2;
                    obj                     = getTargetWaypoint(obj, world, target_assigned, cumulation);
                    obj.chase_mode_counter  = obj.chase_mode_counter +1;
                elseif obj.flight_mode == 3
                    % Dive mode
                    obj.flight_mode         = 3;
                    obj                     = getDiveWaypoint(obj, world, b_dive, cumulation, closing_dist);
                else
                    % Guided trajectory mode
%                     disp('Phase 1 : guided trajectory mode');
                    obj.flight_mode         = 1;
                    obj                     = getBezierDiveWaypoint(obj, world, b_dive, cumulation);
                end
            end

            % integrate
            obj                     = integratePosition(obj, cumulation, dt);
        end

        % ==============================================
        %% NEW FUNCTION dumb chase
        % sets waypoint of agent at target location
        % ==============================================
        function [obj] = moveAgentC(obj, world, cumulation, dt, target_assigned)
            % sets the location of target as waypoint
            obj                 = getTargetWaypoint(obj, world, target_assigned, cumulation);
            % integrate
            obj                     = integratePosition(obj, cumulation, dt);
        end
        
        % ==============================================
        %% NEW FUNCTION (Bezier) waypoint
        % Retrieves next waypoint on the initial Bezier trajectory computed
        % ==============================================
        function [obj] = getBezierWaypoint(obj, world, b, cumulation)
            % extract coordinates to make code look cleaner
            agent_x                 = obj.x(cumulation-1);
            agent_y                 = obj.y(cumulation-1);

            % distance from agent to its waypoint
            wpt_dist = 80;     

            % list of coordinates
            b_x = b(:,1);
            b_y = b(:,2);
            centres_x = world.grid_centre_x(1,:);
            centres_y = world.grid_centre_y(:,1);

            % list of waypoints 
            list_wpt_x = zeros(length(b_x),1);
            list_wpt_y = zeros(length(b_y),1);
            for i=1:1:length(list_wpt_x)
                [~, list_wpt_y(i)] = min(abs(centres_x-b_x(i)));        % exchange x and y bc error in code
                [~, list_wpt_x(i)] = min(abs(centres_y-b_y(i)));
            end

            % loop to have distance from agent to next waypoint to be <
            % wpt_dist_threshold distance. Otherwise agent cant follow up
            wpt_index = length(list_wpt_x);
            dist_agent_wpt = euclideanDistance(agent_x, agent_y, b_x(wpt_index), b_y(wpt_index));
            while dist_agent_wpt > wpt_dist
                wpt_index = wpt_index - 1;
                dist_agent_wpt = euclideanDistance(agent_x, agent_y, b_x(wpt_index), b_y(wpt_index));
            end
            waypoint_x = list_wpt_x(wpt_index);
            waypoint_y = list_wpt_y(wpt_index);

            % set waypoints
            obj.x_setpoint(cumulation) = world.grid_centre_x(waypoint_x, waypoint_y);
            obj.y_setpoint(cumulation) = world.grid_centre_y(waypoint_x, waypoint_y);
        end
        % ==============================================
        %% NEW FUNCTION DIVE (Bezier) waypoint
        % Retrieves next waypoint on the initial Bezier trajectory computed
        % ==============================================
        function [obj] = getBezierDiveWaypoint(obj, world, b_dive, cumulation)
            % extract coordinates to make code look cleaner
            agent_x                 = obj.x(cumulation-1);
            agent_y                 = obj.y(cumulation-1);

            % maximum distance from agent to its next waypoint
            wpt_dist = 80;     

            % list of coordinates
            b_x = b_dive(:,1);
            b_y = b_dive(:,2);
            centres_x = world.grid_centre_x(1,:);
            centres_y = world.grid_centre_y(:,1);

            % list of waypoints 
            list_wpt_x = zeros(length(b_x),1);
            list_wpt_y = zeros(length(b_y),1);
            for i=1:1:length(list_wpt_x)
                [~, list_wpt_y(i)] = min(abs(centres_x-b_x(i)));        % exchange x and y bc error in code
                [~, list_wpt_x(i)] = min(abs(centres_y-b_y(i)));
            end

            % loop to have distance from agent to next waypoint to be <
            % wpt_dist_threshold distance. Otherwise agent cant follow up
            wpt_index = length(list_wpt_x);
            dist_agent_wpt = euclideanDistance(agent_x, agent_y, b_x(wpt_index), b_y(wpt_index));
            while dist_agent_wpt > wpt_dist && wpt_index > 1
                wpt_index = wpt_index - 1;
                dist_agent_wpt = euclideanDistance(agent_x, agent_y, b_x(wpt_index), b_y(wpt_index));
            end
            waypoint_x = list_wpt_x(wpt_index);
            waypoint_y = list_wpt_y(wpt_index);
            
            % if the agent reached P2 then flight mode = dive
            if wpt_index > 101
                obj.flight_mode = 3;
            end

            % set waypoints
            obj.x_setpoint(cumulation) = world.grid_centre_x(waypoint_x, waypoint_y);
            obj.y_setpoint(cumulation) = world.grid_centre_y(waypoint_x, waypoint_y);
        end
        % ==============================================
        %% NEW FUNCTION get Dive waypoint
        % Sets Pd as waypoint for the agent
        % ==============================================
        function [obj] = getDiveWaypoint(obj, world, b_dive, cumulation, closing_dist)
            % extract coordinates to make code look cleaner
            agent_x                 = obj.x(cumulation-1);
            agent_y                 = obj.y(cumulation-1);

            % set waypoint to Pd
            waypoint_x = ceil(b_dive(end, 2)/world.grid_size);
            waypoint_y = ceil(b_dive(end, 1)/world.grid_size);

            dist_Pd    = euclideanDistance(agent_x, agent_y, b_dive(end, 1), b_dive(end, 2));
            % if agent distance to Pd under closing_dist --> switch to
            % chase mode
            if dist_Pd < closing_dist
                obj.flight_mode = 2;
            end
            obj.x_setpoint(cumulation) = world.grid_centre_x(waypoint_x, waypoint_y);
            obj.y_setpoint(cumulation) = world.grid_centre_y(waypoint_x, waypoint_y);
        end
        % ===================================
        %% NEW FUNCTION target waypoint
        % gives the exact position of the target as the next waypoint
        % ===================================
        function [obj] = getTargetWaypoint(obj, world, target_assigned, cumulation)
            % extract coordinates to make code look cleaner
            target_x                = target_assigned.x(cumulation-1);
            target_y                = target_assigned.y(cumulation-1);

            % assign next waypoint on the path
            waypoint_x = ceil(target_y/world.grid_size);
            waypoint_y = ceil(target_x/world.grid_size);

            obj.x_setpoint(cumulation) = world.grid_centre_x(waypoint_x, waypoint_y);
            obj.y_setpoint(cumulation) = world.grid_centre_y(waypoint_x, waypoint_y);
        end

        % ========================
        % get fixedwing optimal waypoint
        % ========================
        function [obj] = getFixedwingWaypoint(obj, world, unreachable_grids, grid_agent_yaw, grid_agent_dist, cumulation)
            % calculate cost to reach each grid centres
            alpha = 0.5; % probability weight
            beta = 0.1;  % distance weight
            gamma = 0.4; % heading weight
            % cost = yaw_change+0.7*(1-uncertainty)+0.3*(1-probability)
            % probability cost
            probability_cost = alpha*(1-world.prob_global(:,:,cumulation-1));
            % distance cost
            distance_cost   = beta*grid_agent_dist./world.RHC_steps/obj.max_velocity;
            % heading cost
            heading_difference = abs(grid_agent_yaw-obj.yaw(cumulation-1));
            heading_difference(heading_difference>=pi)=2*pi-heading_difference(heading_difference>=pi);
            heading_cost = gamma.*heading_difference./pi;
            % unreachable grids
            unreachable_cost = 10*unreachable_grids;
            % total cost
            obj.cost = unreachable_cost + ...
                        probability_cost + ...
                        distance_cost + ...
                        heading_cost;
                                  
            % find the optimum waypoint
            [temp, waypoint_x]      = min(obj.cost);
            [~, waypoint_y]         = min(temp);
            waypoint_x              = waypoint_x(waypoint_y);
            
            % assign the optimum waypoint to the position set point
            obj.x_setpoint(cumulation) = world.grid_centre_x(waypoint_x, waypoint_y);
            obj.y_setpoint(cumulation) = world.grid_centre_y(waypoint_x, waypoint_y);
        end
        
        %% private methods
        % ========================
        % expand arrays
        % ========================
        function [obj] = expandArrays(obj, world, iter)
            obj.x(2:iter,:)             = ones(iter-1,1).*obj.x(1);
            obj.y(2:iter,:)             = ones(iter-1,1).*obj.y(1);
            obj.z(2:iter,:)             = ones(iter-1,1).*obj.z(1);
            obj.roll(2:iter,:)          = ones(iter-1,1).*obj.roll(1);
            obj.pitch(2:iter,:)         = ones(iter-1,1).*obj.pitch(1);
            obj.yaw(2:iter,:)           = ones(iter-1,1).*obj.yaw(1);
            obj.yaw_rate(2:iter,:)      = ones(iter-1,1).*obj.yaw_rate(1);
            obj.yaw_acceleration(2:iter,:) = ones(iter-1,1).*obj.yaw_acceleration(1);
            obj.velocity(2:iter,:)      = ones(iter-1,1).*obj.velocity(1);
            obj.acceleration(2:iter,:)  = ones(iter-1,1).*obj.acceleration(1);
            
            obj.x_setpoint(2:iter,:)            = ones(iter-1,1);
            obj.y_setpoint(2:iter,:)            = ones(iter-1,1);
            obj.yaw_setpoint(2:iter,:)          = ones(iter-1,1);
            obj.yaw_rate_setpoint(2:iter,:)     = ones(iter-1,1);
            obj.yaw_acceleration_setpoint(2:iter,:) = ones(iter-1,1);
            obj.velocity_setpoint(2:iter,1)     = ones(iter-1,1);
            obj.acceleration_setpoint(2:iter,1) = ones(iter-1,1);
            
            obj.prob_map(:,:,2:iter)    = ones(world.number_of_tiles_y, world.number_of_tiles_x, iter-1);
            obj.prob_map_sum(2:iter)    = ones(iter-1,1);
            obj.uncert_map(:,:,2:iter)  = ones(world.number_of_tiles_y, world.number_of_tiles_x, iter-1);
            obj.uncert_map_sum(2:iter)  = ones(iter-1,1);
        end
    end
end