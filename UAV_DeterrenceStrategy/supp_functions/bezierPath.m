%% Function that gives the quadratic bezier trajectory, to be used as waypoints
% Static target until it is being chased, position known from start

function [b, curve_length] = bezierPath(world, agent_assigned, target_assigned, edge, inter_mult, plotting)
    % initial position of the agent
    agent_x = agent_assigned.x(1);
    agent_y = agent_assigned.y(1);

    % initial position of target
    target_x = target_assigned.x(1);
    target_y = target_assigned.y(1);

    % calculation of inter and end point
    [points, yaw_end, target_alert_radius] = quad_b_points(world, target_assigned, edge, inter_mult);
    P = cat(1,[agent_x, agent_y], points, [target_x, target_y]);     % P = [start, inter, end, target]
    P_x = P(:,1);    % used for plotting
    P_y = P(:,2);
    P(end,:) = [];   % points used for the curve (start, inter, end) but not target

    % calculation of bezier path
    b = [];     % b will contain the [x,y] coordinates of each computed point of the curve
    for t = 0:0.01:1
        b_t = quadBezier(t,P);
        b = cat(1, b, b_t);
    end
    
    curve_length = 0;
    for i=1:1:length(b)-1
        curve_length = curve_length + euclideanDistance(b(i,1),b(i,2),b(i+1,1),b(i+1,2));
    end

    if plotting == 1
        % plotting
        hold on;
        %     drawBoundary;
        %     drawRectangleColor(world.world_centre, world.size_x-400, world.size_y-400);

        % plotting control points
        plot(P_x,P_y, 'o', 'markersize', 4);
        plot(P_x(1:end-1),P_y(1:end-1), ':', 'markersize', 4,'color','black');

        % plotting labels
        labels = {'P0', 'P1', 'P2', 'Target'};
        for i = 1:1:4
            text(P_x(i)+30, P_y(i)+30,labels(i));
        end

        % plotting of bird FID
        th = 0:pi/1000:2*pi;
        xunit = target_alert_radius * cos(th) + target_x;
        yunit = target_alert_radius * sin(th) + target_y;
        plot(xunit, yunit, 'LineStyle', '--', 'color', 'red', LineWidth=1);

        % plotting of bezier trajectory
        curve_b = plot(b(:,1), b(:,2), '-','markersize',5,'color','blue');
    end
end
