%% Function that gives the quadratic bezier trajectory, to be used as waypoints
% Static target until it is being chased, position known from start

function [b_dive, curve_length_dive] = bezierPathdive(world, agent_assigned, target_assigned, edge, inter_mult_dive, offset_mult, plotting)
% initial position of the agent
agent_x = agent_assigned.x(1);
agent_y = agent_assigned.y(1);

% initial position of target
target_x = target_assigned.x(1);
target_y = target_assigned.y(1);

% calculation of P1, P2 and Pd
[points, FID] = divePoints(world, target_assigned, edge, offset_mult, inter_mult_dive);
P = cat(1,[agent_x, agent_y], points, [target_x, target_y]);     % P = [P0, P1, P2, Pd, target]
P_x = P(:,1);    % used for plotting
P_y = P(:,2);
P(end-1:end,:) = [];   % points used for the Bézier curve (P0, P1 and P2) but not Pd and Target

% calculation of bezier path
b_dive = [];     % b will contain the [x,y] coordinates of each computed point of the curve
for t = 0:0.01:1
    b_t = quadBezier(t,P);
    b_dive = cat(1, b_dive, b_t);
end

P2 = [points(2,1) points(2,2)];
Pd = [points(3,1) points(3,2)];
for t = 0:0.01:1
    p = (1-t)*P2 + t*Pd;
    b_dive = cat(1, b_dive, p);
end

curve_length_dive = 0;
%%     False calculation : doesnt consider 3d component
for i=1:1:length(b_dive)-1
    curve_length_dive = curve_length_dive + euclideanDistance(b_dive(i,1),b_dive(i,2),b_dive(i+1,1),b_dive(i+1,2));
end

if plotting == 1
    % plotting
    hold on;
    %     drawBoundary;
    %     drawRectangleColor(world.world_centre, world.size_x-400, world.size_y-400);

    % plotting control points
    plot(P_x(1:end-1),P_y(1:end-1), 'o', 'markersize', 4);
    plot(P_x(end),P_y(end), 'diamond', 'markersize', 4);
    plot(P_x(1:end-1),P_y(1:end-1), ':', 'markersize', 4,'color','black');

    % plotting labels
    labels = {'P0', 'P1', 'P2', 'Pd', 'Target'};
    for i = 1:1:3
        text(P_x(i)+20, P_y(i)+30,labels(i));
    end
    text(P_x(4)+30, P_y(4)+30,labels(4));
    text(P_x(5)+30, P_y(5)+30,labels(5));
    % plotting of bird FID
    th = 0:pi/1000:2*pi;
    xunit = FID * cos(th) + target_x;
    yunit = FID * sin(th) + target_y;
    plot(xunit, yunit, 'LineStyle', '--', 'color', 'red', LineWidth=1);

    % plotting of bezier trajectory
    curve_b = plot(b_dive(:,1), b_dive(:,2), '-','markersize',5,'color','blue');
end
end
