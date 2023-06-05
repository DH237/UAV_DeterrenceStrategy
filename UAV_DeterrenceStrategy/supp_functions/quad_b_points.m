% function that gives the P1 and P2 points  to build a quadratic curve

function [points, yaw_end, FID] = quad_b_points(world, target, edge, inter_mult)     % inter_mult adjusts the distance of inter point from end point
    i_threshold = world.inter_threshold; 
    FID = i_threshold*world.inter_alert_radius;      % distance under which target will move because i < i_threshold
    inter_dist = inter_mult*FID;
    target_x = target.x(1);
    target_y = target.y(1);
    if edge == 'N'
        P1 = [target_x, target_y-inter_dist];
        P2 = [target_x, target_y-FID];
        yaw_end = deg2rad(90);
    elseif edge == 'E'
        P1 = [target_x-inter_dist, target_y];
        P2 = [target_x-FID, target_y];
        yaw_end = deg2rad(0);
    elseif edge == 'S'
        P1 = [target_x, target_y+inter_dist];
        P2 = [target_x, target_y+FID];
        yaw_end = deg2rad(270);
    elseif edge == 'W'
        P1 = [target_x+inter_dist, target_y];
        P2 = [target_x+FID, target_y];
        yaw_end = deg2rad(180);
    end
    points = cat(1,P1, P2);
end