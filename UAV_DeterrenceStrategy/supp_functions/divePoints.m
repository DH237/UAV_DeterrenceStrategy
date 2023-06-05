%% function returning the coordinates of the dive point depending on the position of the target and the edge it has to be chased to

function [points, FID] = divePoints(world, target_assigned, edge, offset_mult, inter_mult_dive)
target_x = target_assigned.x(1);
target_y = target_assigned.y(1);
i_threshold = world.inter_threshold; 
FID = i_threshold*world.inter_alert_radius;      % FID : distance under which target will move because i < i_threshold
inter_dist = inter_mult_dive*FID;                   % distance from Pd to P1
dive_offset = (offset_mult)*FID;                    % Sets the offset of Pd relative to the target's position, with offset_mult in [0,1]
if edge == 'N'
    P1 = [target_x - inter_mult_dive*FID, target_y - dive_offset];
    P2 = [target_x - FID - dive_offset, target_y - dive_offset];
    Pd = [target_x, target_y - dive_offset];
elseif edge == 'E'
    P1 = [target_x - dive_offset, target_y - inter_mult_dive*FID];
    P2 = [target_x - dive_offset, target_y - FID - dive_offset];
    Pd = [target_x - dive_offset, target_y];
elseif edge == 'S'
    P1 = [target_x - inter_mult_dive*FID, target_y + dive_offset];
    P2 = [target_x - FID - dive_offset, target_y + dive_offset];
    Pd = [target_x, target_y + dive_offset];
elseif edge == 'W'
    P1 = [target_x + dive_offset, target_y - inter_mult_dive*FID];
    P2 = [target_x + dive_offset, target_y - FID - dive_offset];
    Pd = [target_x + dive_offset, target_y];
end
points = cat(1, P1, P2, Pd);
end
