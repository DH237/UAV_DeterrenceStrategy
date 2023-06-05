% ===================================================================
% This script draws the simulation environment
% void
%
% Created by: Zihao Wang
% Created date: 4 July 2017
% ===================================================================
switch world.shape
    case 'circle'
        drawCircle(world.world_centre, world.size_other);
    case 'square'
        drawSquare(world.world_centre, world.size_other);
    case 'rectangle'
        drawRectangle(world.world_centre, world.size_x, world.size_y);
    otherwise
        warning('Invalid simulation environment shape argument, defaults to square.');
        drawSquare(world.world_centre, world.world_size);
end

axis equal; axis tight; 
%grid on;