%% finds the closest edge ('N', 'E', 'S', 'W') from the given point

function edge = closestEdgeSimple(world, x, y)
    s_x = world.size_x-200;     % crop size adjusted
    s_y = world.size_y-200;
    distances = [];
    distances(1) = s_y - y;     % distance from N edge
    distances(2) = s_x - x;     % distance from E edge
    distances(3) = y - 200;     % distance from S edge
    distances(4) = x - 200;     % distance from W edge
    [~, index] = min(distances);
    if index == 1
        edge = 'N';
    elseif index == 2
        edge = 'E';
    elseif index == 3
        edge = 'S';
    elseif index == 4
        edge = 'W';
    end
end
