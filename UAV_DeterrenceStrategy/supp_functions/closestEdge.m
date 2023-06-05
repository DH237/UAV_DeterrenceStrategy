%% finds the closest edge ('N', 'E', 'S', 'W') from the target

function edge = closestEdge(world, target, cumulation)
    tar_x = target.x(cumulation-1);
    tar_y = target.y(cumulation-1);
    s_x = world.size_x-200;     % crop size adjusted
    s_y = world.size_y-200;
    distances = [];
    distances(1) = s_y - tar_y;     % distance from N edge
    distances(2) = s_x - tar_x;     % distance from E edge
    distances(3) = tar_y - 200;     % distance from S edge
    distances(4) = tar_x - 200;     % distance from W edge
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
