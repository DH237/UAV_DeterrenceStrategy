%% gives random [x y] so that it is closest to a certain edge

function [x y] = randEdge(world, edge)
xmax = world.size_x;
ymax = world.size_y;
x = randi([210 xmax-210]);
y = randi([210 ymax-210]);
c_edge = closestEdgeSimple(world, x, y);
if edge == 'N'
    while c_edge ~= edge
        y = randi([210 ymax-210]);
        c_edge = closestEdgeSimple(world, x, y);
    end
elseif edge == 'E'
    x = randi([xmax-ymax/2+1 xmax-210]);
    c_edge = closestEdgeSimple(world, x, y);
    while c_edge ~= edge
        y = randi([210 ymax-210]);
        c_edge = closestEdgeSimple(world, x, y);
    end
elseif edge == 'S'
    cpt = 0;
    while c_edge ~= edge
        if cpt > 10
            x = randi([210 xmax-210]);
            cpt = 0;
        end
        y = randi([210 ymax-210]);
        c_edge = closestEdgeSimple(world, x, y);
        cpt = cpt + 1;
    end
elseif edge == 'W'
    x = randi([210 ymax/2-1]);
    c_edge = closestEdgeSimple(world, x, y);
    while c_edge ~= edge
        y = randi([201 ymax-201]);
        c_edge = closestEdgeSimple(world, x, y);
    end
end
end