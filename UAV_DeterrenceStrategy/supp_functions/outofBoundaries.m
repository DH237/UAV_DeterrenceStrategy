%% returns true if target out of boundaries, false otherwise

function result = outofBoundaries(target, cumulation)
target_x                = target.x(cumulation-1);
target_y                = target.y(cumulation-1);
result = false;
if target_x < 200 || target_x > 1200
    result = true;
elseif target_y < 200 || target_y > 1000
    result = true;
end
end