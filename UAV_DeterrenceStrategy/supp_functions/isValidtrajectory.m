%% checks the validity of a 2D trajectory : it must not enter FID prematurely

function valid = isValidtrajectory(target, FID, path)
valid = true;
for i = 1:length(path)
    dist = euclideanDistance(target.x(1),target.y(1),path(i,1),path(i,2));
    if dist < FID
        valid = false;
        return
    end
end
end