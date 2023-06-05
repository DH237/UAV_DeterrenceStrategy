%% function that computes the absolute angle between 2 2D vectors in rad

function angle = vectAngle(v1,v2)

% Compute the angle between the two vectors in radians
angle = acos(min(1,max(-1, v1(:).' * v2(:) / norm(v1) / norm(v2) )));

end
