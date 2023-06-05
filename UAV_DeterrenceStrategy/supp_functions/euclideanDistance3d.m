function distance = euclideanDistance(a_x,a_y,a_z,b_x,b_y,b_z)
% 
% This calculates the euclidean distance between given points
% if a_x ... b_y are single values, it is straightfoward
% if inputs are vector, the returned value is a vector
%

distance_squared    = (a_x-b_x).^2+(a_y-b_y).^2+(a_z-b_z).^2;
distance            = sqrt(distance_squared);

end