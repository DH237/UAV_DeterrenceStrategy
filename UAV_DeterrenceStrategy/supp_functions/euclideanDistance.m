function distance = euclideanDistance(a_x,a_y,b_x,b_y)
% ===============================================================
% This calculates the euclidean distance between given points
% if a_x ... b_y are single values, it is straightfoward
% if inputs are vector, the returned value is a vector
%
% Created     : 17 July 2018
% Last updated: 17 July 2018
% By: Zihao Wang
% ===============================================================

distance_squared    = (a_x-b_x).^2+(a_y-b_y).^2;
distance            = sqrt(distance_squared);

end