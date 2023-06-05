% Bezier function gives the coordinates b=[x,y] at t, defined by the 3 points
% in P, using a quadratic expression

function b_t = quadBezier(t,P)        % t has to be in [0,1]
    n = length(P);              % number of points for bezier curve
    d = n-1;                    % degree of bezier curve to generate
    b_ti = zeros(1,d+1);
    for i = 0:1:d
        binom = factorial(d)/(factorial(i)*factorial(d-i));
        b_ti(i+1) = binom*t^(i)*(1-t)^(d-i);
    end
    b_t = b_ti*P;
end
