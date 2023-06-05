function [h] = birds(varargin)

bodyColor           = 0.5*[1 1 1];
boomColor           = 0.5*[1 1 1];
propColor           = 0.5*[1 1 1];
edgeColor           = 'k';
linestyle           = '-';
scale               = 1;
setroll             = false;
setpitch            = false;
setyaw              = false;

%% Parse Inputs
if nargin>2 && isnumeric(varargin{1}) && isnumeric(varargin{2}) && isnumeric(varargin{3})
    x = varargin{1};
    y = varargin{2};
    z = varargin{3};
    assert(isscalar(x)==1, 'quad input x must be a scalar.')
    assert(isscalar(y)==1, 'quad input y must be a scalar.')
    assert(isscalar(z)==1, 'quad input z must be a scalar.')
else
    x = 0;
    y = 0;
    z = 0;
end

tmp = strncmpi(varargin, 'body', 4);
if any(tmp)
    bodyColor = varargin{find(tmp)+1};
end

tmp = strncmpi(varargin, 'lines', 5);
if any(tmp)
    linestyle = varargin{find(tmp)+1};
end

tmp = strncmpi(varargin,'linecol',7)|strncmpi(varargin,'edgecol',3); 
if any(tmp)
    edgeColor = varargin{find(tmp)+1}; 
end

tmp = strcmpi(varargin,'scale');  
if any(tmp) 
    scale = varargin{find(tmp)+1}; 
    assert(isscalar(scale)==1,'It may seem redundant, but the scale must be a scalar.')
end

tmp = strncmpi(varargin, 'yaw', 3);
if any(tmp)
    setyaw = true;
    yaw = varargin{find(tmp)+1};
end

%% Dimensions
bodyWidth           = 0.1*scale;
bodyHeight          = 0.1*bodyWidth;
boomLength          = 0.25*scale;
boomRadius          = 0.02*boomLength;
propRadius          = 0.10*scale;
propHeight          = 0.1*propRadius;

%% Determine if a figure is already open
initialHoldState    = 0;
SetView = isempty(get(gcf,'CurrentAxes'));

if ~SetView
    if ishold
        initialHoldState = 1;
    else
        cla;
        SetView     = true;
    end
end

%% Draw surfaces
% body
p1 = [x-bodyWidth/1.5, y-bodyWidth/1.5, z];
p2 = [x+bodyWidth/1.5, y-bodyWidth/1.5, z];
p3 = [x, y+bodyWidth, z];
p = [p1;p2;p3];
if ~initialHoldState
    hold on
end

h(21) = fill3(p(:,1),p(:,2),p(:,3)-bodyHeight,bodyColor);
h(22) = fill3(p(:,1),p(:,2),p(:,3)+bodyHeight,bodyColor);
h(23) = fill3([p1(1),p2(1),p2(1),p1(1)],...
             [p1(2),p2(2),p2(2),p1(2)],...
             [p1(3)+bodyHeight,p2(3)+bodyHeight,p2(3)-bodyHeight,p1(3)-bodyHeight],...
             bodyColor);
h(24) = fill3([p1(1),p3(1),p3(1),p1(1)],...
             [p1(2),p3(2),p3(2),p1(2)],...
             [p1(3)+bodyHeight,p3(3)+bodyHeight,p3(3)-bodyHeight,p1(3)-bodyHeight],...
             bodyColor);
h(25) = fill3([p2(1),p3(1),p3(1),p2(1)],...
             [p2(2),p3(2),p3(2),p2(2)],...
             [p2(3)+bodyHeight,p3(3)+bodyHeight,p3(3)-bodyHeight,p2(3)-bodyHeight],...
             bodyColor);
         
% body 2
p1 = [x-bodyWidth/1.5, y-bodyWidth/1.5, z] + [0, 2*bodyWidth, 0];
p2 = [x+bodyWidth/1.5, y-bodyWidth/1.5, z] + [0, 2*bodyWidth, 0];
p3 = [x, y+bodyWidth, z] + [0, 2*bodyWidth, 0];
p = [p1;p2;p3];
if ~initialHoldState
    hold on
end

h(1) = fill3(p(:,1),p(:,2),p(:,3)-bodyHeight,bodyColor);
h(2) = fill3(p(:,1),p(:,2),p(:,3)+bodyHeight,bodyColor);
h(3) = fill3([p1(1),p2(1),p2(1),p1(1)],...
             [p1(2),p2(2),p2(2),p1(2)],...
             [p1(3)+bodyHeight,p2(3)+bodyHeight,p2(3)-bodyHeight,p1(3)-bodyHeight],...
             bodyColor);
h(4) = fill3([p1(1),p3(1),p3(1),p1(1)],...
             [p1(2),p3(2),p3(2),p1(2)],...
             [p1(3)+bodyHeight,p3(3)+bodyHeight,p3(3)-bodyHeight,p1(3)-bodyHeight],...
             bodyColor);
h(5) = fill3([p2(1),p3(1),p3(1),p2(1)],...
             [p2(2),p3(2),p3(2),p2(2)],...
             [p2(3)+bodyHeight,p3(3)+bodyHeight,p3(3)-bodyHeight,p2(3)-bodyHeight],...
             bodyColor);
         
% body 3
p1 = [x-bodyWidth/1.5, y-bodyWidth/1.5, z] + [0, -2*bodyWidth, 0];
p2 = [x+bodyWidth/1.5, y-bodyWidth/1.5, z] + [0, -2*bodyWidth, 0];
p3 = [x, y+bodyWidth, z] + [0, -2*bodyWidth, 0];
p = [p1;p2;p3];
if ~initialHoldState
    hold on
end

h(6) = fill3(p(:,1),p(:,2),p(:,3)-bodyHeight,bodyColor);
h(7) = fill3(p(:,1),p(:,2),p(:,3)+bodyHeight,bodyColor);
h(8) = fill3([p1(1),p2(1),p2(1),p1(1)],...
             [p1(2),p2(2),p2(2),p1(2)],...
             [p1(3)+bodyHeight,p2(3)+bodyHeight,p2(3)-bodyHeight,p1(3)-bodyHeight],...
             bodyColor);
h(9) = fill3([p1(1),p3(1),p3(1),p1(1)],...
             [p1(2),p3(2),p3(2),p1(2)],...
             [p1(3)+bodyHeight,p3(3)+bodyHeight,p3(3)-bodyHeight,p1(3)-bodyHeight],...
             bodyColor);
h(10) = fill3([p2(1),p3(1),p3(1),p2(1)],...
             [p2(2),p3(2),p3(2),p2(2)],...
             [p2(3)+bodyHeight,p3(3)+bodyHeight,p3(3)-bodyHeight,p2(3)-bodyHeight],...
             bodyColor);
         
% body 4
p1 = [x-bodyWidth/1.5, y-bodyWidth/1.5, z] + [2*bodyWidth, 0, 0];
p2 = [x+bodyWidth/1.5, y-bodyWidth/1.5, z] + [2*bodyWidth, 0, 0];
p3 = [x, y+bodyWidth, z] + [2*bodyWidth, 0, 0];
p = [p1;p2;p3];
if ~initialHoldState
    hold on
end

h(11) = fill3(p(:,1),p(:,2),p(:,3)-bodyHeight,bodyColor);
h(12) = fill3(p(:,1),p(:,2),p(:,3)+bodyHeight,bodyColor);
h(13) = fill3([p1(1),p2(1),p2(1),p1(1)],...
             [p1(2),p2(2),p2(2),p1(2)],...
             [p1(3)+bodyHeight,p2(3)+bodyHeight,p2(3)-bodyHeight,p1(3)-bodyHeight],...
             bodyColor);
h(14) = fill3([p1(1),p3(1),p3(1),p1(1)],...
             [p1(2),p3(2),p3(2),p1(2)],...
             [p1(3)+bodyHeight,p3(3)+bodyHeight,p3(3)-bodyHeight,p1(3)-bodyHeight],...
             bodyColor);
h(15) = fill3([p2(1),p3(1),p3(1),p2(1)],...
             [p2(2),p3(2),p3(2),p2(2)],...
             [p2(3)+bodyHeight,p3(3)+bodyHeight,p3(3)-bodyHeight,p2(3)-bodyHeight],...
             bodyColor);
         
% body 4
p1 = [x-bodyWidth/1.5, y-bodyWidth/1.5, z] + [-2*bodyWidth, 0, 0];
p2 = [x+bodyWidth/1.5, y-bodyWidth/1.5, z] + [-2*bodyWidth, 0, 0];
p3 = [x, y+bodyWidth, z] + [-2*bodyWidth, 0, 0];
p = [p1;p2;p3];
if ~initialHoldState
    hold on
end

h(16) = fill3(p(:,1),p(:,2),p(:,3)-bodyHeight,bodyColor);
h(17) = fill3(p(:,1),p(:,2),p(:,3)+bodyHeight,bodyColor);
h(18) = fill3([p1(1),p2(1),p2(1),p1(1)],...
             [p1(2),p2(2),p2(2),p1(2)],...
             [p1(3)+bodyHeight,p2(3)+bodyHeight,p2(3)-bodyHeight,p1(3)-bodyHeight],...
             bodyColor);
h(19) = fill3([p1(1),p3(1),p3(1),p1(1)],...
             [p1(2),p3(2),p3(2),p1(2)],...
             [p1(3)+bodyHeight,p3(3)+bodyHeight,p3(3)-bodyHeight,p1(3)-bodyHeight],...
             bodyColor);
h(20) = fill3([p2(1),p3(1),p3(1),p2(1)],...
             [p2(2),p3(2),p3(2),p2(2)],...
             [p2(3)+bodyHeight,p3(3)+bodyHeight,p3(3)-bodyHeight,p2(3)-bodyHeight],...
             bodyColor);         
%% Set roll, pitch, yaw
if setyaw
    rotate(h,[0 0 1],yaw-90,[x y z]);
end

%% Set view
% if SetView
%     view([140 30]);
%     axis tight equal
%     lighting gouraud
%     camlight
% else
%     axis auto
% end
