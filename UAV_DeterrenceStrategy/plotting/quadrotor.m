function [h] = quadrotor(varargin)

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

tmp = strncmpi(varargin, 'boom', 4);
if any(tmp)
    boomColor = varargin{find(tmp)+1};
end

tmp = strncmpi(varargin, 'prop', 4);
if any(tmp)
    propColor = varargin{find(tmp)+1};
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

tmp = strncmpi(varargin, 'roll', 4);
if any(tmp)
    setroll = true;
    roll = varargin{find(tmp)+1};
end

tmp = strncmpi(varargin, 'pitch', 5);
if any(tmp)
    setpitch = true;
    pitch = varargin{find(tmp)+1};
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

% boom
[xboom, zboom, yboom] = cylinder(boomRadius);
h(6) = surface([x-xboom,x+xboom],...
                [y-yboom*boomLength,y+yboom*boomLength],...
                [z-zboom,z+zboom],...
                'faceColor',boomColor,'linestyle',linestyle,'edgeColor',edgeColor);
h(7) = surface([x-xboom,x+xboom],...
                [y-yboom*boomLength,y+yboom*boomLength],...
                [z-zboom,z+zboom],...
                'faceColor',boomColor,'linestyle',linestyle,'edgeColor',edgeColor);
rotate(h(6),[0 0 1],45,[x,y,z]);
rotate(h(7),[0 0 1],-45,[x y z]);

% prop
[xprop, yprop, zprop] = cylinder(propRadius);
xlocation1 = x+sqrt(boomLength^2/2);
ylocation1 = y+sqrt(boomLength^2/2);
xlocation2 = x-sqrt(boomLength^2/2);
ylocation2 = y-sqrt(boomLength^2/2);
xlocation3 = x-sqrt(boomLength^2/2);
ylocation3 = y+sqrt(boomLength^2/2);
xlocation4 = x+sqrt(boomLength^2/2);
ylocation4 = y-sqrt(boomLength^2/2);
h(8) = surface([xlocation1+xprop],...
                [ylocation1+yprop],...
                [z+zprop*propHeight],...
                'faceColor',propColor,'linestyle',linestyle,'edgeColor',edgeColor);
h(9) = surface([xlocation2+xprop],...
                [ylocation2+yprop],...
                [z+zprop*propHeight],...
                'faceColor',propColor,'linestyle',linestyle,'edgeColor',propColor);
h(10) = surface([xlocation3+xprop],...
                [ylocation3+yprop],...
                [z+zprop*propHeight],...
                'faceColor',propColor,'linestyle',linestyle,'edgeColor',propColor);
h(11) = surface([xlocation4+xprop],...
                [ylocation4+yprop],...
                [z+zprop*propHeight],...
                'faceColor',propColor,'linestyle',linestyle,'edgeColor',propColor);
propFactor = 1;
h(12) = fill3(xlocation1+xprop(1,:)*propFactor,ylocation1+yprop(1,:)*propFactor,z+zprop(1,:)+propHeight,propColor);
h(13) = fill3(xlocation1+xprop(1,:)*propFactor,ylocation1+yprop(1,:)*propFactor,z+zprop(1,:),propColor);
h(14) = fill3(xlocation2+xprop(1,:)*propFactor,ylocation2+yprop(1,:)*propFactor,z+zprop(1,:)+propHeight,propColor);
h(15) = fill3(xlocation2+xprop(1,:)*propFactor,ylocation2+yprop(1,:)*propFactor,z+zprop(1,:),propColor);
h(16) = fill3(xlocation3+xprop(1,:)*propFactor,ylocation3+yprop(1,:)*propFactor,z+zprop(1,:)+propHeight,propColor);
h(17) = fill3(xlocation3+xprop(1,:)*propFactor,ylocation3+yprop(1,:)*propFactor,z+zprop(1,:),propColor);
h(18) = fill3(xlocation4+xprop(1,:)*propFactor,ylocation4+yprop(1,:)*propFactor,z+zprop(1,:)+propHeight,propColor);
h(19) = fill3(xlocation4+xprop(1,:)*propFactor,ylocation4+yprop(1,:)*propFactor,z+zprop(1,:),propColor);

%% Set roll, pitch, yaw
if setroll
    rotate(h,[0 1 0],roll,[x y z]);
end

if setpitch
    rotate(h,[1 0 0],pitch,[x y z]);
end

if setyaw
    rotate(h,[0 0 1],yaw-90,[x y z]);
end

% %% Set view
% if SetView
%     view([140 30]);
%     axis tight equal
%     lighting gouraud
%     camlight
% else
%     axis auto
% end
