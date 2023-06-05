function [handle] = drawCircle(centre, diameter)

x                   = centre(1);
y                   = centre(2);
radius              = diameter/2;
theta               = linspace(0,2*pi,100);

x_plot              = radius*cos(theta);
y_plot              = radius*sin(theta);

hold on;
handle = plot(x_plot,y_plot,'m-','linewidth',2);