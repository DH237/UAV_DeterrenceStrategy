function [handle] = drawRectangle(centre, edge_length_x, edge_length_y)

x                   = centre(1);
y                   = centre(2);

x_bottom_l          = x-edge_length_x/2;
y_bottom_l          = y-edge_length_y/2;

x_bottom_r          = x+edge_length_x/2;
y_bottom_r          = y-edge_length_y/2;

x_top_r             = x+edge_length_x/2;
y_top_r             = y+edge_length_y/2;

x_top_l             = x-edge_length_x/2;
y_top_l             = y+edge_length_y/2;

hold on;
plot([x_bottom_l,x_bottom_r],[y_bottom_l,y_bottom_r],'m-','linewidth',2);
plot([x_bottom_r,x_top_r],[y_bottom_r,y_top_r],'m-','linewidth',2);
plot([x_top_r,x_top_l],[y_top_r,y_top_l],'m-','linewidth',2);
handle = plot([x_top_l,x_bottom_l],[y_top_l,y_bottom_l],'m-','linewidth',2);