% %% plot the global uncertainty
% h1                  = figure('Name', 'World', 'NumberTitle', 'off');
% subplot(2,3,1);
% drawBoundary;
% hold on;
% % draw aircraft and map
% for i = 1:size(agents,1)
%     switch agents(i).type
%         case 1
%             agents(i).figure_handle_uncertainty = uav(agents(i).x, agents(i).y, agents(i).z,...
%                      'roll', rad2deg(agents(i).roll), 'pitch', rad2deg(agents(i).pitch),...
%                      'yaw', rad2deg(agents(i).yaw), 'scale', aircraft_scale, 'color', color_pallete(i),...
%                      'wing', color_pallete(i), 'linestyle', 'none');
%         case 2
%             agents(i).figure_handle_uncertainty = quadrotor(agents(i).x, agents(i).y, agents(i).z,...
%                      'roll', rad2deg(agents(i).roll), 'pitch', rad2deg(agents(i).pitch),...
%                      'yaw', rad2deg(agents(i).yaw), 'scale', multirotor_scale, 'body', color_pallete(i),...
%                      'boom', color_pallete(i), 'prop', color_pallete(i), 'linestyle', 'none');
%         otherwise
%             agents(i).figure_handle_uncertainty = uav(agents(i).x, agents(i).y, agents(i).z,...
%                      'roll', rad2deg(agents(i).roll), 'pitch', rad2deg(agents(i).pitch),...
%                      'yaw', rad2deg(agents(i).yaw), 'scale', aircraft_scale, 'color', color_pallete(i),...
%                      'wing', color_pallete(i), 'linestyle', 'none');
%     end
% end
% % draw targets
% for i = 1:size(targets,1)
%     targets(i).figure_handle_uncertainty = birds(targets(i).x, targets(i).y, targets(i).z, ...
%                 'body', color_pallete(i), 'yaw', rad2deg(targets(i).heading), 'scale', target_scale);
% end
% uncertainty_map_handle = surf(world.meshgrid_XX,world.meshgrid_YY,world.uncert_global, 'facealpha', 1, 'linestyle', 'none');
% oldcolormap = colormap('gray');
% colormap( flipud(oldcolormap) );
% caxis([0 1]);
% xlabel('x'); ylabel('y'); zlabel('z');
% axis equal;
% axis([-100 world.size_x+100 -100 world.size_y+100]);
% 
% % scatter plot of envrionment uncertainty
% subplot(2,3,4);
% plot(0, sum(sum(world.uncert_global))/world.number_of_tiles,'--or','markersize',10,'linewidth',2);
% grid on;
% title('Evolution of global envrioment uncertainty');
% hold on;
% 
% %% plot the global probability
% subplot(2,3,2);
% drawBoundary;
% hold on;
% % draw aircraft and map
% for i = 1:size(agents,1)
%     switch agents(i).type
%         case 1
%             agents(i).figure_handle_probability = uav(agents(i).x, agents(i).y, agents(i).z,...
%                      'roll', rad2deg(agents(i).roll), 'pitch', rad2deg(agents(i).pitch),...
%                      'yaw', rad2deg(agents(i).yaw), 'scale', aircraft_scale, 'color', color_pallete(i),...
%                      'wing', color_pallete(i), 'linestyle', 'none');
%         case 2
%             agents(i).figure_handle_probability = quadrotor(agents(i).x, agents(i).y, agents(i).z,...
%                      'roll', rad2deg(agents(i).roll), 'pitch', rad2deg(agents(i).pitch),...
%                      'yaw', rad2deg(agents(i).yaw), 'scale', multirotor_scale, 'body', color_pallete(i),...
%                      'boom', color_pallete(i), 'prop', color_pallete(i), 'linestyle', 'none');
%         otherwise
%             agents(i).figure_handle_probability = uav(agents(i).x, agents(i).y, agents(i).z,...
%                      'roll', rad2deg(agents(i).roll), 'pitch', rad2deg(agents(i).pitch),...
%                      'yaw', rad2deg(agents(i).yaw), 'scale', aircraft_scale, 'color', color_pallete(i),...
%                      'wing', color_pallete(i), 'linestyle', 'none');
%     end
% end
% % draw targets
% for i = 1:size(targets,1)
%     targets(i).figure_handle_probability = birds(targets(i).x, targets(i).y, targets(i).z, ...
%                 'body', color_pallete(i), 'yaw', rad2deg(targets(i).heading), 'scale', target_scale);
% end
% probability_map_handle = surf(world.meshgrid_XX,world.meshgrid_YY,world.prob_global, 'facealpha', 1, 'linestyle', 'none');
% oldcolormap = colormap('gray');
% colormap( flipud(oldcolormap) );
% caxis([0 1]);
% xlabel('x'); ylabel('y'); zlabel('z');
% axis equal;
% axis([-100 world.size_x+100 -100 world.size_y+100]);
% 
% % scatter plot of target probability
% subplot(2,3,5)
% plot(0, sum(sum(world.prob_global))/world.number_of_tiles,'--ob','markersize',10,'linewidth',2);
% grid on;
% title('Evolution of global target probability');
% hold on;
% 
% %% Plot target and interests
% subplot(2,3,3);
% drawBoundary;
% hold on;
% % draw aircraft and map
% for i = 1:size(agents,1)
%     switch agents(i).type
%         case 1
%             agents(i).figure_handle_interests = uav(agents(i).x, agents(i).y, agents(i).z,...
%                      'roll', rad2deg(agents(i).roll), 'pitch', rad2deg(agents(i).pitch),...
%                      'yaw', rad2deg(agents(i).yaw), 'scale', aircraft_scale, 'color', color_pallete(i),...
%                      'wing', color_pallete(i), 'linestyle', 'none');
%         case 2
%             agents(i).figure_handle_interests = quadrotor(agents(i).x, agents(i).y, agents(i).z,...
%                      'roll', rad2deg(agents(i).roll), 'pitch', rad2deg(agents(i).pitch),...
%                      'yaw', rad2deg(agents(i).yaw), 'scale', multirotor_scale, 'body', color_pallete(i),...
%                      'boom', color_pallete(i), 'prop', color_pallete(i), 'linestyle', 'none');
%         otherwise
%             agents(i).figure_handle_interests = uav(agents(i).x, agents(i).y, agents(i).z,...
%                      'roll', rad2deg(agents(i).roll), 'pitch', rad2deg(agents(i).pitch),...
%                      'yaw', rad2deg(agents(i).yaw), 'scale', aircraft_scale, 'color', color_pallete(i),...
%                      'wing', color_pallete(i), 'linestyle', 'none');
%     end
% end
% % draw target and map
% for i = 1:size(targets,1)
%     targets(i).figure_handle_interests = birds(targets(i).x, targets(i).y, targets(i).z, ...
%                 'body', color_pallete(i), 'yaw', rad2deg(targets(i).heading), 'scale', target_scale);
% end
% interests_map_handle = surf(world.meshgrid_XX,world.meshgrid_YY,world.inter_global, 'facealpha',1,'linestyle','none');
% oldcolormap = colormap('gray');
% colormap(flipud(oldcolormap));
% caxis([0 1]);
% xlabel('x'); ylabel('y'); zlabel('z');
% axis equal;
% axis([-100 world.size_x+100 -100 world.size_y+100]);

% =======================================================================================================
% =======================================================================================================
% =======================================================================================================
%% debug figure
h2                  = figure('Name', 'Debug figure', 'NumberTitle', 'off');
% subplot(1,2,1);
drawBoundary;
hold on;
% draw aircraft and map
% for i = 1:size(agents,1)
%     switch agents(i).type
%         case 1
%             agents(i).figure_handle_debug = uav(agents(i).x, agents(i).y, agents(i).z,...
%                      'roll', rad2deg(agents(i).roll), 'pitch', rad2deg(agents(i).pitch),...
%                      'yaw', rad2deg(agents(i).yaw), 'scale', aircraft_scale, 'color', color_pallete_agent(i),...
%                      'wing', color_pallete(i), 'linestyle', 'none');
%         case 2
%             agents(i).figure_handle_debug = quadrotor(agents(i).x, agents(i).y, agents(i).z,...
%                      'roll', rad2deg(agents(i).roll), 'pitch', rad2deg(agents(i).pitch),...
%                      'yaw', rad2deg(agents(i).yaw), 'scale', multirotor_scale, 'body', color_pallete_agent(i),...
%                      'boom', color_pallete(i), 'prop', color_pallete(i), 'linestyle', 'none');
%         otherwise
%             agents(i).figure_handle_debug = uav(agents(i).x, agents(i).y, agents(i).z,...
%                      'roll', rad2deg(agents(i).roll), 'pitch', rad2deg(agents(i).pitch),...
%                      'yaw', rad2deg(agents(i).yaw), 'scale', aircraft_scale, 'color', color_pallete_agent(i),...
%                      'wing', color_pallete(i), 'linestyle', 'none');
%     end
%     % draw position setpoint
%     agents(i).figure_handle_waypoint = plot3(agents(i).x, agents(i).y, agents(i).z, ...
%                 'o', 'markersize', 10, 'color', color_pallete_agent(i));
% end
% draw target and map
for i = 1:size(targets,1)
    targets(i).figure_handle_debug = birds(targets(i).x, targets(i).y, targets(i).z, ...
                'body', color_pallete(i), 'yaw', rad2deg(targets(i).heading), 'scale', target_scale);
    % draw position setpoint
    targets(i).figure_handle_waypoint = plot3(targets(i).x, targets(i).y, targets(i).z, ...
                '^', 'markersize', 10, 'color', color_pallete(i));
end
debug_map_handle = surf(world.meshgrid_XX,world.meshgrid_YY,world.inter_global, 'facealpha',1,'linestyle','none');
oldcolormap = colormap('gray');
colormap(flipud(oldcolormap));
caxis([0 1]);
xlabel('x (m)','FontSize',20,'FontName','Arial','FontWeight','Bold');
ylabel('y (m)','FontSize',20,'FontName','Arial','FontWeight','Bold');
zlabel('z (m)','FontSize',20,'FontName','Arial','FontWeight','Bold');
axis equal;
axis([-100 world.size_x+100 -100 world.size_y+100]);