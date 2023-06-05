%% animation
figure(h1);
if animation_f
    t = t_init;
    cumulation = 1;
    while t <= t_total
        subplot(2,3,1);
        for i = 1:size(agents,1)
            delete(agents(i).figure_handle_uncertainty);
            switch agents(i).type
                case 1
                    agents(i).figure_handle_uncertainty = uav(agents(i).x(cumulation), agents(i).y(cumulation), agents(i).z(cumulation),...
                             'roll', rad2deg(agents(i).roll(cumulation)), 'pitch', rad2deg(agents(i).pitch(cumulation)),...
                             'yaw', rad2deg(agents(i).yaw(cumulation)), 'scale', aircraft_scale, 'color', color_pallete(i),...
                             'wing', color_pallete(i), 'linestyle', 'none');
                case 2
                    agents(i).figure_handle_uncertainty = quadrotor(agents(i).x(cumulation), agents(i).y(cumulation), agents(i).z(cumulation),...
                             'roll', rad2deg(agents(i).roll(cumulation)), 'pitch', rad2deg(agents(i).pitch(cumulation)),...
                             'yaw', rad2deg(agents(i).yaw(cumulation)), 'scale', multirotor_scale, 'body', color_pallete(i),...
                             'boom', color_pallete(i), 'prop', color_pallete(i), 'linestyle', 'none');
                otherwise
                    agents(i).figure_handle_uncertainty = uav(agents(i).x(cumulation), agents(i).y(cumulation), agents(i).z(cumulation),...
                             'roll', rad2deg(agents(i).roll(cumulation)), 'pitch', rad2deg(agents(i).pitch(cumulation)),...
                             'yaw', rad2deg(agents(i).yaw(cumulation)), 'scale', aircraft_scale, 'color', color_pallete(i),...
                             'wing', color_pallete(i), 'linestyle', 'none');
            end
        end
        for i = 1:size(targets,1)
            delete(targets(i).figure_handle_uncertainty);
            targets(i).figure_handle_uncertainty = birds(targets(i).x(cumulation), targets(i).y(cumulation), targets(i).z(cumulation), ...
                        'body', color_pallete(i), 'yaw', rad2deg(targets(i).heading(cumulation)), 'scale', target_scale);
        end
        delete(uncertainty_map_handle);
        uncertainty_map_handle = surf(XX,YY,uncertainty_global(:,:,cumulation), 'facealpha', 1, 'linestyle', 'none');
        title(['Uncertainty map - time=',num2str(t),'s']);
        
        subplot(2,3,2);
        for i = 1:size(agents,1)
            delete(agents(i).figure_handle_probability);
            switch agents(i).type
                case 1
                    agents(i).figure_handle_probability = uav(agents(i).x(cumulation), agents(i).y(cumulation), agents(i).z(cumulation),...
                             'roll', rad2deg(agents(i).roll(cumulation)), 'pitch', rad2deg(agents(i).pitch(cumulation)),...
                             'yaw', rad2deg(agents(i).yaw(cumulation)), 'scale', aircraft_scale, 'color', color_pallete(i),...
                             'wing', color_pallete(i), 'linestyle', 'none');
                case 2
                    agents(i).figure_handle_probability = quadrotor(agents(i).x(cumulation), agents(i).y(cumulation), agents(i).z(cumulation),...
                             'roll', rad2deg(agents(i).roll(cumulation)), 'pitch', rad2deg(agents(i).pitch(cumulation)),...
                             'yaw', rad2deg(agents(i).yaw(cumulation)), 'scale', multirotor_scale, 'body', color_pallete(i),...
                             'boom', color_pallete(i), 'prop', color_pallete(i), 'linestyle', 'none');
                otherwise
                    agents(i).figure_handle_probability = uav(agents(i).x(cumulation), agents(i).y(cumulation), agents(i).z(cumulation),...
                             'roll', rad2deg(agents(i).roll(cumulation)), 'pitch', rad2deg(agents(i).pitch(cumulation)),...
                             'yaw', rad2deg(agents(i).yaw(cumulation)), 'scale', aircraft_scale, 'color', color_pallete(i),...
                             'wing', color_pallete(i), 'linestyle', 'none');
            end
        end
        for i = 1:size(targets,1)
            delete(targets(i).figure_handle_probability);
            targets(i).figure_handle_probability = birds(targets(i).x(cumulation), targets(i).y(cumulation), targets(i).z(cumulation), ...
                        'body', color_pallete(i), 'yaw', rad2deg(targets(i).heading(cumulation)), 'scale', target_scale);
        end
        delete(probability_map_handle);
        probability_map_handle = surf(XX,YY,probability_global(:,:,cumulation), 'facealpha', 1, 'linestyle', 'none');
        title(['Probability map - time=',num2str(t),'s']);
        
        subplot(2,3,3);
        for i = 1:size(agents,1)
            delete(agents(i).figure_handle_interests);
            switch agents(i).type
                case 1
                    agents(i).figure_handle_interests = uav(agents(i).x(cumulation), agents(i).y(cumulation), agents(i).z(cumulation),...
                             'roll', rad2deg(agents(i).roll(cumulation)), 'pitch', rad2deg(agents(i).pitch(cumulation)),...
                             'yaw', rad2deg(agents(i).yaw(cumulation)), 'scale', aircraft_scale, 'color', color_pallete(i),...
                             'wing', color_pallete(i), 'linestyle', 'none');
                case 2
                    agents(i).figure_handle_interests = quadrotor(agents(i).x(cumulation), agents(i).y(cumulation), agents(i).z(cumulation),...
                             'roll', rad2deg(agents(i).roll(cumulation)), 'pitch', rad2deg(agents(i).pitch(cumulation)),...
                             'yaw', rad2deg(agents(i).yaw(cumulation)), 'scale', multirotor_scale, 'body', color_pallete(i),...
                             'boom', color_pallete(i), 'prop', color_pallete(i), 'linestyle', 'none');
                otherwise
                    agents(i).figure_handle_interests = uav(agents(i).x(cumulation), agents(i).y(cumulation), agents(i).z(cumulation),...
                             'roll', rad2deg(agents(i).roll(cumulation)), 'pitch', rad2deg(agents(i).pitch(cumulation)),...
                             'yaw', rad2deg(agents(i).yaw(cumulation)), 'scale', aircraft_scale, 'color', color_pallete(i),...
                             'wing', color_pallete(i), 'linestyle', 'none');
            end
        end
        for i = 1:size(targets,1)
            delete(targets(i).figure_handle_interests);
            targets(i).figure_handle_interests = birds(targets(i).x(cumulation), targets(i).y(cumulation), targets(i).z(cumulation), ...
                        'body', color_pallete(i), 'yaw', rad2deg(targets(i).heading(cumulation)), 'scale', target_scale);
            delete(targets(i).figure_handle_waypoint);
            targets(i).figure_handle_waypoint = plot3(targets(i).x_setpoint(cumulation), targets(i).y_setpoint(cumulation), targets(i).z(cumulation), ...
                        'o', 'markersize', 10, 'color', color_pallete(i));
        end
        delete(interests_map_handle);
        interests_map_handle = surf(XX,YY,interests_global(:,:,cumulation), 'facealpha', 1, 'linestyle', 'none');
        title(['Interests map - time=',num2str(t*dt),'s']);
        
        subplot(2,3,4);
        plot(cumulation, sum(sum(uncertainty_global(:,:,cumulation)))/number_of_tiles_sum,'--or','markersize',10,'linewidth',2);
        
        subplot(2,3,5);
        plot(cumulation, sum(sum(probability_global(:,:,cumulation)))/number_of_tiles_sum,'--ob','markersize',10,'linewidth',2);

        drawnow;
        t = t+dt;
        cumulation = cumulation+1;
    end
end