clear('all');
close all
format compact


            temp_speed=[];
            number_of_meeting_points=length(obj.meeting_times)+1;
            temp_trajectory_x=obj.trajectory_x;
            temp_trajectory_y=obj.trajectory_y;
            stop_points=sort([0 obj.meeting_times],2);
            for i=2:number_of_meeting_points
                if i==2
                time_indexes=stop_points(i-1)+1:stop_points(i);
            else
                time_indexes=stop_points(i-1):stop_points(i);
            end
                position_matrix=[temp_trajectory_x(time_indexes)', temp_trajectory_y(time_indexes)'];
                distance_between_points = diff(position_matrix,1);
                dist_from_vertex_to_vertex = hypot(distance_between_points(:,1), distance_between_points(:,2));
                cumulative_dist_along_path = [0; cumsum(dist_from_vertex_to_vertex,1)];
                if stop_points(i-1)==0
                    speed=cumulative_dist_along_path(stop_points(2))/stop_points(2);
                    trajectory_indexes_to_fill=1:stop_points(2);
                else
                    speed=cumulative_dist_along_path(stop_points(i-1):stop_points(i))/(stop_points(i)-stop_points(i-1)-obj.charging_period_time)
                    trajectory_indexes_to_fill=(stop_points(i-1)+obj.charging_period_time+1):stop_points(i);
                end
                temp_speed=[temp_speed speed]
                
                dist_steps = speed:speed:cumulative_dist_along_path(end);



                new_points = interp1(cumulative_dist_along_path, position_matrix, dist_steps);
                obj.trajectory_x(trajectory_indexes_to_fill)=new_points(:,1)';
                obj.trajectory_y(trajectory_indexes_to_fill)=new_points(:,2)';
                trajectory_indexes_to_fill=trajectory_indexes_to_fill(end)+(1:obj.charging_period_time);
                obj.trajectory_x(trajectory_indexes_to_fill)=new_points(end,1)*ones(1,obj.charging_period_time);
                obj.trajectory_y(trajectory_indexes_to_fill)=new_points(end,2)*ones(1,obj.charging_period_time);
            end
            % figure()
            % plot(temp_speed)
            obj.trajectory_x=[obj.trajectory_x obj.trajectory_x(end)*[1:(obj.simulation_time-length(obj.trajectory_x))]];
            obj.trajectory_y=[obj.trajectory_y obj.trajectory_y(end)*[1:(obj.simulation_time-length(obj.trajectory_y))]];
