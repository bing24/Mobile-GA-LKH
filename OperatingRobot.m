classdef OperatingRobot <handle

    properties
        id
        simulation_time
        initial_x=0
        initial_y=0
        current_x
        current_y
        path_x
        path_y
        trajectory_x
        trajectory_y
        all_trajectory_x
        all_trajectory_y
        dist_can_reach
        position_can_reach
        trajectory_power
        battery_life=50 % in number of timesteps switch to 50 for circle
        charging_period_time
        max_speed=.2 % how to difine? switch to 2 for the circle scenario
        power_level
        battery_drain_rate
        recharge_window_max_level
        recharge_window_min_level
        charging
        figure_handle=[]
        scale_coef
        temp
        alert_level=.4;
        critical_level=0.1;
        meeting_times=[];
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods (Access=public)
        function obj=OperatingRobot()
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setImageScale(obj,scale_coef)
            obj.scale_coef=scale_coef;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setID(obj, ID_number)
            obj.id=ID_number;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setSimulationTime(obj, simulation_time)
            obj.simulation_time=simulation_time;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function has_trajectory = hasTrajectory(obj)
            has_trajectory = ~isempty(obj.trajectory_x)&~isempty(obj.trajectory_y);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setTrajectoryBarzinEdition(obj,position_x,position_y,position_description)
            
            radius=1.5;
            switch nargin
            case 1
                if (~isempty(obj.initial_x)&&~isempty(obj.initial_y))
                    obj.randomTrajectory(radius);
                    obj.trajectory_x=obj.trajectory_x+obj.initial_x;
                    obj.trajectory_y=obj.trajectory_y+obj.initial_y;
                else
                    disp('The function needs more inputs or the initial x and y should be set.');
                end
            case 3
                obj.randomTrajectory(radius);
                obj.trajectory_x=obj.trajectory_x+position_x;
                obj.trajectory_y=obj.trajectory_y+position_y;
            case 4
                if (any(strcmp(position_description,{'Center','center'})))
                    obj.randomTrajectory(radius);
                    obj.trajectory_x=obj.trajectory_x+position_x;
                    obj.trajectory_y=obj.trajectory_y+position_y;
                elseif (any(strcmp(position_description,{'Initial','initial'})))
                    obj.randomTrajectory(radius);
                    [obj.trajectory_x,obj.trajectory_y]=[trajectory_x,trajectory_y]-[trajectory_x(1),trajectory_y(2)]+[position_x,position_y];
                elseif (any(strcmp(position_description,{'Set','set'})))
                    obj.trajectory_x=position_x;
                    obj.trajectory_y=position_y;
                else
                    disp('The "position description" is unknown. Use either "center" or "initial"')
                end
            % default
            %     disp('Number of input arguments is not acceptable.')
            end
            obj.segmentTrajectory();
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function finalizeTrajectory(obj)
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
                    speed=cumulative_dist_along_path(stop_points(i)-stop_points(i-1))/(stop_points(i)-stop_points(i-1)-obj.charging_period_time);
                    trajectory_indexes_to_fill=(stop_points(i-1)+obj.charging_period_time+1):stop_points(i);
                end
                temp_speed=[temp_speed speed];
                
                dist_steps = speed:speed:cumulative_dist_along_path(end);



                new_points = interp1(cumulative_dist_along_path, position_matrix, dist_steps);

                % new_points=position_matrix;
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

        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function plot(obj,time)
            switch nargin
            case 1
                hold on             
                charged=(obj.power_level>=obj.alert_level);
                critical=(obj.power_level<obj.critical_level);
                alert=(obj.power_level<obj.alert_level & obj.power_level>=obj.critical_level);
                % laps=length(find(obj.power_level==1));

                for i=1:length(find(obj.power_level==1))
                    temp_charged=find(charged);
                    temp_charged=temp_charged(find(temp_charged<=obj.battery_life*i & temp_charged>obj.battery_life*(i-1)));
                    indeces_to_plot=[temp_charged min(max(temp_charged)+1,obj.simulation_time)];
                    plot(obj.trajectory_x(indeces_to_plot),obj.trajectory_y(indeces_to_plot),'b','LineWidth',3);
                    
                    temp_alert=find(alert);
                    temp_alert=temp_alert(find(temp_alert<=obj.battery_life*i & temp_alert>obj.battery_life*(i-1)));
                    indeces_to_plot=temp_alert;
                    plot(obj.trajectory_x(indeces_to_plot),obj.trajectory_y(indeces_to_plot),'b','LineWidth',3);
                    
                    temp_critical=find(critical);
                    temp_critical=temp_critical(find(temp_critical<=obj.battery_life*i & temp_critical>obj.battery_life*(i-1)));
                    indeces_to_plot=[min(temp_critical)-1 temp_critical min(max(temp_critical)+1,obj.simulation_time)];
                    plot(obj.trajectory_x(indeces_to_plot),obj.trajectory_y(indeces_to_plot),'b','LineWidth',3); 
                    % set(handle_of_sim,'XTickLabel',[])

                end
            case 2
                if ishandle(obj.figure_handle) delete(obj.figure_handle);end
                [workerimage,~,alpha]=imread('or.png');
                if time==1
                    p1=[obj.trajectory_x(time+1) obj.trajectory_y(time+1)];
                else
                    p1=[obj.trajectory_x(time-1) obj.trajectory_y(time-1)];
                end
                p2=[obj.trajectory_x(time) obj.trajectory_y(time)];
                sita = atan2(p2(2)-p1(2),p2(1)-p1(1))*180/pi+3;
                if isnan(sita)
                    sita=0;
                end
                workerrotate=imrotate(workerimage,-sita);
                alpharotate=imrotate(alpha,-sita);
                
                x_scale=obj.scale_coef*[-1 1]*size(workerrotate,2)+obj.trajectory_x(time);
                y_scale=obj.scale_coef*[-1 1]*size(workerrotate,1)+obj.trajectory_y(time);
                
                obj.figure_handle=image(workerrotate,'XData',x_scale,'YData',y_scale);
                set(obj.figure_handle,'AlphaData',alpharotate);
                % plot(obj.trajectory_x(time),obj.trajectory_y(time),'x')
               
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setspeed(obj,speed)
            obj.max_speed=speed;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function segmentTrajectory(obj)
            obj.charging=[];
            position_matrix = [obj.trajectory_x' obj.trajectory_y'];
            distance_between_points = diff(position_matrix,1);
            dist_from_vertex_to_vertex = hypot(distance_between_points(:,1), distance_between_points(:,2));
            cumulative_dist_along_path = [0; cumsum(dist_from_vertex_to_vertex,1)];
            dist_steps = 0:obj.max_speed:obj.max_speed*obj.simulation_time; %linspace(0, travel_length, number_of_timesteps);

            if dist_steps(end)>cumulative_dist_along_path(end)
                error('error working robot speed is too fast')

            else
                iter=1;
                dist_can_reach=[];
                position_can_reach=[];
                while dist_steps(end)-cumulative_dist_along_path(iter)>= 0
                    dist_can_reach(iter)=cumulative_dist_along_path(iter);
                    position_can_reach(iter,:)=position_matrix(iter,:);

                iter=iter+1;
            end
            dist_can_reach=[dist_can_reach cumulative_dist_along_path(iter)];
            position_can_reach=[position_can_reach; position_matrix(iter,:)];
            end
         

            new_points = interp1(dist_can_reach, position_can_reach, dist_steps);
            obj.trajectory_x=new_points(:,1)';
            obj.trajectory_y=new_points(:,2)';
            obj.power_level=1-mod([0:obj.simulation_time],obj.battery_life)/obj.battery_life;
            alert=(obj.power_level<obj.alert_level & obj.power_level>=obj.critical_level);
            for i=1:length(find(obj.power_level==1)) % for each charging window that exists
                temp_alert=find(alert);
                temp_alert=temp_alert(find(temp_alert<=obj.battery_life*i & temp_alert>obj.battery_life*(i-1)));
                indeces_to_plot=temp_alert;
                obj.charging=[obj.charging [obj.trajectory_x(indeces_to_plot); obj.trajectory_y(indeces_to_plot); indeces_to_plot]]; % TODO: put into seperate function
            end
        end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        function randomTrajectory(obj,radius)
            laps=obj.simulation_time*obj.max_speed/radius/pi/2;
            twoPI=2*pi;
            number_of_trajectory_steps=obj.simulation_time*5; % do unit test
            number_of_divergences=round(laps*15); % do unit test
            divergence_range=.02;
            divergence_steps=linspace(0,2*pi*laps,number_of_divergences);
            discrete_steps=linspace(0,2*pi*laps,number_of_trajectory_steps);
            random_values=radius*divergence_range*randn([1 number_of_divergences]);
            fitted_values=radius+spline(divergence_steps,[0 random_values 0],discrete_steps);
            random_theta=rand*2*pi;
            direction=sign(rand-.5);
            x_array=fitted_values.*cos(direction*discrete_steps+random_theta);
            y_array=fitted_values.*sin(direction*discrete_steps+random_theta);
            radius_increase=(linspace(1,laps*radius,length(x_array)));
            obj.trajectory_x=radius_increase.*x_array;
            obj.trajectory_y=radius_increase.*y_array;
        end
        function second_trajectory(obj)
            travel_length=2*pi;
                number_of_trajectory_steps=100;
                number_of_divergences=15;
                radius=5+rand;
                divergence_range=1;
                divergence_steps=linspace(0,2*pi,number_of_divergences);
                discrete_steps=linspace(0,2*pi,number_of_trajectory_steps);
                random_values=radius+divergence_range*randn([1 number_of_divergences]);
                random_values(end)=random_values(1);
                fitted_values=spline(divergence_steps,[0 random_values 0],discrete_steps);
                x_array=fitted_values.*cos(discrete_steps);
                y_array=fitted_values.*sin(discrete_steps);
                normalization_coef=sum(sqrt(diff(x_array).^2+diff(y_array).^2));
                
                obj.trajectory_x=travel_length/normalization_coef*x_array+center_x;
                obj.trajectory_y=travel_length/normalization_coef*y_array+center_y;

                obj.all_trajectory_x=repmat(obj.trajectory_x,1,obj.laps);
                obj.all_trajectory_y=repmat(obj.trajectory_y,1,obj.laps);
            end
    end

end
