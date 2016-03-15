classdef Simulation < handle
    
    properties
        analyze
        time_step
        area=[66 10]
        simulation_time
        total_time
        FigHandle
        charging_time
        meeting_time
        chargingTrajectory_x
        chargingTrajectory_y
        list_of_operating_robots
        list_of_charging_robots
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        function obj=Simulation()
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setTimeStep(obj,time_step)
            obj.time_step=time_step;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setImageScale(obj,scale_coef)
            for i=1:length(obj.list_of_charging_robots)
                obj.list_of_charging_robots(i).setImageScale(scale_coef);
            end
            for i=1:length(obj.list_of_operating_robots)
                obj.list_of_operating_robots(i).setImageScale(scale_coef);
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setChargingTime(obj,charging_time)
            obj.charging_time=charging_time;
            for i=1:length(obj.list_of_charging_robots)
                obj.list_of_charging_robots(i).charging_period_time=charging_time;
            end
            for i=1:length(obj.list_of_operating_robots)
                obj.list_of_operating_robots(i).charging_period_time=charging_time;
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function addOperatingRobots(obj, number_of_robots_to_add)
            if nargin==1
                number_of_robots_to_add=1;
            end
            for i=1:number_of_robots_to_add
                obj.list_of_operating_robots=[obj.list_of_operating_robots, OperatingRobot()];
                obj.list_of_operating_robots(end).setID(length(obj.list_of_operating_robots));
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function addChargingRobots(obj, number_of_robots_to_add)
            if nargin==1
                number_of_robots_to_add=1;
            end
            for i=1:number_of_robots_to_add
                obj.list_of_charging_robots=[obj.list_of_charging_robots, ChargingRobot()];
                obj.list_of_charging_robots(end).setID(length(obj.list_of_charging_robots));
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function setSimulationTime(obj, number_of_steps)
            obj.simulation_time=number_of_steps;
            
            number_of_operating_robots=length(obj.list_of_operating_robots);
            for i=1:number_of_operating_robots
                obj.list_of_operating_robots(i).setSimulationTime(number_of_steps);
            end
            number_of_charging_robots=length(obj.list_of_charging_robots);
            for i=1:number_of_charging_robots
                obj.list_of_charging_robots(i).setSimulationTime(number_of_steps);
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function error=plan(obj,SOLVER,goal)
            % for i=1:length(obj.list_of_operating_robots)
            %     obj.list_of_operating_robots(i).segmentTrajectory();
            % end

            error=0;
            %x and set
            set=[];
            ChargingRobotx=[];
            iter=0;
            for i=1:length(obj.list_of_charging_robots)
                temp=[obj.list_of_charging_robots(i).initial_x;obj.list_of_charging_robots(i).initial_y];
                ChargingRobotx=cat(2,ChargingRobotx,temp);
                set=cat(1,set,i);
                iter=iter+1;
            end
            ChargingRobotx=ChargingRobotx';
            OperatingRobotx=[];
            OperatingRobotsets=zeros(length(obj.list_of_operating_robots),10);
            tempSet=[];
            time=zeros(length(obj.list_of_charging_robots),1);
            for i=1:length(obj.list_of_operating_robots)
                OperatingRobotx=cat(2,OperatingRobotx,obj.list_of_operating_robots(i).charging);
                time=cat(1,time,obj.list_of_operating_robots(i).charging(3,:)');
                numbeer_of_charging_windows = size(obj.list_of_operating_robots(i).charging,2);
                temp=[];
                for j=1:numbeer_of_charging_windows
                    iter=iter+1;
                    temp=[temp iter];
                    
                    tempSet=cat(1,tempSet,iter);
                end
                OperatingRobotsets(i,1:length(temp))=temp;
            end
            OperatingRobotx=[OperatingRobotx(1,:)' OperatingRobotx(2,:)'];
            
            x=cat(1,ChargingRobotx,OperatingRobotx);
            set=cat(1,set,tempSet);
            
            numCities = max(set);
            xSorted = [];
            setSorted = [];
            timeSorted=[];
            
            for i = 1:numCities
                currentSet = find(set == i);
                
                for j = 1:length(currentSet)
                    setSorted = [setSorted; i];
                    xSorted = [xSorted; x(currentSet(j),:)];
                    timeSorted=[timeSorted;time(currentSet(j))];
                end
            end
            obj.analyze=xSorted ;
            obj.analyze(:,3)=setSorted ;
            obj.analyze(:,4)=timeSorted;
            numElements = length(setSorted);
            A = zeros(numElements, numElements);
            
            if (strcmp(goal,'Distance'))
                
                for i = 1:numElements
                    
                    for j = 1:numElements
                        
                        time_matrix(i,j) = timeSorted(j) - timeSorted(i);
                        vertices_distance = norm(xSorted(i,:) - xSorted(j,:));
                        
                        if (vertices_distance/obj.list_of_charging_robots(1).max_speed+obj.charging_time<=time_matrix(i,j))
                            A(i,j)=vertices_distance;
                        else
                            A(i,j)=nan;
                        end
                        
                    end
                end

            elseif (strcmp(goal,'Time'))
                
                %--------------------------------------------------------------------------
                
                for i = 1:numElements
                    
                    for j = 1:numElements
                        
                        vertices_distance = norm(xSorted(i,:) - xSorted(j,:));
                        if (vertices_distance/obj.list_of_chargingRobots(1).max_speed<=timeSorted(i,:) - timeSorted(j,:))  %Need fix
                            A(i,j)=timeSorted(i,:) - timeSorted(j,:);
                        else
                            A(i,j)=nan;
                        end
                        
                    end
                end
                
            else disp('Unknown Objective');
                
            end
            
            % Transform GTSP to ATSP
            
            [atspMatrix infcost] = gtsp_to_atsp(A, setSorted);
            
            index=atspMatrix~=atspMatrix;
            
            for i=1:length(atspMatrix)
                for j=1:length(atspMatrix)
                    if index(i,j)==1
                        
                        atspMatrix(i,j)=9999999;
                    end
                end
            end
            for i=1:length(obj.list_of_charging_robots)
                
                atspMatrix(:,i)=0;
            end
            
            % Transform ATSP to TSP if needed
            
            if (strcmp(SOLVER,'LinKern'))
                symtsp = atsp_to_tsp(atspMatrix, infcost);
            elseif (strcmp(SOLVER,'LKH'))
                symtsp = atspMatrix;
                %symtsp = transformATSP2TSP(atspMatrix, infcost);
            else
                disp('Unknown Solver');
            end
            
            % Solver TSP using SOLVER
            vertexSequence = [];
            
            if (strcmp(SOLVER,'LinKern'))
                vertexSequenceOrdered = get_linkern_result(symtsp,setSorted);
            elseif (strcmp(SOLVER,'LKH'))
                vertexSequenceOrdered = get_lkh_result(symtsp, setSorted);
            else
                disp(' Cannot solve using unknown solver ');
            end
            
            
            
            %% Prune ATSP solution to the final GTSP solution
            
            vertexSequenceFinal = [];
            currentVertexIdx = 1;
            setsFound = [];
            
            %while(setsFound < numCities)
            while(currentVertexIdx <= length(setSorted))
                
                currentVertex = vertexSequenceOrdered(currentVertexIdx);
                vertexSequenceFinal = [vertexSequenceFinal currentVertex];
                thisSet = setSorted(currentVertex);
                setsFound = [setsFound; thisSet];
                [r c]=find(OperatingRobotsets == thisSet);
                if (isempty (r))
                    indexes = find(setSorted == thisSet);
                    currentVertexIdx = currentVertexIdx + (length(indexes));
                else
                    obj.list_of_operating_robots(r).meeting_times=[obj.list_of_operating_robots(r).meeting_times   timeSorted(currentVertex)];
                    indexes = find(setSorted == thisSet);
                    currentVertexIdx = currentVertexIdx + (length(indexes));
                end
            end
            
            
            % Solver Analysis
            setsNotFound=[];
            for i=1:numCities
                
                if (isempty(find(setsFound == i)))
                    
                    setsNotFound = [setsNotFound i];
                end
            end
            if (strcmp(goal,'Distance'))
                obj.meeting_time=timeSorted(vertexSequenceFinal);
                
                %                             judge=[];
                %                             for i=1:length(meet)-1
                %                                 judge(i)=max(meet(i:i+1))~=meet(i+1);
                %                             end
                %                             if sum(judge)==0;
                disp('Route found. The meeting time:')
                %                             disp((meet(2:end)))
                %                             obj.meeting_times=meet;
                %                             else disp(meet)
                %                                 disp('Mission impossible. Please speed up or assign more chargers.')
                %                                 error=1;
                %                             end
                disp(obj.meeting_time)
                
                obj.chargingTrajectory_x= xSorted(vertexSequenceFinal,1);
                obj.chargingTrajectory_y= xSorted(vertexSequenceFinal,2);
            elseif (strcmp(goal,'Time'))
                
                
                meet=timeSorted(vertexSequenceFinal);
                meet(2:4)=flip(meet(2:4));
                judge=[];
                for i=1:length(meet)-1
                    judge(i)=max(meet(i:i+1))~=meet(i+1);
                end
                if sum(judge)==0;
                    disp('Route found. The meeting time:');
                    disp((meet(2:end)));
                    obj.meeting_times=meet;
                else
                    disp((meet(2:end)));
                    disp('Mission impossibe. Please speed up or assign more chargers.');
                    error=1;
                end
                
                
                obj.chargingTrajectory_x= xSorted(vertexSequenceFinal,1);
                obj.chargingTrajectory_y= xSorted(vertexSequenceFinal,2);
                
            else
                disp(' Cannot get result ');
            end
            
            index=find(obj.meeting_time==0);
            
            for i=1:length(index)
                if i==length(index)
                    obj.list_of_charging_robots(i).path_x=obj.chargingTrajectory_x(index(i):end)';
                    obj.list_of_charging_robots(i).path_y=obj.chargingTrajectory_y(index(i):end)';
                    obj.list_of_charging_robots(i).meeting_times=obj.meeting_time(index(i):end)';
                else
                    obj.list_of_charging_robots(i).path_x=obj.chargingTrajectory_x(index(i):index(i+1)-1)';
                    obj.list_of_charging_robots(i).path_y=obj.chargingTrajectory_y(index(i):index(i+1)-1)';
                    obj.list_of_charging_robots(i).meeting_times=obj.meeting_time(index(i):index(i+1)-1)';
                end
            end

            % for i=1:length(obj.list_of_charging_robots)
            %     obj.list_of_charging_robots(i).segmentTrajectory();
            % end

            % for i=1:length(obj.list_of_operating_robots)
            %     obj.list_of_operating_robots(i).finalizeTrajectory()
            % end

        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function plot(obj,time)
            number_of_operating_robots=length(obj.list_of_operating_robots);
            switch nargin
                case 1
                    % close all
                    % x=120;
                    % axis([-0 27 -6 21]);
                       % axis([-145.2409  126.7725 -125.3140  120.2255]); uncomment for circle
                    obj.FigHandle=figure(1);
                    axis equal
                    % set(obj.FigHandle, 'Position', [100, 100, 1049, 895]);

                    for i=1:number_of_operating_robots
                        if~(obj.list_of_operating_robots(i).hasTrajectory())
                            obj.list_of_operating_robots(i).setTrajectoryBarzinEdition(obj.area(1)*i/number_of_operating_robots,0);
                        end
                        obj.list_of_operating_robots(i).plot();
                    end
                    
                    for i=1:length(obj.list_of_charging_robots)
                        obj.list_of_charging_robots(i).plot();
                    end
                    % legend([plot(NaN,NaN,'b','LineWidth',3),plot(NaN,NaN,'r','LineWidth',3),plot(NaN,NaN,'k--','LineWidth',2)],'Operating Robot Trajectory','Operating Robot In Low Battery Level','Charging Robot Trajectory');
                case 2
                    
                    for i=1:number_of_operating_robots
                        obj.list_of_operating_robots(i).plot(time);
                    end
                    
                    for i=1:length(obj.list_of_charging_robots);
                         obj.list_of_charging_robots(i).plot(time);
                    end
                    % if (isempty (find(time==[1 2 0:obj.simulation_time/10:obj.simulation_time])));
                    %     time;
                    % else
                        
                    %     saveas(obj.FigHandle,sprintf('FIG%d.png',time));
                    % end
            end

        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function simStep(obj)
            for i=1:length(obj.list_of_operatoring_robots)
                obj.list_of_operatoring_robots(i).move(obj.time_step);
            end
            for i=1:length(obj.list_of_charging_robots)
                obj.list_of_charging_robots(i).move(obj.time_step);
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function simulate(obj,record)
            switch nargin
            case 1
                for i=1:obj.simulation_time*1.2
                    clc
                    i
                    obj.plot(i);
                    drawnow;
                    pause(.02);
                end
            case 2
                if (any(strcmp(record,{'Record','record','Rec','rec'})))
                    
                    c=fix(clock);
                    cstr=strcat(mat2str(c),'.avi');
                    SimulationVideo = VideoWriter(cstr);
                    SimulationVideo.FrameRate = 5; % Barzin tried to slow the video 2/4/2016
                    open(SimulationVideo);
                    
                    set(gca,'nextplot','replacechildren');
                    set(gcf,'Renderer','zbuffer');
                    obj.plot();
                    axis([-0 27 -6 21]);
                    % axis([-123.9075  130.7855 -103.0364   113.1102]); uncomment for circle
                    for i=1:obj.simulation_time
                        obj.plot(i);
                        drawnow;
                        frame = getframe;
                        writeVideo(SimulationVideo,frame)
                    end
                    close(SimulationVideo);
                else
                    disp('Unknown command.');
                end
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end % methods
    
end % classdef