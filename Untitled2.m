aa=sim.list_of_operating_robots(1).charging(1,:);
bb=sim.list_of_operating_robots(1).charging(2,:);
plot(aa,bb)

for i=1:length(aa)
    hold on
handle=plot(aa(i),bb(i),'.','MarkerSize',40)
i
pause (0.1)
delete(handle)
end

% obj=sim;
%             
%             index=find(obj.meeting_time==0);
% 
%             hold on
%             for i=1:length(index)
%                 if i==length(index)
%                     plot(obj.chargingTrajectory_x(index(i):end),obj.chargingTrajectory_y(index(i):end),'--')
%                 else
%                     plot(obj.chargingTrajectory_x(index(i):index(i+1)-1),obj.chargingTrajectory_y(index(i):index(i+1)-1),'--')
%                 end
%             end