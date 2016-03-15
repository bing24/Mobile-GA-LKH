clear all
close all

mmm = 0;
% for nnn = [1 1 1 1 1 2 2 2 2 2 3 3 3 3 3 4 4 4 4 4 5 5 5 5 5]
 for nnn = [3]
mmm = mmm+1;
tic;
map = Map('map_image.bmp','resolution',20,'hieght',200);

% map.show('border')
represent(map)
userConfig.xy=map.mission_location;
userConfig.minTour=floor(size(userConfig.xy,1)/nnn);
userConfig.popSize=80;
userConfig.nSalesmen= nnn;
userConfig.batteryLife = 5;
% a = opti_stations(userConfig);
length_charging = 3;
a = position_ga(userConfig);
toc
hold on 
contour(map.matrix,1,'black','linewidth',5)


route = a.optRoute;
breaks = a.optBreak;
N = length(route);
rng = [[1 breaks+1];[breaks N]]';
minTime = 15;
indBreaks = a.optRoute(a.optBreak);
ends = [a.xy(a.optStations,1),a.xy(a.optStations,2)];

end

%% Add working robots and charging robots 
sim=Simulation();

sim.addOperatingRobots(nnn);
sim.addChargingRobots(nnn);
sim.setImageScale(.01);
sim.setSimulationTime(400); % switch to 200 for circle scenario

% Give working robots trajectories

for s = 1:a.nSalesmen
	rte = route(rng(s,1):rng(s,2));
	sim.list_of_operating_robots(s).trajectory_x = a.xy(rte,1);
	sim.list_of_operating_robots(s).trajectory_y = a.xy(rte,2);
end

%% Set initial positions and speed of charging robots using ChargingRobot.m 

setpos(sim.list_of_charging_robots(1),2,2);
      setspeed(sim.list_of_charging_robots(1),10);
setpos(sim.list_of_charging_robots(2),6,2);
      setspeed(sim.list_of_charging_robots(2),10);
setpos(sim.list_of_charging_robots(3),13,2);
      setspeed(sim.list_of_charging_robots(3),10);

setTimeStep(sim,1);
setChargingTime(sim,length_charging); %switch to 15 for circle
aw=sim.list_of_operating_robots(1);
% Calculate charging positions time
% Calculate length of each route
lbreaks=diff([1 breaks N]);

nStations(1)= floor(lbreaks(1)/userConfig.batteryLife-0.000001);
charging_period = [0 ones(1,nStations(1)-1)*length_charging];
charging_period = cumsum(charging_period);
templm=ones(1,nStations(1))*userConfig.batteryLife;
if isempty(templm) == 1
    indexStations=[];
else
    templm(1)=0+userConfig.batteryLife;
    time = cumsum(templm)+charging_period;
    
end
for i = 2 : length(lbreaks)
    % count how many stations for each salesmen
    
    nStations(i)= floor(lbreaks(i)/userConfig.batteryLife-0.000001);
    charging_period = [0 ones(1,nStations(i)-1)*length_charging];
	% charging_period = cumsum(charging_period);
    templm=ones(1,nStations(i))*userConfig.batteryLife+ charging_period;
    if isempty(templm) == 1
        time = time;
    else
        time = [time cumsum(templm)] ;
                               
    end
end

% Add "charging" to each operating robot
tem = cumsum(nStations);
sim.list_of_operating_robots(1).charging(1,:) = ends(1:tem(1),1);
sim.list_of_operating_robots(1).charging(2,:) = ends(1:tem(1),2);
sim.list_of_operating_robots(1).charging(3,:) = time(1:tem(1));
for s = 2:a.nSalesmen
	sim.list_of_operating_robots(s).charging(1,:) = ends(tem(s-1)+1:tem(s),1);
	sim.list_of_operating_robots(s).charging(2,:) = ends(tem(s-1)+1:tem(s),2);
	sim.list_of_operating_robots(s).charging(3,:) = time(tem(s-1)+1:tem(s));
end


plan(sim,'LKH','Distance');


sim.plot()
% set(gca,'XTick',[])
% set(gca,'YTick',[])
 % pause (3)
 % sim.simulate()