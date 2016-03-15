% barzintest2.m
close all
clear all
clear class


% FigHandle=figure(1);
% set(FigHandle, 'Position', [600, 0, 1600, 800]);
sim=Simulation();

sim.addOperatingRobots(4);
sim.addChargingRobots(2);
sim.setImageScale(.01);
sim.setSimulationTime(400); % switch to 200 for circle scenario

% close all
%% Set initial positions and speed of charging robots using ChargingRobot.m 

setpos(sim.list_of_charging_robots(1),0,3);
      setspeed(sim.list_of_charging_robots(1),100);
setpos(sim.list_of_charging_robots(2),0,13);
      setspeed(sim.list_of_charging_robots(2),100);

setTimeStep(sim,1);
setChargingTime(sim,5); %switch to 15 for circle
a=sim.list_of_operating_robots(1);

plan(sim,'LKH','Distance');


sim.plot()
set(gca,'XTick',[])
set(gca,'YTick',[])
 % pause (3)
 % sim.simulate()