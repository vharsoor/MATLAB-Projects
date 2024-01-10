%% Calling simulink model and security

clc
clear all


Gain = 900000;
InitSpeed = 45; 
decelLim = -150;

[A,B,C,D,Kess, Kr, Ke, uD] = designControl(secureRand(),Gain);
open_system('LaneMaintainSystem.slx')

set_param('LaneMaintainSystem/VehicleKinematics/Saturation','LowerLimit',num2str(decelLim))
set_param('LaneMaintainSystem/VehicleKinematics/vx','InitialCondition',num2str(InitSpeed))

simModel = sim('LaneMaintainSystem.slx');
% Access the simulation output data
simOut = simModel.get('sx1');

% Extract the time and data vectors
time = simOut.time;
data = simOut.data;

% Display the time and data
disp('Time:');
disp(time(end));
disp('Distance:');
disp(data(end));

figure
plot(simModel.sx1.Time,simModel.sx1.Data)
title('Distance from the car')

figure
plot(simModel.vx1.Time,simModel.vx1.Data)
title('Velocity of the car')

figure
plot(simModel.ax1.Time,simModel.ax1.Data)
title('Deceleration of the car')