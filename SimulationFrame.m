%% Initializing
% Clear all variables from the workspace.
clear all
% clc
% new SMORES object
M1 = SMORES([0 0 0]',0,0,0,0,[0 -1 0]',[1 0 0]',[0 1 0]');

%% Begain The Main Loop
% Configuration of the simulation loop
tInitial =0;            % Start time
tStep = 1;           % The simulation's time step, in seconds.
maxExcutTime = 60;    % The maximum simulating time, in second, not accurate.

ControlTimer = tic;
while tInitial<=maxExcutTime
%% Motion Control Loop
    tInitial=tInitial+tStep;
    RealTimeInterval = toc(ControlTimer);
% How many modules there are, how many motion control functions there will
% be. The only input of the motion control function is the time interval
%     M1.MotionCalculation(RealTimeInterval);
    M1.MotionCalculation(0.01);


    ControlTimer = tic;
%% Visulized Animation
    M1.SpacePlot;




    hold off;
%% Reconfiguration Algorithm
% Must contain the condition for ending the simulation
    M1.Wheel1SpeedControl(4,3);
    M1.Wheel2SpeedControl(10,3);



    pause(tStep);
end