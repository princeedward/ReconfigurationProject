clear all;
clf;
% BEFA1 = SMORES([0 0 0]',30,90,0,-10);
BEFA1 = SMORES([0 0 0]',0,0,0,0,[0 -1 0]',[1 0 0]',[0 1 0]');
BEFA1.Wheel1SpeedControl(17,3);
% BEFA1.Wheel2SpeedControl(10,3);
% tic
BEFA1.MotionCalculation(0.01);
% toc
tic
BEFA1.SpacePlot();
toc
% BEFA1.Wheel1Speed
BEFA1.CenterPos
% BEFA1.PosDirVector