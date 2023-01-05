function [xMiddle,yMiddle,phiMiddle,totalTime] = BLF(xNear,yNear,phiNear,xNew,yNew,phiNew,maxSpeed)
%% Change to relative position between these two points
xNew_relative = xNew-xNear;
yNew_relative = yNew-yNear;
phiNew_relative = phiNew-phiNear;

%% middle point
phiMiddle = phiNew;
[xMiddle,yMiddle] = findMiddlePoint(xNear,yNear,phiNear,xNew,yNew,phiNew);

%% total time
time_B = distance(xNear,yNear,xMiddle,yMiddle)/maxSpeed.v;
time_L = abs((phiNew_relative)/maxSpeed.w);
time_F = distance(xMiddle,yMiddle,xNew,yNew)/maxSpeed.v;
totalTime = time_F+time_L+time_B;