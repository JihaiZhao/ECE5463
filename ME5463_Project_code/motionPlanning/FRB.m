function [xMiddle,yMiddle,phiMiddle,totalTime] = FRB(xNear,yNear,phiNear,xNew,yNew,phiNew,maxSpeed)
%% Change to relative position between these two points
xNew_relative = xNew-xNear;
yNew_relative = yNew-yNear;
phiNew_relative = phiNew-phiNear;

%% middle point
phiMiddle = phiNew;
[xMiddle,yMiddle] = findMiddlePoint(xNear,yNear,phiNear,xNew,yNew,phiNew);

%% total time
time_F = distance(xNear,yNear,xMiddle,yMiddle)/maxSpeed.v;
time_R = abs(phiNew_relative/maxSpeed.w);
time_B = distance(xMiddle,yMiddle,xNew,yNew)/maxSpeed.v;
totalTime = time_F+time_R+time_B;
end