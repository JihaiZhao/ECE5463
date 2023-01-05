close all;
clear;
clc;

%% Load x,y,phi from RRT result
load('Track3Data.mat');
x_pos = tree.x;
y_pos = tree.y;
phi_pos = tree.phi;

% delete NaN point
x_pos = x_pos(~isnan(x_pos));
y_pos = y_pos(~isnan(y_pos));
phi_pos = phi_pos(~isnan(phi_pos));

pos =  [phi_pos; x_pos; y_pos];

% get path
path = [];
for i=1:length(x_pos)
    path = [path; x_pos(i) y_pos(i)];
end

%% Use toolbox to get control
% set robot's specifcation
R = 5;
L = 2.5;
dd = DifferentialDrive(R,L);

% define controller
sampleTime = 0.1;
tVec = 0:sampleTime:120;

initPose = [32.5, 50, -pi/2];
pose = zeros(3, numel(tVec));
pose(:, 1) = initPose;

controller = robotics.PurePursuit;
controller.Waypoints = path;
controller.LookaheadDistance = 0.1;
controller.DesiredLinearVelocity = 1;
cpntroller.MaxAngularVelocity = 1;

% get track of the robot
for idx = 2:numel(tVec)
    [vRef,wRef] = controller(pose(:, idx-1));
    [wL, wR] = inverseKinematics(dd, vRef, wRef);
    
    [v, w] = forwardKinematics(dd, wL, wR);
    velB = [v; 0; w];
    vel = bodyToWorld(velB, pose(:, idx-1));
    
    pose(:, idx) = pose(:, idx-1) + vel * sampleTime;
end

x = pose(1,:);
y = pose(2,:);
phi = pose(3,:);
r = 5;

%% Generate robot's moving video
out = VideoWriter('TrackRoute3.avi');
out.FrameRate = 12;
open(out)

for t = 1:5:1201
    
x_temp = x(t);
y_temp = y(t);
phi_temp = phi(t);

open('Track3.fig')

robotShape(x_temp, y_temp, 5, phi_temp)

F = getframe (gcf);
writeVideo(out, F);

figNum = t + 1;

close all

end

close(out);

%% Generate robot's route
open('Track3.fig')
plot(x,y,'k', 'LineWidth', 2)

[phiList,index] = findPHIforPlot(x_pos,y_pos,x,y,phi);
   
phiList = rem(rem(phiList, 2*pi),2*pi);
for i=1:length(phiList)
    if phiList(i) > pi
        phiList(i) = phiList(i) - 2*pi;
    end
    if phiList(i) < -pi
        phiList(i) = 2*pi + phiList(i);
    end
end   
    
%% Generate comparison graph for phi
figure(2)
plot(1:length(phiList),phiList ,'og--', 'linewidth', 2)
hold on
plot(1:length(phiList), phi_pos, '*r-', 'linewidth', 2)
grid on
legend('Control Track', 'RRT Track')
xlabel('Node')
ylabel('phi(rad)')
