clc, close all; clear all;
xStart = 5;   
yStart = 90; 
phiStart = 0;

xTarget1 = 60;
yTarget1 = 90;
phiTarget1 = 0;

xTarget2 = 32.5;
yTarget2 = 50;
phiTarget2 = -pi/2;

xTarget3 = 80;
yTarget3 = 32.5;
phiTarget3 = pi/2;


%% Get the motion planning (tree)
[cost,timeCost,iter,path] = RRT(xTarget2,yTarget2, phiTarget2,xTarget3,yTarget3,phiTarget3, 1,3,2000,1);

tree = struct('x',[],'y',[],'phi',[],'route',[]);
tree.x = [tree.x flip(path.x)];
tree.y = [tree.y flip(path.y)];
tree.phi = [tree.phi flip(path.phi)];
tree.route = [tree.route flip(path.route)];

