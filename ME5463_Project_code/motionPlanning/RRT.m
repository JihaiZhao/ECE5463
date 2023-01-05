% This function is modifed based on the example developed by 
% Omkar Halbe, Technical University of Munich, 31.10.2015
function [cost,timeCost,iter,path] = RRT(xStart, yStart, phiStart, xGoal, yGoal, phiGoal, bias, MaxDist, MaxTree,enable_visual)
% clear; clc
% close all;

if nargin < 7
    bias = 1;
end

if nargin < 8
    MaxDist = 3;    % the maximum distance allowed to travel for each time
end

if nargin < 9
    MaxTree = 2000; % the maximum number of nodes in the tree
end

if nargin < 10
    enable_visual = true;
    close all;
end

maxSpeed.v=3;
maxSpeed.w=1;

%% initialization
xMin = 0;    % minimum x value
yMin = 0;    % minimum y value
xMax = 100;  % maximum x value
yMax = 100;  % maximum y value

phiMin = -pi; % minimum phi value
phiMax = pi; % maximum phi value

thresh = 3;   % acceptable distance from the goal point
phiThresh = 10*pi/180;

% we will use an structure array to represent the tree structure
% each entry in the array represent one node in the tree 
% (i.e., one point in the 2D configuration space)
% each node the following element:
%  - x: the x coordinate of the node
%  - y: the y coordinate of the node
%  - cost: the cost to arrive at this node
%  - parent: the index of the parent node 

% The tree will be initialized by the starting point
nodes(1).x = xStart;
nodes(1).y = yStart;

nodes(1).phi = phiStart;

nodes(1).cost = 0;
nodes(1).parent = 0;
nodes(1).timeCost = 0;

%%%%%%%%%%%%%% Modify obstacle %%%%%%%%%%%%%%

list = linspace(10, 85, 6);
for i = 1:3
    for j = 1:3
        num = j+(i-1)*3;
        obstacles(num).xv = [list(2*j-1); list(2*j); list(2*j); list(2*j-1); list(2*j-1)];
        obstacles(num).yv = [list(2*i-1); list(2*i-1); list(2*i); list(2*i); list(2*i-1)];
    end
end

obstacles(10).xv = [70; 100; 100; 70; 70];
obstacles(10).yv = [90; 90; 100; 100; 90];

%%%%%%%%%%%%%%%%%%%% End %%%%%%%%%%%%%%%%%%%%


%% visualization

if enable_visual 
    
    showMap;

    WheeledRobotPoints(xStart,yStart,phiStart);

end


%% search
iter = 2;
% the value of "bias" can affect how to sample from the C-space
% bias = 1: no bias
% bias < 1: biased toward the goal  (xMax, yMax)
% bias > 1: biased toward the start (xMin, yMin)
% bias = 1;%1/sqrt(2);

while iter < MaxTree   
    % sample a random point and random orientation
    [xRand, yRand, phiRand ] = random_sample(xMin,yMin,xMax,yMax,phiMin,phiMax,bias);
    % sort the tree to find the nearest node
    p_index = sort_tree(nodes, xRand, yRand); % not use phi in finding nearest
    xNear = nodes(p_index).x;
    yNear = nodes(p_index).y;
    phiNear = nodes(p_index).phi;
    % plan to motion from the nearest point to the random point by MaxDist
    [xNew, yNew, phiNew,xMiddle,yMiddle,phiMiddle,totalTime,route] = local_planner(xNear,yNear,phiNear,xRand,yRand,phiRand,MaxDist,maxSpeed);
    % check collision of the new path from the nearest point to the new
    % point
    
    %% Test whether the middle point or the new point collide
    if ~isnan(xMiddle) && ~isnan(yMiddle) && ~isnan(phiMiddle)
    c_test1 = collision_detector(xNear,yNear,xMiddle,yMiddle,obstacles);% no need phi here
    c_test2 = collision_detector(xMiddle,yMiddle,xNew, yNew,obstacles);
    % if there is a collision, dump the current trial
        if c_test1==true || c_test2==true
        continue;
        end
    end
    
    if isnan(xMiddle) && isnan(yMiddle) && isnan(phiMiddle)
        c_test = collision_detector(xNear,yNear,xNew, yNew,obstacles);
        if c_test==true
        continue;
        end
    end
        
    
    
    
    %% update the tree    
    nodes(iter).x = xNew;       %#ok<*AGROW,*SAGROW> 
    nodes(iter).y = yNew;       % the new point as a new node
    nodes(iter).phi = phiNew;
    nodes(iter).parent = p_index; % the nearest point is the parent node
    nodes(iter).cost = distance(xNew,yNew,xNear,yNear) + nodes(p_index).cost; %parent's cost + distance
    nodes(iter).timeCost = nodes(iter-1).timeCost+totalTime;
    nodes(iter).middlePointX = xMiddle;
    nodes(iter).middlePointY = yMiddle;
    nodes(iter).middlePointPHI = phiMiddle;
    nodes(iter).route = route;
    
    % plot the new path
    if enable_visual
        plot([nodes(iter).x; nodes(p_index).x],[nodes(iter).y; nodes(p_index).y], 'r');
        pause(0.0001);
    end
    
    %% check if it reaches the goal
    if (distance(xNew, yNew, xGoal, yGoal) <= thresh) && (phiNew<=phiThresh+phiGoal && phiNew>=-phiThresh+phiGoal)
        break
    end
    
    
    iter = iter + 1;
    
    
end

%% final cost
cost = nodes(end).cost;
timeCost = nodes(end).timeCost;

%% determine the feasible path

if iter < MaxTree
    xPath(1) = xGoal;
    yPath(1) = yGoal;
    phiPath(1) = phiGoal;
    
    xPath(2) = nodes(end).x;
    yPath(2) = nodes(end).y;
    phiPath(2) = nodes(end).phi;
    routePath{2} = nodes(end).route;
    
    xPath(3) = nodes(end).middlePointX;
    yPath(3) = nodes(end).middlePointY;
    phiPath(3) = nodes(end).middlePointPHI;
    
    parent = nodes(end).parent;
    j=0;

    while parent ~= 0
        xPath(j+4) = nodes(parent).x;
        yPath(j+4) = nodes(parent).y;   
        phiPath(j+4) = nodes(parent).phi;
        routePath{j+4} = nodes(parent).route;
        
        if parent~=1 
        xPath(j+5) = nodes(parent).middlePointX;
        yPath(j+5) = nodes(parent).middlePointY;
        phiPath(j+5) = nodes(parent).middlePointPHI;
        end 
        
        parent = nodes(parent).parent;
        j=j+2;
    end
   
    %% Get path and plot
    path.x = xPath;
    path.y = yPath;
    path.phi = phiPath;
    path.route = routePath;
    
    % use to plot: remove NaN:
    xPath_plot = xPath(~isnan(xPath));
    yPath_plot = yPath(~isnan(yPath));
    if enable_visual
        
        plot(xPath_plot, yPath_plot, 'g', 'Linewidth', 3);
    end
else
    
    disp('No path found. Increase number of iterations and retry.');
    cost = NaN;
    iter = NaN;
    path = NaN;
end

end
