% This function is modifed based on the example developed by 
% Omkar Halbe, Technical University of Munich, 31.10.2015
function [cost,iter,path] = RRT(xStart, yStart, phiStart, xGoal, yGoal, phiGoal, bias, MaxDist, MaxTree,enable_visual)
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

%% initialization
xMin = 0;    % minimum x value
yMin = 0;    % minimum y value
xMax = 100;  % maximum x value
yMax = 100;  % maximum y value

thresh = 3;   % acceptable distance from the goal point

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
nodes(1).cost = 0;
nodes(1).parent = 0;

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
    % sample a random point
    [xRand, yRand] = random_sample(xMin,yMin,xMax,yMax,bias);
    % sort the tree to find the nearest node
    p_index = sort_tree(nodes, xRand, yRand);
    xNear = nodes(p_index).x;
    yNear = nodes(p_index).y;
    % plan to motion from the nearest point to the random point by MaxDist
    [xNew, yNew] = local_planner(xNear,yNear,xRand,yRand,MaxDist);
    % check collision of the new path from the nearest point to the new
    % point
    c_test = collision_detector(xNear,yNear,xNew, yNew,obstacles);
    % if there is a collision, dump the current trial
    if c_test
        continue;
    end
    
    % update the tree    
    nodes(iter).x = xNew;       %#ok<*AGROW,*SAGROW> 
    nodes(iter).y = yNew;       % the new point as a new node
    nodes(iter).parent = p_index; % the nearest point is the parent node
    nodes(iter).cost = distance(xNew,yNew,xNear,yNear) + nodes(p_index).cost; %parent's cost + distance
    
    % plot the new path
    if enable_visual
        plot([nodes(iter).x; nodes(p_index).x],[nodes(iter).y; nodes(p_index).y], 'r');
        pause(0.001);
    end
    
    % check if it reaches the goal region
    if distance(xNew, yNew, xGoal, yGoal) <= thresh
        break
    end
    
    iter = iter + 1;
end

%% final cost
cost = nodes(end).cost;
%% determine the feasible path

if iter < MaxTree
    xPath(1) = xGoal;
    yPath(1) = yGoal;
    xPath(2) = nodes(end).x;
    yPath(2) = nodes(end).y;
    
    parent = nodes(end).parent;
    j=0;
    while 1
        xPath(j+3) = nodes(parent).x;
        yPath(j+3) = nodes(parent).y;        
        parent = nodes(parent).parent;
        if parent == 0
            break
        end
        j=j+1;
    end
    if enable_visual
        plot(xPath, yPath, 'g', 'Linewidth', 3);
    end
else
    disp('No path found. Increase number of iterations and retry.');
    cost = NaN;
    iter = NaN;
end
path.x = xPath;
path.y = yPath;
end
