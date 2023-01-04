clc, close all; clear all;

%% p1
[cost,iter] = RRT()

%% p2
[cost, iter] = RRTmax()

%% p3
bias = [sqrt(2) 1 1/sqrt(2) 1/2];

for i = 1:length(bias)
    for j = 1:10
        [cost,iter] = RRT(bias(i));
        costSet(i,j) = cost;
        iterSet(i,j) = iter;
    end
end
costAve = mean(costSet,2)
iterAve = mean(iterSet,2)
costVar = var(costSet,0,2)
iterVar = var(iterSet,0,2)

%% p4
maxDist = [1 3 5 10];

for i = 1:length(maxDist)
    for j = 1:10
        [cost,iter] = RRT(1, maxDist(i));
        costSet(i,j) = cost;
        iterSet(i,j) = iter;
    end
end
costAve = mean(costSet,2)
iterAve = mean(iterSet,2)
costVar = var(costSet,0,2)
iterVar = var(iterSet,0,2)

%% p5
clc
maxTree = [500 1000 2000 3000];

for i = 1:length(maxTree)
    for j = 1:10
        [cost,iter] = RRTmax(1, 5, maxTree(i),0);
        costSet(i,j) = cost;
    end
end
costAve = mean(costSet,2)
costVar = var(costSet,0,2)

%% p6
clc; clear all; close all;
%%
[cost,iter] = RRT()
%%
[cost,iter] = RRTstarr()
%%
[cost, iter] = RRTmax()
%%
[cost, iter] = RRTmaxstarr()



