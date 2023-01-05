function [xNew, yNew, phiNew,xMiddle,yMiddle,phiMiddle,totalTime,route] = local_planner(xNear,yNear,phiNear,xRand,yRand,phiRand,MaxDist,maxSpeed)
    dist = distance(xRand,yRand,xNear,yNear);
    % if the distance to go to the (xRand,yRand) is greater than the
    % maximum distance, then travel MaxDist toward (xRand,yRand)
    if dist > MaxDist
        xNew= xNear + MaxDist*(xRand-xNear)/dist;
        yNew = yNear + MaxDist*(yRand-yNear)/dist;
        phiNew = phiRand;
        
        
    else % otherwise, travel to the (xRand,yRand)
        xNew = xRand;
        yNew = yRand;
        phiNew = phiRand;
        
%         
    end
    %%  
    [~,~,~,totalTime1] = FRB(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
    [~,~,~,totalTime2] = FLB(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
    [~,~,~,totalTime3] = BRF(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
    [~,~,~,totalTime4] = BLF(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
    [~,~,~,totalTime5] = RFR(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
    [~,~,~,totalTime6] = RFL(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
    [~,~,~,totalTime7] = LFR(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
    [~,~,~,totalTime8] = LFL(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
    [~,~,~,totalTime9] = LBL(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
    [~,~,~,totalTime10] = LBR(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
    [~,~,~,totalTime11] = RBL(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
    [~,~,~,totalTime12] = RBR(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
    
    %% Find the minimum one
    [~,index] = min([totalTime1 totalTime2 totalTime3 totalTime4 totalTime5 totalTime6 totalTime7 totalTime8 totalTime9 totalTime10 totalTime11 totalTime12]);
    if index==1
        [xMiddle,yMiddle,phiMiddle,totalTime] = FRB(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
        route = 'FRB';
    end
    if index==2
        [xMiddle,yMiddle,phiMiddle,totalTime] = FLB(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
        route = 'FLB';
    end
    if index==3
        [xMiddle,yMiddle,phiMiddle,totalTime] = BRF(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
        route = 'BRF';
    end
    if index==4
        [xMiddle,yMiddle,phiMiddle,totalTime] = BLF(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
        route = 'BLF';
    end
    if index==5
        [xMiddle,yMiddle,phiMiddle,totalTime] = RFR(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
        route = 'RFR';
    end
    if index==6
        [xMiddle,yMiddle,phiMiddle,totalTime] = RFL(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
        route = 'RFL';
    end
    if index==7
        [xMiddle,yMiddle,phiMiddle,totalTime] = LFR(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
        route = 'LFR';
    end
    if index==8
        [xMiddle,yMiddle,phiMiddle,totalTime] = LFL(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
        route = 'LFL';
    end
    if index==9
        [xMiddle,yMiddle,phiMiddle,totalTime] = LBL(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
        route = 'LBL';
    end
    if index==10
        [xMiddle,yMiddle,phiMiddle,totalTime] = LBR(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
        route = 'LBR';
    end
    if index==11
        [xMiddle,yMiddle,phiMiddle,totalTime] = RBL(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
        route = 'RBL';
    end
    if index==12
        [xMiddle,yMiddle,phiMiddle,totalTime] = RBR(xNear,yNear,phiNear,xNew,yNew,phiNew, maxSpeed);
        route = 'RBR';
    end
        
    
end