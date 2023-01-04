function [xRand, yRand] = random_sample(xMin,yMin,xMax,yMax,bias)
    if nargin < 5
        bias = 1;
    end
    
    xRand = (xMax-xMin)*rand^bias;
    yRand = (yMax-yMin)*rand^bias;
end