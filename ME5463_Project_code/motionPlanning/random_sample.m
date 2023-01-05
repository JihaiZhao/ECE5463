function [xRand, yRand, phiRand] = random_sample(xMin,yMin,xMax,yMax,phiMin,phiMax,bias)
    
    xRand = (xMax-xMin)*rand^bias;
    yRand = (yMax-yMin)*rand^bias;
    
    %phiRand = (phiMax-phiMin)*rand^bias;
    phiRand = ((phiMax-phiMin)/2)*(-1+2*rand)^bias;
end