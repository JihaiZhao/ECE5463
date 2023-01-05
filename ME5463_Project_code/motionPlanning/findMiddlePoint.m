function [xMiddle,yMiddle] = findMiddlePoint(xNear,yNear,phiNear,xNew,yNew,phiNew)
myfunction = @(x) (tan(phiNew)*x+(yNew-xNew*tan(phiNew)))-(tan(phiNear)*x+(yNear-xNear*tan(phiNear)));
x0=xNew;

xMiddle = fzero(myfunction,x0);
yMiddle = tan(phiNew)*xMiddle+(yNew-xNew*tan(phiNew));     
end

