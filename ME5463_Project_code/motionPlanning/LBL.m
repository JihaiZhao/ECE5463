function [xMiddle,yMiddle,phiMiddle,totalTime] = LBL(xNear,yNear,phiNear,xNew,yNew,phiNew,maxSpeed)
%% Change to relative position between these two points
xNew_relative = xNew-xNear;
yNew_relative = yNew-yNear;

if yNew_relative > 0
    phi_t = acos(xNew_relative/sqrt(xNew_relative^2+yNew_relative^2));
else
    phi_t = -acos(xNew_relative/sqrt(xNew_relative^2+yNew_relative^2));
end

phi_t = phi_t+pi;

if phi_t > phiNear
    phiNew_relative1 = phiNear-phi_t+2*pi;
else
    phiNew_relative1 = phiNear-phi_t;
end

phiNew_relative1 = rem(2*pi-phiNew_relative1,2*pi);

if phiNew > phi_t
    phiNew_relative2 = phi_t-phiNew+2*pi;
else
    phiNew_relative2 = phi_t-phiNew;
end

phiNew_relative2 = rem(2*pi-phiNew_relative2,2*pi);

%% total time
time_1 = abs(phiNew_relative1/maxSpeed.w);
time_2 = abs((sqrt(xNew_relative^2+yNew_relative^2))/maxSpeed.v);
time_3 = abs(phiNew_relative2/maxSpeed.w);
totalTime = time_1+time_2+time_3;

%% middle point
phiMiddle = NaN;
xMiddle = NaN;
yMiddle = NaN;

end