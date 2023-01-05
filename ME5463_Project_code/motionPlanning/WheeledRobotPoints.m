function WheeledRobotPoints(x,y,phi)

d = 2.5;
r = 5;

k = sqrt(r^2-d^2);
xp = x-k*cos(phi);
yp = y-k*sin(phi);
l = k+r/2;

x1 = xp-d*sin(phi);
y1 = yp+d*cos(phi);

x2 = xp+d*sin(phi);
y2 = yp-d*cos(phi);

x3 = x2+l*cos(phi);
y3 = y2+l*sin(phi);

x5 = x1+l*cos(phi);
y5 = y1+l*sin(phi);

x4 = x+r*cos(phi);
y4 = y+r*sin(phi);

xlist = [x1; x2; x3; x4; x5; x1];
ylist = [y1; y2; y3; y4; y5; y1];

% draw wheeled robot 
plot(xlist,ylist,'k','LineWidth',1,'Color','r');

end