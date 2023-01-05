
xStarT = 5;   
yStarT = 90; 
phiStarT = 0;

xTarget1 = 60;
yTarget1 = 90;
phiTarget1 = 0;

xTarget2 = 32.5;
yTarget2 = 50;
phiTarget2 = 3*pi/2;

xTarget3 = 80;
yTarget3 = 32.5;
phiTarget3 = pi/2;

list = linspace(10, 85, 6);
thresh = 3;

for i = 1:3
    for j = 1:3
        num = j+(i-1)*3;
        obstacles(num).xv = [list(2*j-1); list(2*j); list(2*j); list(2*j-1); list(2*j-1)];
        obstacles(num).yv = [list(2*i-1); list(2*i-1); list(2*i); list(2*i); list(2*i-1)];
    end
end

obstacles(10).xv = [70; 100; 100; 70; 70];
obstacles(10).yv = [90; 90; 100; 100; 90];

figure;hold on; grid on
set(gcf, 'Position', get(0, 'Screensize'))    
axis equal
axis([0,100,0,100]);
% plot the boundary
plot([0,0,100,100,0],[0,100,100,0,0],'k--','LineWidth',1);


% plot the obstacles
for i=1:length(obstacles)
    plot(obstacles(i).xv,obstacles(i).yv,'k','LineWidth',1);
end

% plot the starting point
plot(xStarT, yStarT, 'ro', 'MarkerSize',3, 'MarkerFaceColor','r');

% Plot the target points and target regions 
th = 0:pi/50:2*pi;

plot(xTarget1, yTarget1, 'go', 'MarkerSize',3, 'MarkerFaceColor','b');
plot(xTarget2, yTarget2, 'go', 'MarkerSize',3, 'MarkerFaceColor','b');
plot(xTarget3, yTarget3, 'go', 'MarkerSize',3, 'MarkerFaceColor','b');

xcircle1 = thresh * cos(th) + xTarget1;
ycircle1 = thresh * sin(th) + yTarget1;
h = plot(xcircle1, ycircle1, 'Color','b');

xcircle2 = thresh * cos(th) + xTarget2;
ycircle2 = thresh * sin(th) + yTarget2;
h = plot(xcircle2, ycircle2, 'Color','b');


xcircle3 = thresh * cos(th) + xTarget3;
ycircle3 = thresh * sin(th) + yTarget3;
h = plot(xcircle3, ycircle3, 'Color','b');
