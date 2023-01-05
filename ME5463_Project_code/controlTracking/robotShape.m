function [] = robotShape(x,y,r,Q)

rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1],'LineWidth',1),axis equal
hold on
x2=x+10*cos(Q);
y2=y+10*sin(Q);

plot([x,x2],[y,y2],'k','LineWidth',1)  

end
