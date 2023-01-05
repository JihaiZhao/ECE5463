function  c_test = collision_detector(xNear,yNear,xNew, yNew,obstacles)

d = 2.5;
r = 5;

    % we test 90 uniformally distributed points along the circle 
    % radius of circle: r
    % center of circle: (x, y)
    % at 10 uniformally distributed points along the line that
    % connects (xNear,yNear) to (xNew,yNew)
    s = 0:0.1:1;
    xq = xNear + s.*(xNew-xNear);
    yq = yNear + s.*(yNew-yNear);
    
    c_test = 0; % initialize as false (no collision)
    
    for theta = 0:2*pi/90:2*pi

        xqq = xq+r*cos(theta);
        yqq = yq+r*sin(theta);

        for i=1:length(obstacles)
            % check if any point is within the polygon
            in = inpolygon(xqq,yqq,obstacles(i).xv,obstacles(i).yv);
            if any(in) % if any one of the points are within the polygon
                c_test = 1; % set it true (collision)
                break;      % break the loop, no need to test other areas
            end
        end
    end
end