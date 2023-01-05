function [phiList,index] = findPHIforPlot(x_pos,y_pos,x,y,phi)
    for ii = 1:1:length(x_pos)
        distanceList=[];
        for jj=1:length(x)
            dist = distance(x_pos(ii), y_pos(ii),x(jj),y(jj));
            distanceList=[distanceList; dist];
        end
        [~,index(ii)]=min(distanceList);
        phiList(ii) = phi(index(ii));

    end
end

