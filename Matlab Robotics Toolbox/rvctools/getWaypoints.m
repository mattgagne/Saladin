%Anton Dolgovykh
%Mathieu Gagne
%Nicholas Heersink

function [waypoint] = getWaypoints(objPos, objDim, bowlPos, bowlDim)
waypoint = [];
    for ob = 1:size(objPos,1)
        distAt = sqrt(objPos(ob,1)^2 + objPos(ob,2)^2) - objDim(ob,1)/2 - 0.01;
        ang = atan2(objPos(ob,2),objPos(ob,1));
        x = distAt*cos(ang);
        y = distAt*sin(ang);
        
        b(1,:) = bowlPos + [0, -bowlDim(2)/2 - 0.01];
        b(2,:) = bowlPos + [0, bowlDim(2)/2 + 0.01];
        b(3,:) = bowlPos + [-bowlDim(1)/2 - 0.01, 0];
        [d, index] = min([norm([x y] - b(1,:)), norm([x y] - b(2,:)), norm([x y] - b(3,:))]);
        bowl = b(index,:);
        
        % Add to waypoints, above, at, above, bowl, above, at, above for each object
        waypoint(size(waypoint,1)+1,:) = [x, y];
        waypoint(size(waypoint,1)+1,:) = [bowl(1), bowl(2)];
        waypoint(size(waypoint,1)+1,:) = [x, y]; 
    end
    waypoint(length(waypoint)+1,:) = waypoint(length(waypoint),:);    
end