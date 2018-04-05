function [objPoints] = edgeObj(objDim, objPos)
    objPoints = [0;0;0;0];
    for n = 1:size(objPos,1)
        objL = objDim(n,1); objW = objDim(n,2);
        objX = objPos(n,1); objY = objPos(n,2); objTh = atan2(objY,objX);
        objPointsTemp = [[objL/2; objW/2] [-objL/2; objW/2] [-objL/2; -objW/2] [objL/2; -objW/2]];
        objRot = [cos(objTh) -sin(objTh); sin(objTh) cos(objTh)];
        objTrs = [[objX; objY] [objX; objY] [objX; objY] [objX; objY]...
                  ];
        temp = (objRot*objPointsTemp + objTrs)';
        objPoints(:,2*n-1) = temp(:,1);
        objPoints(:,2*n) = temp(:,2);
        
    end
end