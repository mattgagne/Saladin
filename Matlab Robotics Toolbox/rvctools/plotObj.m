function plotObj(objDim, objPos, c)
    for n = 1:size(objPos,1)
        objL = objDim(n,1); objW = objDim(n,2); objH = objDim(n,3);
        objX = objPos(n,1); objY = objPos(n,2); objZ = objPos(n,3); objTh = atan2(objY,objX);
        objPoints = [[objL/2; objW/2; 0] [-objL/2; objW/2; 0] [-objL/2; -objW/2; 0] [objL/2; -objW/2; 0] [objL/2; objW/2; objH] [-objL/2; objW/2; objH] [-objL/2; -objW/2; objH] [objL/2; -objW/2; objH]];
        objRot = [cos(objTh) -sin(objTh) 0; sin(objTh) cos(objTh) 0; 0 0 1];
        objTrs = [[objX; objY; objZ] [objX; objY; objZ] [objX; objY; objZ] [objX; objY; objZ]...
                  [objX; objY; objZ] [objX; objY; objZ] [objX; objY; objZ] [objX; objY; objZ]];
        objPoints = objRot*objPoints + objTrs;
        faces = [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 5 6 7 8];
        for f = 1:size(faces)
            fill3(objPoints(1,faces(f,:)),objPoints(2,faces(f,:)),objPoints(3,faces(f,:)), c); hold on;
        end
    end
end