function plotBound()
    % xyz positive and xyz negative for bounding box
    xp = 0.7; yp = 0.150; zp = 0.7;
    xn = 0; yn = -0.150; zn = -0.2;
    boundPoints = [[xn;yn;zn],[xn;yp;zn],[xp;yp;zn],[xp;yn;zn]...
                   [xn;yn;zp],[xn;yp;zp],[xp;yp;zp],[xp;yn;zp]];
    faces = [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 5 6 7 8];
    for f = 1:size(faces)
        b = fill3(boundPoints(1,faces(f,:)),boundPoints(2,faces(f,:)),boundPoints(3,faces(f,:)), [0.9 0.6 0.6]); hold on;
        alpha(b,0.5);
    end
end