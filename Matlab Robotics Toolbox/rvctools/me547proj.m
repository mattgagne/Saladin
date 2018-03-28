close all; clear all; clc;
startup_rvc;

    syms pi
    % DH table
    L(1) = Link('d', 0.254, 'a', 0, 'alpha', pi/2);
    L(2) = Link('d', 0, 'a', 0.254, 'alpha', 0);
    L(3) = Link('d', 0, 'a', 0.254, 'alpha', 0);
    L(4) = Link('d', 0, 'a', 0, 'alpha', pi/2);
    L(5) = Link('d', 0.0508, 'a', 0, 'alpha', 0);

    %Form the robot
    Fanuc_200id = SerialLink(L, 'name', '200id');
    
    
    
%% Set up environment
objDim = [0.1 0.1 0.1; 0.1 0.1 0.1];
objPos = [0.1 0.4; 0.4 0.1];
bowlDim = [0.15 0.15 0.15];
bowlPos = [0.3 0.3;];
height_clear = 0.05;
offset = 0.1;
waypoint = [];
    for ob = 1:size(objDim,1)
        dist_at = sqrt(objPos(ob,1)^2 + objPos(ob,2)^2) - objDim(ob,1)/2;
        dist_above = dist_at - offset;
        ang = atan2(objPos(ob,2),objPos(ob,1));
        x_at = dist_at*cos(ang);
        y_at = dist_at*sin(ang);
        x_above = dist_above*cos(ang);
        y_above = dist_above*sin(ang);
        % Add to waypoints, above, at, bowl, above, at for each object
        waypoint(size(waypoint,1)+1,:) = [x_above, y_above, objDim(ob,3) + height_clear];
        waypoint(size(waypoint,1)+1,:) = [x_at, y_at, objDim(ob,3)/2];
        waypoint(size(waypoint,1)+1,:) = [bowlPos(1), bowlPos(2), bowlDim(3) + height_clear];
        waypoint(size(waypoint,1)+1,:) = [x_above, y_above, objDim(ob,3) + height_clear];
        waypoint(size(waypoint,1)+1,:) = [x_at, y_at, objDim(ob,3)/2];
    end
    
%waypoint = [0.1 0.1 0.1;  0.13 0.13 0.1; 0.16 0.16 0.1; 0.2 0.2 0.1; 0.23 0.23 0.1; 0.26 0.26 0.1; 0.3 0.3 0.1];
%waypoint = [0.15 0.25 0.05; 0.1 0.1 0.05];
%% Follow path
q = [0 0 0 0 0];
%q = [ 0.92    0.1842   -1.5754         0         0];
getTmatrix;
%plotRobot;
qPrev = q;
npts = 0;
%%
for point = 1:size(waypoint,1)
    xNext = waypoint(point,1);
    yNext = waypoint(point,2);
    zNext = waypoint(point,3);
    
    r = sqrt(xNext^2 + yNext^2);
    s = zNext - 0.254;
    L1 = 0.254;
    L2 = 0.254;

    D = (r^2 + s^2 - L1^2 - L2^2)/(2*L1*L2);

    qNext(3) = atan2(-sqrt(1-D^2), D);
    qNext(2) = atan2(s,r) - (-acos((r^2 + s^2 + L1^2 - L2^2)/(2*L1*sqrt(r^2+s^2))));
    qNext(1) = atan2(yNext,xNext);
    qNext(4) = -qNext(2) - qNext(3) - pi/2;
    qNext(5) = deg2rad(-90);
    disc = 8;
    path_p(1, :) = linspace(qPrev(1), qNext(1), disc);
    path_p(2, :) = linspace(qPrev(2), qNext(2), disc);
    path_p(3, :) = linspace(qPrev(3), qNext(3), disc);
    path_p(4, :) = linspace(qPrev(4), qNext(4), disc);
    path_p(5, :) = linspace(qPrev(5), qNext(5), disc);
    if (mod(point,5) == 1) || (mod(point,5) == 2) || (mod(point,5) == 3)
        for joint = 3:-1:1%size(q,2)
            for delta = 1:disc
               q(joint) = path_p(joint,delta);

                getTmatrix;

                npts = npts+1;
                x_points(npts,:) = [0 T01(1,4) T02(1,4) T03(1,4) T04(1,4)];% T05(1,4) T06(1,4)];
                y_points(npts,:) = [0 T01(2,4) T02(2,4) T03(2,4) T04(2,4)];% T05(2,4) T06(2,4)];
                z_points(npts,:) = [0 T01(3,4) T02(3,4) T03(3,4) T04(3,4)];% T05(3,4) T06(3,4)];
           end
        end
    end
    if (mod(point,5) == 4) || (mod(point,5) == 0)% || (mod(point,5) == 3)
        for joint = 1:1:3%size(q,2)
            for delta = 1:disc
               q(joint) = path_p(joint,delta);

                getTmatrix;

                npts = npts+1;
                x_points(npts,:) = [0 T01(1,4) T02(1,4) T03(1,4) T04(1,4)];% T05(1,4) T06(1,4)];
                y_points(npts,:) = [0 T01(2,4) T02(2,4) T03(2,4) T04(2,4)];% T05(2,4) T06(2,4)];
                z_points(npts,:) = [0 T01(3,4) T02(3,4) T03(3,4) T04(3,4)];% T05(3,4) T06(3,4)];
           end
        end
    end
   qPrev = qNext;
end
%% Plot
plotRobot;
c = [1 0 0];
plotObj(objDim, objPos, c);
c = [0 1 1];
plotObj(bowlDim, bowlPos, c);