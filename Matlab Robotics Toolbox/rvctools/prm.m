%Anton Dolgovykh
%Mathieu Gagne
%Nicholas Heersink
%This code was created using base code from Steven Waslander's MTE 544
%course

clear; clc;

%% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('prm.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 0.5;
    open(vidObj);
end

%% Parameters:

% Set up the boundaries
xMax = [0.7 0.15]; % cage bounds
xMin = [0 -0.15];
xR = xMax-xMin;

%Set up start of robot arm:
x0 = [0 0];
% Set up the cup and bowl positions
%Config 1:
objDim = [0.055 0.055; 0.055 0.055; 0.0555 0.055; 0.055 0.055];
objPos = [0.2 0.08; 0.2 -0.08; 0.5 0.1; 0.5 -0.1;];
nCups = size(objPos,1);
bowlDim = [0.1 0.1];
bowlPos = [0.4 0];
randObjDim = [0.025 0.065; 0.025 0.065; 0.065 0.025; 0.025 0.065];
randObjPos = [0.3 -0.1; 0.3 0.05; 0.5 0; 0.175 0];
nRand = length(randObjPos);  

%Config 2:
% objDim = [0.055 0.055; 0.055 0.055; 0.0555 0.055; 0.055 0.055];
% objPos = [0.2 0.1; 0.3 0.1; 0.4 0.1; 0.5 0.1;];
% nCups = size(objPos,1);
% bowlDim = [0.1 0.1];
% bowlPos = [0.35 -0.12];
% randObjDim = [0.09 0.015; 0.09 0.015; 0.09 0.015];
% randObjPos = [0.25 0; 0.35 0; 0.45 0];
% nRand = length(randObjPos);  

%Config 3:
% objDim = [0.055 0.055; 0.055 0.055; 0.0555 0.055; 0.055 0.055];
% objPos = [0.2 0.08; 0.2 -0.08; 0.5 0.1; 0.5 -0.1;];
% nCups = size(objPos,1);
% bowlDim = [0.1 0.1];
% bowlPos = [0.35 -0.12];
% randObjDim = [0.02 0.2; 0.2 0.02; 0.2 0.02; 0.2 0.02];
% randObjPos = [0 0.05; 0.2 -0.02; 0.5 0.05;  0.5 0];
% nRand = length(randObjPos);    

%Config 4:

% objDim = [0.055 0.055; 0.055 0.055; 0.0555 0.055];
% objPos = [0.1 0.05; 0.22 -0.1; 0.475 0.11];
% nCups = size(objPos,1);
% bowlDim = [0.1 0.1];
% bowlPos = [0.4 0];
% randObjDim = [0.025 0.025; 0.025 0.025; 0.025 0.025];
% randObjPos = [0.2 0; 0.3 0.05; 0.125 -0.025];
% nRand = length(randObjPos);  

%Config 5:
% objDim = [ 0.055 0.055; 0.055 0.055; 0.055 0.055; 0.055 0.055;];
% objPos = [0.125 -0.055; 0.3 0.1; 0.5 0.02; 0.5 -0.1];
% nCups = size(objPos,1);
% bowlDim = [0.1 0.1];
% bowlPos = [0.4 0];
% randObjDim = [0.035 0.015; 0.020 0.025; 0.025 0.025; 0.025 0.025; 0.025 0.025; 0.015 0.035];
% randObjPos = [0.2 0; 0.25 0.0; 0.125 0.04; 0.45 0.12; 0.56 0.075; 0.35 -0.075];
% nRand = length(randObjPos);

for i=1: length(objPos)
    totalObjDim(i,:) = objDim(i,:);
    totalObjPos(i,:) = objPos(i,:);
end
for i=1 : length(randObjDim) 
    totalObjDim(i+ length(objPos),:) = randObjDim(i,:);
    totalObjPos(i+ length(objPos),:) = randObjPos(i,:);
end
totalObjDim(length(totalObjDim) +1,:) = bowlDim;
totalObjPos(length(totalObjPos) +1,:) = bowlPos;

obsPtsStore = edgeObj(totalObjDim, totalObjPos);

%Set up waypoints:
waypoints = getWaypoints(objPos, objDim, bowlPos, bowlDim);

% Cup params
nE = 4; % number of edges per obstacle (square vis)


total_distance = 0;
for wp=1:length(waypoints)
    totalObjDim = [];
    totalObjPos = [];
    if mod(wp,3) ~= 1
        if floor(wp/4)+1>1
            for i=1: floor((wp-1)/3)
                totalObjDim(i,:) = objDim(i,:) + [0.04 0.04];
                totalObjPos(i,:) = objPos(i,:);
            end
            for i=floor((wp-1)/3)+2 :length(objPos)
                totalObjDim(i-1,:) = objDim(i,:)+ [0.04 0.04];
                totalObjPos(i-1,:) = objPos(i,:); 
            end
            for i=1 : length(randObjDim) 
                totalObjDim(i+ length(objPos)-1,:) = randObjDim(i,:) + [0.04 0.04];
                totalObjPos(i+ length(objPos)-1,:) = randObjPos(i,:) ;
            end
            totalObjDim(length(totalObjDim) +1,:) = bowlDim;
            totalObjPos(length(totalObjPos) +1,:) = bowlPos;
        else
            for i=2 :length(objPos)
                totalObjDim(i-1,:) = objDim(i,:)+ [0.03 0.03];
                totalObjPos(i-1,:) = objPos(i,:); 
            end
            for i=1 : length(randObjDim) 
                totalObjDim(i+ length(objPos)-1,:) = randObjDim(i,:)+ [0.04 0.04];
                totalObjPos(i+ length(objPos)-1,:) = randObjPos(i,:);
            end
            totalObjDim(length(totalObjDim)+1 ,:) = bowlDim;
            totalObjPos(length(totalObjPos)+1 ,:) = bowlPos;
            
        end
        % Cup params
        nCups = size(objPos,1)-1;
            
        
    else
        for i=1: length(objPos)
            totalObjDim(i,:) = objDim(i,:);
            totalObjPos(i,:) = objPos(i,:);
        end
        nCups = size(objPos,1);
        for i=1 : length(randObjDim) 
            totalObjDim(i+ length(objPos),:) = randObjDim(i,:);
            totalObjPos(i+ length(objPos),:) = randObjPos(i,:);
        end
        totalObjDim(length(totalObjDim) +1,:) = bowlDim;
        totalObjPos(length(totalObjPos) +1,:) = bowlPos;
    end


    obsPtsStore = edgeObj(totalObjDim, totalObjPos);
    nO = length(totalObjDim);

    %Create enviornment:
    env = [xMin(1) xMin(2);xMin(1) xMax(2);xMax(1) xMax(2);xMax(1) xMin(2); xMin(1) xMin(2)];
    obsEdges = [];

    for i=1:nO
        env = [env; NaN NaN; obsPtsStore(:,2*(i-1)+1:2*i);obsPtsStore(1,2*(i-1)+1:2*i)];
        obsEdges = [obsEdges; obsPtsStore(1:nE,2*(i-1)+1:2*i) obsPtsStore([2:nE 1],2*(i-1)+1:2*i)];
    end

    %clear variables:
    d=[];
    ind = [];
    d2 =[];
    
    
    xF = waypoints(wp,:);
    % Plot obstacles
    figure(1); clf; hold on;
    plotEnvironment(obsPtsStore,xMin, xMax, x0, xF, nCups, nRand);
    drawnow();

    %% PRM

    % Get sample points(milestones)
    nS = 200;
    samples = [xR(1)*rand(nS,1)+xMin(1) xR(2)*rand(nS,1)+xMin(2)];
    keep = inpolygon(samples(:,1),samples(:,2), env(:,1),env(:,2));
    milestones = [x0; xF; samples(find(keep==1),:)];
    figure(1); hold on;
    plot(milestones(:,1),milestones(:,2),'m.');
    nM = length(milestones(:,1));

    % Attempt to add closest p edges
    p = 20;
    e = zeros(nM,nM);
    D = zeros*ones(nM,nM);

    for i = 1:nM
        % Find closest neighbours
        for j = 1:nM
            d(j) = norm(milestones(i,:)-milestones(j,:));
        end
        [d2,ind] = sort(d);
        % Check for edge collisions
        for j=1:p
            cur = ind(j);
            if (i<cur)
                if (~CheckCollision(milestones(i,:),milestones(cur,:), obsEdges))
                    e(i,cur) = 1;
                    e(cur,i) = 1;
                    plot([milestones(i,1) milestones(cur,1)],[milestones(i,2) milestones(cur,2)],'m', 'LineWidth', 0.5);
                end
            end
        end
    end

    % Find shortest path from milestones

    [sp, sd] = shortestpath(milestones, e, 1, 2);
    for i=1:length(sp)-1
        plot(milestones(sp(i:i+1),1),milestones(sp(i:i+1),2), 'go-', 'LineWidth',2);
    end
    pause(2);
    x0 = waypoints(wp,:);
    total_distance = total_distance + sd;
    if (makemovie) writeVideo(vidObj, getframe(gca)); end
    h = zeros(5, 1);
    h(1) = scatter(NaN,NaN,'filled','r');
    h(2) = scatter(NaN,NaN,'filled','g');
    h(3) = scatter(NaN,NaN,'filled','c');
    h(4) = scatter(NaN,NaN,'filled','b');
    h(5) = scatter(NaN,NaN,'filled','k');
    legend(h, 'Bowl', 'Obstacle', 'Cup','Start', 'Goal');
end
close(vidObj);
total_distance