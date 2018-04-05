
clear; clc;


%% Parameters:

% Set up the boundaries
xMax = [0.7 0.15]; % cage bounds
xMin = [0 -0.15];
xR = xMax-xMin;

%Set up start of robot arm:
x0 = [0 0];
% Set up the cup and bowl positions
objDim = [0.075 0.075; 0.075 0.075; 0.075 0.075];
objPos = [0.2 0.08; 0.2 -0.08; 0.5 0.1];
nCups = size(objPos,1);
bowlDim = [0.1 0.1];
bowlPos = [0.4 -0.05];
randObjDim = [];
randObjPos = [];
nRand = size(randObjPos,1);  

for i=1: length(objPos)
    totalObjDim(i,:) = objDim(i,:);
    totalObjPos(i,:) = objPos(i,:);
end
for i=1 : length(randObjDim) 
    totalObjDim(i+ length(objPos,1),:) = randObjDim(i,:);
    totalObjPos(i+ length(objPos,1),:) = randObjPos(i,:);
end
totalObjDim(length(totalObjDim) +1,:) = bowlDim;
totalObjPos(length(totalObjPos) +1,:) = bowlPos;

obsPtsStore = edgeObj(totalObjDim, totalObjPos);

%Set up waypoints:
waypoints = getWaypoints(objPos, objDim, bowlPos, bowlDim);

% Cup params
%nO = length(objDim) + length(randObjDim) +1; % number of cups + number of random obstacles + bowl -1
nE = 4; % number of edges per obstacle (square vis)

%Create enviornment:

% obsEdges = [];
% 
% for i=1:nO
%     env = [env; NaN NaN; obsPtsStore(:,2*(i-1)+1:2*i);obsPtsStore(1,2*(i-1)+1:2*i)];
%     obsEdges = [obsEdges; obsPtsStore(1:nE,2*(i-1)+1:2*i) obsPtsStore([2:nE 1],2*(i-1)+1:2*i)];
% end

total_distance = 0;
for wp=1:length(waypoints)
    totalObjDim = [];
    totalObjPos = [];
    if mod(wp,3) ~= 1
        if floor(wp/4)+1>1
            for i=1: floor((wp-1)/3)
                totalObjDim(i,:) = objDim(i,:);
                totalObjPos(i,:) = objPos(i,:);
            end
            for i=floor((wp-1)/3)+2 :length(objPos)
                totalObjDim(i,:) = objDim(i,:);
                totalObjPos(i,:) = objPos(i,:); 
            end
        else
            for i=2 :length(objPos)
                totalObjDim(i,:) = objDim(i,:);
                totalObjPos(i,:) = objPos(i,:); 
            end
        end
        % Cup params
%         nO = length(objDim) + length(randObjDim); % number of cups + number of random obstacles + bowl -1
    else
        for i=1: length(objPos)
            totalObjDim(i,:) = objDim(i,:);
            totalObjPos(i,:) = objPos(i,:);
        end
        % Cup params
%         nO = length(objDim) + length(randObjDim) +1; % number of cups + number of random obstacles + bowl -1
    end
    
    for i=1 : length(randObjDim) 
        totalObjDim(i+ length(objPos,1)-1,:) = randObjDim(i,:);
        totalObjPos(i+ length(objPos,1)-1,:) = randObjPos(i,:);
    end
    totalObjDim(length(totalObjDim) +1,:) = bowlDim;
    totalObjPos(length(totalObjPos) +1,:) = bowlPos;

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
    disp('Time to create environment');

    %% Multi-query PRM, created in batch
    tic;

    % Get milestones
    nS = 200;
    samples = [xR(1)*rand(nS,1)+xMin(1) xR(2)*rand(nS,1)+xMin(2)];
    keep = inpolygon(samples(:,1),samples(:,2), env(:,1),env(:,2));
    milestones = [x0; xF; samples(find(keep==1),:)];
    figure(1); hold on;
    %plot(samples(:,1),samples(:,2),'k.');
    plot(milestones(:,1),milestones(:,2),'m.');
    nM = length(milestones(:,1));
    disp('Time to generate milestones');
    toc;

    % Attempt to add closest p edges
    tic;
    p = 20;
    e = zeros(nM,nM);
    D = zeros*ones(nM,nM);

    for i = 1:nM
        % Find closest neighbours
        for j = 1:nM
            d(j) = norm(milestones(i,:)-milestones(j,:));
        end
        [d2,ind] = sort(d);
        % Check for edge collisions (no need to check if entire edge is
        % contained in obstacles as both endpoints are in free space)
        for j=1:p
            cur = ind(j);
            if (i<cur)
                if (~CheckCollision(milestones(i,:),milestones(cur,:), obsEdges))
                    e(i,cur) = 1;
                    e(cur,i) = 1;
                    plot([milestones(i,1) milestones(cur,1)],[milestones(i,2) milestones(cur,2)],'m');
                end
            end
        end
    end
    disp('Time to connect roadmap');
    toc;

    % Find shortest path
    tic;
    [sp, sd] = shortestpath(milestones, e, 1, 2);
    for i=1:length(sp)-1
        plot(milestones(sp(i:i+1),1),milestones(sp(i:i+1),2), 'go-', 'LineWidth',3);
    end
    pause(2);
    disp('Time to find shortest path');
    toc;
    x0 = waypoints(wp,:);
    total_distance = total_distance + sd;
end
total_distance