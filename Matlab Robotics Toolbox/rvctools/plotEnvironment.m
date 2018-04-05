%takes in the points of the rectangles and plots the environment

function plotEnvironment(ptsStore,posMinBound, posMaxBound, startPos, endPos, nCups, nRand)

hold on
for i = 1:2:size(ptsStore,2)
    if (i/2 <= nCups)
        c = [0 1 1]; % cup colour
    elseif (i <= nCups + nRand)
        c = [0 1 0]; % rand obj colour
    else
        c = [1 0 0]; % bowl colour
    end
    patch(ptsStore(:,i),ptsStore(:,i+1), c,'linestyle','-','EdgeColor', c, 'LineWidth',2)
end
plot(startPos(1),startPos(2),'b*')
plot(endPos(1),endPos(2),'g*')
patch([posMinBound(1) posMaxBound(1) posMaxBound(1) posMinBound(1)],[posMinBound(2) posMinBound(2) posMaxBound(2) posMaxBound(2)],...
    [1 0 0],'facecolor','none','linestyle','-','EdgeColor',[0 1 1])
hold off
axis equal
axis([posMinBound(1) posMaxBound(1) posMinBound(2) posMaxBound(2)]);