    for i = 1:npts
        figure(1);clf; hold on;
        xlim([0 0.8]); ylim([-0.3 0.3]); zlim([-0.33 0.8]);
        view(20,20);
        scatter3(x_points(i,:), y_points(i,:), z_points(i,:), 100, 'filled', 'b'); hold on;
        plot3(x_points(i,:), y_points(i,:), z_points(i,:), 'LineWidth', 5, 'color', 'b');
        c = [1 0 0];
        plotObj(objDim, objPos, c);
        c = [0 1 1];
        plotObj(bowlDim, bowlPos, c);
        plotBound;
        pause(0.1);
    end