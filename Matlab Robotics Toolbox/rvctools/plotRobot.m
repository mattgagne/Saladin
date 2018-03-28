    for i = 1:npts
        figure(1);clf; hold on;
        xlim([0 0.6]); ylim([0 0.6]); zlim([0 0.5]);
        view(45,45);
        scatter3(x_points(i,:), y_points(i,:), z_points(i,:), 100, 'filled', 'b'); hold on;
        plot3(x_points(i,:), y_points(i,:), z_points(i,:), 'LineWidth', 5, 'color', 'b');
        c = [1 0 0];
        plotObj(objDim, objPos, c);
        c = [0 1 1];
        plotObj(bowlDim, bowlPos, c);
        pause(0.1);
    end