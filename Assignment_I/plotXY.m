function endSim = plotXY(Ranges, Angles)
    endSim = any(Ranges > 0);
    if endSim
        x = Ranges .* cos(Angles);
        y = Ranges .* sin(Angles);
        xValid = x(isfinite(x));
        yValid = y(isfinite(y));
        plot(xValid, yValid, marker = '.', LineStyle = 'none', ...
            MarkerSize = 8);
        title('LaserScan');
        xlabel('x[m]', 'Interpreter','latex');
        ylabel('y', 'Interpreter','latex');
        set(gca, 'YDir', 'reverse');
        grid on;
        axis equal;
        ylim([-5, 5]);
        xlim([-5 5]);
        view([90 -90]);
end