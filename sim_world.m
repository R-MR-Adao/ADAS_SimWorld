function sim_world()
    close all
    
    interface = [];
    interface.main_figure = figure('color','w');
    hold on
    
    dt = 0.05;   % s
    
    road = [];
    road.x = 0:40;  % units
    road.T = 40;
    road.y = @(x) road.T/2*sin(2*pi/road.T*x);
    
    ego = [];
    ego.v = 10;     % units/s
    ego.x = @(t,x_1) ego.v*t - road.x(end)*floor(x_1/road.x(end));
    ego.y = @(x) road.y(x);
    ego.x_1 = 0;
    
    t = 0;
    plot(road.y(road.x), road.x)
    axis image
    ego.m = plot(ego.y(ego.x(t,ego.x_1)),ego.x(t,ego.x_1),'or');
    
    while t < 100;
        % plot ego
        ego_x = ego.x(t,ego.x_1);
        ego_y = ego.y(ego_x);
        set(ego.m,'xData',ego_y,'yData',ego_x)
        ego.x_1 = ego.x(t,0);
        t = t + dt;
        pause(dt)
    end
    
end