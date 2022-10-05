function sim_world()
    close all
    
    interface = [];
    interface.colors.background      = [1 1 1]*0.25;
    interface.colors.plot_background = [1 1 1]*0.4;
    interface.colors.plot_lines      = [1 1 1]*1;
    interface.main_figure.f = figure(...
        'color',interface.colors.background,...
        'position',[1080 30 800 900]);%[450 80 1080 720]);
    interface.main_figure.ax_static = axes(...
        'position',[0.05 0.05 0.4 0.9],...
        'color',interface.colors.plot_background,...
        'xcolor',interface.colors.plot_lines,...
        'ycolor',interface.colors.plot_lines);
    axis image
    hold on
    grid on
    interface.main_figure.ax_dynamic = axes(...
        'position',[0.55 0.05 0.4 0.9],...
        'color',interface.colors.plot_background,...
        'xcolor',interface.colors.plot_lines,...
        'ycolor',interface.colors.plot_lines);
    axis image
    hold on
    grid on
    
    % ******************* initialize object properties *******************
    dt = 0.05;              % (s) time resolution
    
    road = [];              % road properties
    road.T = 40;            % (m) road periodicity
    road.x = 0:0.2:80;      % (m) road x array
    road.y = @(x) ...       % (m) road y array
        road.T/2*sin(2*pi/road.T*x).*cos(2*pi/road.T/3*x);
    
    ego = [];               % ego properties
    ego.v = 5;             % (m/s) ego speed
    ego.x = @(t,x_1)...     % (m) ego x position
        ego.v*t - road.x(end)*floor(x_1/road.x(end));
    ego.y = @(x) road.y(x); % (m) ego y position
    ego.x_1 = 0;            % (m) ego's last x
    
    % ************ initialize simulation animation variables *************
    t = 0;                                      % (s) time
    set(interface.main_figure.ax_static,...     % dynamic axes limits
        'xlim', [-20 20],...
        'ylim', [0 80]);
    set(interface.main_figure.ax_dynamic,...    % dynamic axes limits
        'xlim', [-20 20],...
        'ylim', [-40 40]);
    road_tail = road.x(round(end/2));           % (m) road tail behind ego
    
    road_x = road.x;
    road_y = road.y(road_x);
    
    ego_x = ego.x(t,ego.x_1);
    ego_y = ego.y(ego_x);
    
    % static axis
    road.m.static = plot(interface.main_figure.ax_static,...
        road_y, road_x,'w','linewidth',2);
    ego.m.static =  plot(interface.main_figure.ax_static,...
        ego_y,ego_x,'or','linewidth',2);
    
    % dynamic axes
    [road_x,road_y] = dynamic_transform_coordinates(...
        road_x,road_y,ego_x,ego_y);
    road.m.dynamic = plot(interface.main_figure.ax_dynamic,...
        road_y,road_x,'w','linewidth',2);
    ego.m.dynamic =  plot(interface.main_figure.ax_dynamic,...
        0,0,'or','linewidth',2);
    
    % ************************* simulation start *************************
    
    while t < 100;
        
        % define plot properties
        
        % update common variables
        ego_x = ego.x(t,ego.x_1);
        ego_y = ego.y(ego_x);
        road_x = road.x + ego.x_1-road_tail;
        road_y = road.y(road_x);
        
        % ************************* static axes *************************
        
        % update ego
        set(ego.m.static,'xData',ego_y,'yData',ego_x)
        ego.x_1 = ego.x(t,0);
        
        % ************************* dynamic axes *************************
        
        % update road
        [road_x,road_y] = dynamic_transform_coordinates(...
        road_x,road_y,ego.x_1,ego.y(ego.x_1));
        set(road.m.dynamic,'xData',road_y,'yData',road_x)
        
        % increment time
        t = t + dt;
        pause(dt)
    end
    
    % *********************** function definitions ***********************
    
    function [x, y] = dynamic_transform_coordinates(x,y,x_ref,y_ref)
        % rotation matrix
        r = @(x,y,t) [x(:),y(:)]*[cos(t) -sin(t);...
                                  sin(t)  cos(t)];
        % rmove offset
        x = x - x_ref;
        y = y - y_ref;
        
        %find root
        [~,i_root] = min(abs(x));
                
        % calculate rotation angle
        dx = x(i_root+1) - x(i_root);
        dy = y(i_root+1) - y(i_root);
        theta = atan2(dy,dx);
        
        % apply rotation
        xy = r(x,y,theta);
        
        % return results
        x = xy(:,1);
        y = xy(:,2);
        
    end
    
end