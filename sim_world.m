function sim_world()
    close all
    
    interface = [];
    interface.main_figure.f = figure('color','w');
    interface.main_figure.ax_static = axes('position',[0.1 0.1 0.3 0.8]);
    axis image
    hold on
    interface.main_figure.ax_dynamic = axes('position',[0.6 0.1 0.3 0.8]);
    axis image
    hold on
    xlim([-20 20]); ylim([-20 20])
    
    dt = 0.05;   % s
    
    road = [];
    road.x = 0:80;  % units
    road.T = 40;
    road.y = @(x) road.T/2*sin(2*pi/road.T*x);
    
    ego = [];
    ego.v = 10;     % units/s
    ego.x = @(t,x_1) ego.v*t - road.x(end)*floor(x_1/road.x(end));
    ego.y = @(x) road.y(x);
    ego.x_1 = 0;
    
    % initialize simulation animation variables
    t = 0;          % time
    
    road_x = road.x;
    road_y = road.y(road_x);
    
    ego_x = ego.x(t,ego.x_1);
    ego_y = ego.y(ego_x);
    
    % static axis
    road.m.static = plot(interface.main_figure.ax_static,...
        road_y, road_x);
    ego.m.static =  plot(interface.main_figure.ax_static,...
        ego_y,ego_x,'or');
    
    % dynamic axes
    [road_x,road_y] = dynamic_transform_coordinates(...
        road_x,road_y,ego_x,ego_y);
    road.m.dynamic = plot(interface.main_figure.ax_dynamic,...
        road_y,road_x);
    ego.m.dynamic =  plot(interface.main_figure.ax_dynamic,...
        0,0,'or');
    
    while t < 100;
        
        % update common variables
        ego_x = ego.x(t,ego.x_1);
        ego_y = ego.y(ego_x);
        road_x = road.x + ego.x_1;
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