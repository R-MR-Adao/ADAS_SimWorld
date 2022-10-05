function sim_world()
    close all
    
    interface = [];
    interface.colors.background      = [1 1 1]*0.25;
    interface.colors.plot_background = [1 1 1]*0.4;
    interface.colors.plot_lines      = [1 1 1]*1;
    interface.main_figure.f = figure(...
        'color',interface.colors.background,...
        'position',[1080 30 800 900],... %[450 80 1080 720]
        'CloseRequestFcn',@f_CloseRequestFcn);
    interface.main_figure.ax_static = axes(...
        'position',[0.05 0.05 0.3 0.9],...
        'color',interface.colors.plot_background,...
        'xcolor',interface.colors.plot_lines,...
        'ycolor',interface.colors.plot_lines);
    axis image
    hold on
    grid on
    interface.main_figure.ax_dynamic = axes(...
        'position',[0.45 0.05 0.5 0.9],...
        'color',interface.colors.plot_background,...
        'xcolor',interface.colors.plot_lines,...
        'ycolor',interface.colors.plot_lines);
    axis image
    hold on
    grid on
    
    % ******************* initialize object properties *******************
    dt = 0.05;              % (s) time resolution
    
    road = [];              % road properties
    road.T = 80;            % (m) road periodicity
    road.x = 0:0.2:240;      % (m) road x array
    road.y = @(x) ...       % (m) road y array
        road.T/2*sin(2*pi/road.T*x).*cos(2*pi/road.T/3*x);
    
    % motion equation
    
    x_t = @(v,t,x_1)...     % (m) ego x position
        v.*t - road.x(end).*floor(x_1/road.x(end));
    
    ego = [];               % ego properties
    ego.v = 5;              % (m/s) ego speed
    ego.x = @(t,x_1) x_t(ego.v,t,x_1); % (m) ego x position;
    ego.y = @(x) road.y(x); % (m) ego y position
    ego.x_1 = 0;            % (m) ego's last x
    % set sensor field od view properties
    ego.sensor.n = 4;              % number of sensors
    ego.sensor.fov.range = 80;     % sensor range
    ego.sensor.fov.theta = 150;    % sensor angular range
    % initialize sensors
    ego.sensor.theta(1:ego.sensor.n) =... % sensor orientation
        45 + 360/ego.sensor.n*(1:ego.sensor.n);
    ego.sensor.fov.draw.s = ...    % sensor drawing parameter
        (-0.5:0.02:0.5)'*ego.sensor.fov.theta;
    % initialize sensor specific coverage circles
    for ii = 1 : ego.sensor.n
        ego.sensor.fov.draw.circ{ii} = ego.sensor.fov.range*...
            [[0, 0];...
            [cosd(ego.sensor.fov.draw.s+ego.sensor.theta(ii)),...
            sind(ego.sensor.fov.draw.s+ego.sensor.theta(ii))];...
            [0, 0]];
    end
    
    % standing objects
    stand_n_objects = 10;
    stand_rg_x = [min(road.x) max(road.x)]; % range for standing objects
    stand_rg_y = [-1 1]*road.T ; % range for standing objects
    stand = []; % standing (static) objects properties
    % randomly generate standing object positions
    stand.x = rand(stand_n_objects,1)*diff(stand_rg_x) + stand_rg_x(1);
    stand.y = rand(stand_n_objects,1)*diff(stand_rg_y) + stand_rg_y(1);
    
    % moving objects
    mov_n_objects = 2;      % number of moving objects
    mov = [];               % moving object properties
    mov.v = -ego.v*1.5*rand(mov_n_objects,1); % (m/s) moving object speed
    mov.t0 = ...            % random starting position
        rand(mov_n_objects,1)*diff(road.x([1 end]))./mov.v;
    mov.x = @(t,x_1)...     % (m) moving object x position;
        x_t(mov.v,t+mov.t0,x_1); 
    mov.y = @(x) road.y(x); % (m) moving object y position
    mov.x_1 = zeros(mov_n_objects,1); % (m) moving object last x
    
    % ************ initialize simulation animation variables *************
    t = 0;                                      % (s) time
    set(interface.main_figure.ax_static,... % dynamic axes limits
        'xlim', [-1 1]*road.T/2,...
        'ylim', [road.x(1) road.x(end)]);
    set(interface.main_figure.ax_dynamic,...% dynamic axes limits
        'xlim', [-1 1]*(ego.sensor.fov.range + 10),...
        'ylim', [-1 1]*(ego.sensor.fov.range + 10));
    road_tail = road.x(round(end/2));       % (m) road tail behind ego
    
    % initialze common variables
    road_x = road.x;            % road x array
    road_y = road.y(road_x);    % road y array
    
    ego_x = ego.x(t,ego.x_1);   % ego x position
    ego_y = ego.y(ego_x);       % ego y position
    
    mov_x = mov.x(t,mov.x_1);   % ego x position
    mov_y = mov.y(mov_x);       % ego y position
    
    % static axes plots
    road.m.static = plot(interface.main_figure.ax_static,...
        road_y, road_x,'w','linewidth',2);
    ego.m.static =  plot(interface.main_figure.ax_static,...
        ego_y,ego_x,'or','linewidth',2);
    stand.m.static = plot(interface.main_figure.ax_static,...
        stand.y, stand(1).x,'og','linewidth',2);
    mov.m.static =  plot(interface.main_figure.ax_static,...
        mov_y,mov_x,'ob','linewidth',2);
    
    % dynamic axes transformations
    % transform road coordinates
    [road_x,road_y,theta] = dynamic_transform_coordinates(...
        road_x,road_y,ego_x,ego_y);
    % transform standing object coordinates
    [s_x,s_y] = dynamic_transform_coordinates(...
            stand.x,stand.y,ego_x,ego_y, theta);
    % transform moving object corrdinates
    [mov_x,mov_y] = dynamic_transform_coordinates(...
            mov_x,mov_y,ego_x,ego_y, theta);
    
    % dynamic axes plots
    road.m.dynamic = plot(interface.main_figure.ax_dynamic,...
        road_y,road_x,'w','linewidth',2);
    ego.m.dynamic =  plot(interface.main_figure.ax_dynamic,...
        0,0,'or','linewidth',2);
    stand.m.dynamic = plot(interface.main_figure.ax_dynamic,...
        s_y,s_x,'og','linewidth',2);
    mov.m.dynamic =  plot(interface.main_figure.ax_dynamic,...
        mov_x,mov_y,'ob','linewidth',2);
    
    % draw sensor FoV
    for ii = 1 : ego.sensor.n
        ego.sensor.m(ii) = patch(...
            ego.sensor.fov.draw.circ{ii}(:,1),...
            ego.sensor.fov.draw.circ{ii}(:,2),...
            'w','FaceAlpha',.3);
    end
    % ************************* simulation start *************************
    
    while strcmp(get(interface.main_figure.f,'visible'),'on')
        
        % update common variables
        ego_x =  ego.x(t,ego.x_1);
        ego_y =  ego.y(ego_x);
        mov_x =  mov.x(t,mov.x_1);
        mov_y =  mov.y(mov_x);
        road_x = road.x + ego.x_1-road_tail;
        road_y = road.y(road_x);
        
        % ************************* static axes *************************
        
        % update static axes
        set(ego.m.static,'xData',ego_y,'yData',ego_x)
        ego.x_1 = ego.x(t,0);
        set(mov.m(1).static,'xData',mov_y,'yData',mov_x)
        mov.x_1 = mov.x(t,0);
        
        % ************************* dynamic axes *************************
        
        % update dynamic varibles
        [road_x,road_y,theta] = dynamic_transform_coordinates(...
            road_x,road_y,ego.x_1,ego.y(ego.x_1));
        % shift standing objects for smoother rendering
        s_shift = road.x(end)*((ego_x-stand.x) > road.x(round(end/2))) +... 
                 -road.x(end)*((stand.x-ego_x) > road.x(round(end/2)));
        [s_x,s_y] = dynamic_transform_coordinates(...
            stand.x+s_shift,stand.y,ego_x,ego_y, theta);
        % transform moving object corrdinates
        m_shift = road.x(end)*((ego_x-mov_x) > road.x(round(end/2))) +... 
                -road.x(end)*((mov_x-ego_x) > road.x(round(end/2)));
        [mov_x,mov_y] = dynamic_transform_coordinates(...
            mov_x+m_shift,mov_y,ego_x,ego_y, theta);
        
        % update dynamic axes
        set(road.m.dynamic,'xData',road_y,'yData',road_x)
        set(stand.m.dynamic,'xData',s_y,'ydata',s_x);
        set(mov.m.dynamic,'xData',mov_y,'yData',mov_x)
        
        % increment time
        t = t + dt;
        pause(dt)
    end
    
    delete(interface.main_figure.f)
    
    % *********************** function definitions ***********************
    
    function [x,y,theta] = dynamic_transform_coordinates(...
            x,y,x_ref,y_ref,theta)
        % rotation matrix
        r = @(x,y,t) [x(:),y(:)]*[cos(t) -sin(t);...
                                  sin(t)  cos(t)];
        % rmove offset
        x = x - x_ref;
        y = y - y_ref;
        
        if nargin  < 5
            %find root
            [~,i_root] = min(abs(x));

            % calculate rotation angle
            dx = x(i_root+1) - x(i_root);
            dy = y(i_root+1) - y(i_root);
            theta = atan2(dy,dx);
        end
        
        % apply rotation
        xy = r(x,y,theta);
        
        % return results
        x = xy(:,1);
        y = xy(:,2);
        
    end

    % ************************ interface callbacks ************************
    function f_CloseRequestFcn(source,eventData)
        set(source,'visible','off')
    end
    
end