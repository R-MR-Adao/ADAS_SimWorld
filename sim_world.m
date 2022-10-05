function sim_world()
    
    % initialize interface
    interface = init_interface();
    
    % ******************* initialize object properties *******************
    dt = 0.05;              % (s) time resolution
    
    % initialize road map
    road = init_road();
    
    % motion equation for ego and moving objects
    x_t = @(v,t,x_1) v.*t - road.x(end).*floor(x_1/road.x(end));
    
    % initialize ego
    ego = init_ego(x_t, road);
    
    % initialize standing objects
    stand_n_objects = 10;
    stand = init_stand(stand_n_objects,road);
    
    % initialize moving and oncoming objects
    mov_n_objects = 2;      % number of moving objects
    mov = init_mov_objects(mov_n_objects, 1,x_t,ego,road); % moving obj
    onc = init_mov_objects(mov_n_objects,-1,x_t,ego,road); % oncoming obj
    
    % initialize simulation animation variables
    [interface, road, ego, stand, mov, onc, road_tail] = ...
            init_plots(interface, road, ego, stand, mov, onc);
    %initialize simulation time
    t = 0; % (s) time
    while strcmp(get(interface.main_figure.f,'visible'),'on')
        [t, dt, road, ego, stand, mov, onc, road_tail] = ...
            simulation_run(t, dt, road, ego, stand, mov, onc, road_tail);
        output_sim_data(interface,road,ego,stand,mov,onc)
    end
    
    
    
    % *********************** function definitions ***********************
    
    function [t, dt, road, ego, stand, mov, onc, road_tail] =...
            simulation_run(t, dt, road, ego, stand, mov, onc, road_tail)
        % update common variables
        ego_x =  ego.x(t,ego.x_1);
        ego_y =  ego.y(ego_x);
        mov_x =  mov.x(t,mov.x_1);
        mov_y =  mov.y(mov_x);
        onc_x =  onc.x(t,onc.x_1);
        onc_y =  onc.y(onc_x);
        road_x = road.x + ego.x_1-road_tail;
        road_y = road.y(road_x);
        
        % ************************* static axes *************************
        
        % update static axes
        set(ego.m.static,'xData',ego_x,'yData',ego_y)
        ego.x_1 = ego.x(t,0);
        set(mov.m.static,'xData',mov_x,'yData',mov_y)
        mov.x_1 = mov.x(t,0);
        set(onc.m.static,'xData',onc_x,'yData',onc_y)
        onc.x_1 = onc.x(t,0);
        
        % ************************* dynamic axes *************************
        
        % update dynamic varibles
        [road_x,road_y,theta] = dynamic_transform_coordinates(...
            road_x,road_y,ego.x_1,ego.y(ego.x_1));
        % shift standing objects for smoother rendering
        s_shift = road.x(end)*((ego_x-stand.x) > road.x(round(end/2))) +... 
                 -road.x(end)*((stand.x-ego_x) > road.x(round(end/2)));
        [stand_x,stand_y] = dynamic_transform_coordinates(...
            stand.x+s_shift,stand.y,ego_x,ego_y, theta);
        % transform moving object corrdinates
        m_shift = road.x(end)*((ego_x-mov_x) > road.x(round(end/2))) +... 
                 -road.x(end)*((mov_x-ego_x) > road.x(round(end/2)));
        [mov_x,mov_y] = dynamic_transform_coordinates(...
            mov_x+m_shift,mov_y,ego_x,ego_y, theta);
        % transform oncoming object corrdinates
        o_shift = road.x(end)*((ego_x-onc_x) > road.x(round(end/2))) +... 
                 -road.x(end)*((onc_x-ego_x) > road.x(round(end/2)));
        [onc_x,onc_y] = dynamic_transform_coordinates(...
            onc_x+o_shift,onc_y,ego_x,ego_y, theta);
        
        % find sensor data
        for ii = 1 : ego.sensor.n
            ego.sensor.data(ii).stand = sensor_data_find(...
                ego.sensor,ii,stand_x,stand_y);
            ego.sensor.data(ii).mov = sensor_data_find(...
                ego.sensor,ii,mov_x,mov_y);
            ego.sensor.data(ii).onc = sensor_data_find(...
                ego.sensor,ii,onc_x,onc_y);
        end
        
        % update dynamic axes
        set(road.m.dynamic, 'xData',road_y, 'yData',road_x)
        set(stand.m.dynamic,'xData',stand_y,'ydata',stand_x);
        set(mov.m.dynamic,  'xData',mov_y,  'yData',mov_x)
        set(onc.m.dynamic,  'xData',onc_y,  'yData',onc_x)
        
        sensor_remap = [3 4 2 1];
        ego.sensor.key = {'RL', 'RR', 'FR', 'FL'};
        % update sensor-specific data
        for ii = 1 : 4
            set(ego.sensor.d{sensor_remap(ii)}.stand,...
                'xData',ego.sensor.data(ii).stand(:,2),...
                'yData',ego.sensor.data(ii).stand(:,1))
            set(ego.sensor.d{sensor_remap(ii)}.mov,...
                'xData',ego.sensor.data(ii).mov(:,2),...
                'yData',ego.sensor.data(ii).mov(:,1))
            set(ego.sensor.d{sensor_remap(ii)}.onc,...
                'xData',ego.sensor.data(ii).onc(:,2),...
                'yData',ego.sensor.data(ii).onc(:,1))
        end
        
        % increment time
        t = t + dt;
        pause(dt)
    end

    function data = sensor_data_find(sensor,ii,data_x,data_y)
        rd = @(x,y,t) [x(:),y(:)]*[cosd(t) -sind(t);... % rotation frame
                                   sind(t)  cosd(t)];
        data = rd(data_x, data_y,sensor.theta(ii));
        d = sqrt(data(:,1).^2 + data(:,2).^2);
        theta = atan2d(data(:,2),data(:,1));
        theta(theta < -360) = theta(theta < -360) + 360;
        theta(theta > 360) = theta(theta > 360) - 360;
        cond = logical((d < sensor.fov.range).*...
            (-sensor.fov.theta/2<theta).*(theta<sensor.fov.theta/2));
        data = data(cond,:);
    end
    
    function obj = init_road()
        obj = [];              % road properties
        obj.T = 80;            % (m) road periodicity
        obj.x = 0:0.2:240;     % (m) road x array
        obj.y = @(x) ...       % (m) road y array
            obj.T/2*sin(2*pi/obj.T*x).*cos(2*pi/obj.T/3*x);
    end
    
    function obj = init_ego(x_t, road)
        obj = [];                % ego properties
        obj.v = 5;              % (m/s) ego speed
        obj.x = @(t,x_1) x_t(obj.v,t,x_1); % (m) ego x position;
        obj.y = @(x) road.y(x); % (m) ego y position
        obj.x_1 = 0;            % (m) ego's last x
        % set sensor field od view properties
        obj.sensor.n = 4;              % number of sensors
        obj.sensor.fov.range = 80;     % sensor range
        obj.sensor.fov.theta = 150;    % sensor angular range
        % initialize sensors
        obj.sensor.theta(1:obj.sensor.n) =... % sensor orientation
            45 + 360/obj.sensor.n*(1:obj.sensor.n);
        % normalize to 360 deg
        obj.sensor.theta(obj.sensor.theta > 360) =...
            obj.sensor.theta(obj.sensor.theta > 360) - 360;
        obj.sensor.fov.draw.s = ...    % sensor drawing parameter
            (-0.5:0.02:0.5)'*obj.sensor.fov.theta;
        % initialize global coverage drawing
        obj.sensor.fov.draw.fov = obj.sensor.fov.range*...
                [[0, 0];...
                [cosd(obj.sensor.fov.draw.s),...
                 sind(obj.sensor.fov.draw.s)];...
                [0, 0]];
        % initialize sensor specific coverage circles
        for ii = 1 : obj.sensor.n
            obj.sensor.fov.draw.circ{ii} = obj.sensor.fov.range*...
                [[0, 0];...
                [cosd(obj.sensor.fov.draw.s+obj.sensor.theta(ii)),...
                 sind(obj.sensor.fov.draw.s+obj.sensor.theta(ii))];...
                [0, 0]];
        end 
    end

    function obj = init_stand(n,road)
        % range for standing objects
        rg_x = [min(road.x) max(road.x)]; 
        rg_y = [-1 1]*road.T ;
        obj = []; % standing (static) objects properties
        % randomly generate standing object positions
        obj.x = rand(n,1)*diff(rg_x) + rg_x(1);
        obj.y = rand(n,1)*diff(rg_y) + rg_y(1); 
    end
    
    function obj = init_mov_objects(n, direction, x_t, ego, road)        
        % moving objects
        obj = [];               % moving object properties
        obj.v = direction*ego.v*1.5*rand(n,1); % (m/s) moving object speed
        obj.t0 = ...            % random starting position
            rand(n,1)*diff(road.x([1 end]))./obj.v;
        obj.x = @(t,x_1)...     % (m) moving object x position;
            x_t(obj.v,t+obj.t0,x_1); 
        obj.y = @(x) road.y(x); % (m) moving object y position
        obj.x_1 = zeros(n,1); % (m) moving object last x
    end
    
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

    function [interface, road, ego, stand, mov, onc, road_tail] =...
            init_plots(interface, road, ego, stand, mov, onc)
        set(interface.main_figure.ax_static,... % dynamic axes limits
            'xlim', [road.x(1) road.x(end)],...
            'ylim', [-1 1]*road.T/2);
        set(interface.main_figure.ax_dynamic,...% dynamic axes limits
            'xlim', [-1 1]*(ego.sensor.fov.range + 10),...
            'ylim', [-1 1]*(ego.sensor.fov.range + 10));
        road_tail = road.x(round(end/2));       % (m) road tail behind ego

        % initialze common variables
        r_x = road.x;           % road x array
        r_y = road.y(r_x);      % road y array

        e_x = ego.x(0,ego.x_1); % ego x position
        e_y = ego.y(e_x);       % ego y position

        m_x = mov.x(0,mov.x_1); % moving obect x position
        m_y = mov.y(m_x);       % moving object y position

        o_x = onc.x(0,onc.x_1); % oncoming object x position
        o_y = onc.y(o_x);       % oncoming object y position

        % static axes plots
        road.m.static = plot(interface.main_figure.ax_static,...
            r_x, r_y,'w','linewidth',2);
        ego.m.static =  plot(interface.main_figure.ax_static,...
            e_x,e_y,'or','linewidth',2);
        stand.m.static = plot(interface.main_figure.ax_static,...
            stand.x, stand.y,'og','linewidth',2);
        mov.m.static =  plot(interface.main_figure.ax_static,...
            m_x,m_y,'o','color',[1 0.5 0],'linewidth',2);
        onc.m.static =  plot(interface.main_figure.ax_static,...
            o_x,o_y,'oc','linewidth',2);

        % dynamic axes transformations
        % transform road coordinates
        [r_x,r_y,th] = dynamic_transform_coordinates(...
            r_x,r_y,e_x,e_y);
        % transform standing object coordinates
        [s_x,s_y] = dynamic_transform_coordinates(...
                stand.x,stand.y,e_x,e_y, th);
        % transform moving object corrdinates
        [m_x,m_y] = dynamic_transform_coordinates(...
                m_x,m_y,e_x,e_y, th);
        % transform oncoming object corrdinates
        [o_x,o_y] = dynamic_transform_coordinates(...
                o_x,o_y,e_x,e_y, th);

        % sensor fov
        for ii = 1 : ego.sensor.n
            ego.sensor.m(ii) = patch(... % draw sensor FoV
                ego.sensor.fov.draw.circ{ii}(:,1),...
                ego.sensor.fov.draw.circ{ii}(:,2),...
                'w','FaceAlpha',.3,...
                'parent', interface.main_figure.ax_dynamic);
            ego.sensor.s(ii) = patch(... % draw sensor FoV
                ego.sensor.fov.draw.fov(:,2),...
                ego.sensor.fov.draw.fov(:,1),...
                'w','FaceAlpha',.3,...
                'parent', interface.main_figure.ax_sensor(ii));
            ego.sensor.d{ii}.stand = plot(...
                interface.main_figure.ax_sensor(ii),...
                [0],[0],'og');
            ego.sensor.d{ii}.mov = plot(...
                interface.main_figure.ax_sensor(ii),...
                [0],[0],'o','color',[1 0.5 0]);
            ego.sensor.d{ii}.onc = plot(...
                interface.main_figure.ax_sensor(ii),...
                [0],[0],'oc');
        end
        
        % dynamic axes plots
        road.m.dynamic = plot(interface.main_figure.ax_dynamic,...
            r_y,r_x,'w','linewidth',2);
        ego.m.dynamic =  plot(interface.main_figure.ax_dynamic,...
            0,0,'or','linewidth',2);
        stand.m.dynamic = plot(interface.main_figure.ax_dynamic,...
            s_y,s_x,'og','linewidth',2);
        mov.m.dynamic =  plot(interface.main_figure.ax_dynamic,...
            m_x,m_y,'o','color',[1 0.5 0],'linewidth',2);
        onc.m.dynamic =  plot(interface.main_figure.ax_dynamic,...
            o_x,o_y,'oc','linewidth',2);

    end

    % ***************************** interface *****************************
    
    function interface = init_interface()
        interface = [];
        interface.colors.background      = [1 1 1]*0.25;
        interface.colors.plot_background = [1 1 1]*0.4;
        interface.colors.plot_lines      = [1 1 1]*1;
        interface.main_figure.f = figure(...
            'color',interface.colors.background,...
            'position',[1 31 1920 973],... %[450 80 1080 720]
            'CloseRequestFcn',@f_CloseRequestFcn);
        
        % initialize axes
        interface.main_figure.ax_static = axes(...
            'position',[0.55 0.73 0.45 0.21]);
        init_axes_style(interface.main_figure.ax_static,interface)
        interface.main_figure.ax_dynamic = axes(...
            'position',[0.55 0.05 0.45 0.63],'xdir','reverse');
        init_axes_style(interface.main_figure.ax_dynamic,interface)
        % draw sensor axes
        w = 0.21;        % axes width
        h = 0.2;        % axes height
        off_x = 0.02;       % x offset
        off_y = 0.05;       % y offset
        xy_off = [0       w+off_x 0 w+off_x;...
                  h+off_y h+off_y 0 0      ]';
        ttl = {'FL','FR','RL','RR'};
        for ii = 1 : 4
            interface.main_figure.ax_sensor(ii) = axes(...
                'position',[0.15+xy_off(ii,1) 0.05+xy_off(ii,2) w h],...
                'xdir','reverse');
            init_axes_style(interface.main_figure.ax_sensor(ii),interface)
            title(ttl{ii},'color','w')
        end
    end

    function init_axes_style(ax, interface)
       set(ax,...
           'color',interface.colors.plot_background,...
           'xcolor',interface.colors.plot_lines,...
           'ycolor',interface.colors.plot_lines)
        axis image
        hold on
        grid on
        box on
    end
    
    function output_sim_data(interface,road,ego,stand,mov,onc)
        sim_world_data.road = road;
        sim_world_data.ego = ego;
        sim_world_data.stand = stand;
        sim_world_data.mov = mov;
        sim_world_data.onc = onc;
        sim_world_data.interface = interface;
        assignin('base','sim_world_data',sim_world_data)
    end

    function f_CloseRequestFcn(source,eventData)
        set(source,'visible','off')
    end
    
end