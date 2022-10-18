function sim_world()
    
    % initialize interface
    init_interface();
    
    % ******************* initialize object properties *******************
    controls_main_reset_Callback()
    
    % *********************** function definitions ***********************
    
    function master_run(interface,road,lane,road_edge,road_area,...
            ego,stand,mov,onc,road_tail,dt,t)
        
        override = true;    % override bool to implement do while
        
        while (get(interface.main_figure.buttons.controls_main_play,'value') ...
                || override)

            % run simulation
            [road,lane,road_edge,road_area,ego,stand,mov,onc,road_tail] = ...
                simulation_run(interface,t,dt,road,lane,road_edge,road_area,...
                ego,stand,mov,onc,road_tail);
            
            % fill inputs for user reconstruction
            sensor_f = fill_input_reconstruct_360_space(...
                interface,ego.sensor);
            
            % recover 360 degree coordinate space
            [stand_u,mov_u,onc_u] = reconstruct_360_space(sensor_f);
            
            % draw recovered coordinates
            update_plot_dynamic_user(stand,stand_u,mov,mov_u,onc,onc_u);

            % increment time
            t = t + dt;
            pause(dt/2)

            % output data to base workspace
            output_sim_data(...
                interface,road,lane,road_edge,road_area,...
                ego,stand,mov,onc,t,dt,road_tail)
            
            override = false;
        end
    end
    
    function [road,lane,road_edge,road_area,ego,stand,mov,onc,road_tail] =...
            simulation_run(interface,t,dt,road,lane,road_edge,road_area,ego,stand,mov,onc,road_tail)
        
        % ****************** update data for static plot ******************
        ego_x =  ego.x(t,ego.x_1);
        ego_y =  ego.y(ego_x);
        ego.x_1 = ego.x(t,0);
        
        mov_x =  mov.x(t,dt,mov.x_1);
        mov_y =  mov.y(t,dt,mov.x_1);
        mov.x_1 = mov.x(t,dt,0);
        
        onc_x =  onc.x(t,dt,onc.x_1);
        onc_y =  onc.y(t,dt,onc.x_1);
        onc.theta = atan2d(onc_y - onc.y(t-dt,dt,onc.x_1),onc_x - onc.x(t-dt,dt,onc.x_1));
        onc.x_1 = onc.x(t,dt,0);
        
        road_x = road.x + ego.x_1-road_tail;
        road_y = road.y(road_x);
        
        lane_x = lane.x(road_x);
        lane_y = lane.y(road_x);
        
        road_edge_x = road_edge.x(road_x);
        road_edge_y = road_edge.y(road_x);
        
        % update road area
        road_area = find_road_area(...
            interface,road_area,road_edge_x,road_edge_y,ego,ego_y);
        
        % update static axis
        update_plot_static(ego,ego_x,ego_y,mov,mov_x,mov_y,onc,onc_x,onc_y)
        
        % ***************** update data for dynamic plot *****************
        
        % transform road coordinates
        [road_x,road_y,theta] = dynamic_transform_coordinates(...
            road_x,road_y,ego.x_1,ego.y(ego.x_1));
        
        % transform lane coordinates
        [lane_x,lane_y] = dynamic_transform_coordinates(...
            lane_x,lane_y,ego.x_1,ego.y(ego.x_1), theta);
        
        % transform road edge coordinates
        [road_edge_x,road_edge_y] = dynamic_transform_coordinates(...
            road_edge_x,road_edge_y,ego.x_1,ego.y(ego.x_1), theta);
        
        % transform standing object coordinates
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
        for ii = 1 : onc.n % oncoming object cubes (visualization)
            onc.cube(ii).center = [onc_x(ii),onc_y(ii),0];
            onc.cube(ii).theta  = theta/pi*180 - onc.theta(ii)+90;
            onc.cube(ii)        = init_cube(onc.cube(ii));
        end
        
        % transform road map coordinates
        [ra_x,ra_y] = dynamic_transform_coordinates(...
                road_area.X(:),road_area.Y(:),0,0, theta);
        
        % reconstruct road area coordinates
        road_area_x = reshape(ra_x,size(road_area.X));
        road_area_y = reshape(ra_y,size(road_area.Y));
        
        % find data in sensor frames
        for ii = 1 : ego.sensor.n
            ego.sensor.data(ii).road = sensor_data_find(...
                ego.sensor,ii,road_x,road_y);
            ego.sensor.data(ii).stand = sensor_data_find(...
                ego.sensor,ii,stand_x,stand_y);
            ego.sensor.data(ii).mov = sensor_data_find(...
                ego.sensor,ii,mov_x,mov_y);
            ego.sensor.data(ii).onc = sensor_data_find(...
                ego.sensor,ii,onc_x,onc_y);
        end
               
        % update dynamic axes
        update_plot_dynamic(road,road_x,road_y,...
            lane,lane_x,lane_y,...
            road_edge,road_edge_x,road_edge_y,...
            road_area,road_area_x,road_area_y,...
            ego,stand,stand_x,stand_y,mov,mov_x,mov_y,onc,onc_x,onc_y)
    end

    function sensor_f = fill_input_reconstruct_360_space(interface,sensor)
        % copy selected data from the sensor
        sensor_f.n = sensor.n;                          % number of sensors
        sensor_f.fov.range = sensor.fov.range;          % fov dist range
        sensor_f.fov.theta = sensor.fov.theta;          % fov angular range
        sensor_f.theta = sensor.theta;                  % fov orientation
        sensor_f.pos = sensor.key;                      % sensor position
        
        % path to handle of sensor fov drawing
        sensor_path = ...
            'interface.main_figure.checkboxes.controls_main_showSensor_';
        
        % copy detected object data
        for ii = 1 : sensor.n
            sensor_switch = [sensor_path sensor.key{ii}];
            if get(eval(sensor_switch),'value')
                sensor_f.active(ii) = true;
                sensor_f.data(ii).stand = ...   % standing objects
                    sensor.data(ii).stand(~isnan(sensor.data(ii).stand(:,1)),:); 
                sensor_f.data(ii).mov = ...     % moving objects
                    sensor.data(ii).mov(~isnan(sensor.data(ii).mov(:,1)),:);
                sensor_f.data(ii).onc = ....    % oncoming objects
                    sensor.data(ii).onc(~isnan(sensor.data(ii).onc(:,1)),:);
                set(sensor.m(ii),'visible','on')
            else
                sensor_f.active(ii) = false;
                set(sensor.m(ii),'visible','off')
            end
        end
    end

    function update_plot_dynamic_user(stand,stand_u,mov,mov_u,onc,onc_u)
        
        % get interface from base workspace
        interface = evalin('base','sim_world_data.interface');
        
        if sum([...
                get(interface.main_figure.checkboxes.controls_main_showSensor_FL,'value'),...
                get(interface.main_figure.checkboxes.controls_main_showSensor_FR,'value'),...
                get(interface.main_figure.checkboxes.controls_main_showSensor_RL,'value'),...
                get(interface.main_figure.checkboxes.controls_main_showSensor_RR,'value')])
            set(stand.m.dynamic_user,...
                'xdata',stand_u(:,2),'ydata',stand_u(:,1),'visible','on')
            set(mov.m.dynamic_user,...
                'xdata',mov_u(:,2),'ydata',mov_u(:,1),'visible','on')
            set(onc.m.dynamic_user,...
                'xdata',onc_u(:,2),'ydata',onc_u(:,1),'visible','on')
        else
            set(stand.m.dynamic_user,'visible','off')
            set(mov.m.dynamic_user,'visible','off')
            set(onc.m.dynamic_user,'visible','off')
        end
    end

    function data = sensor_data_find(sensor,ii,data_x,data_y)
        % rotation matrix
        rd = @(x,y,t) [x(:),y(:)]*[cosd(t) -sind(t);... 
                                   sind(t)  cosd(t)]';
        
        % rotate data to sensor frame
        data = rd(data_x,data_y,-sensor.theta(ii));
        
        
        % check if data is in sensor FoV
        d = sqrt(data(:,1).^2 + data(:,2).^2);  % distance from ego
        theta = atan2d(data(:,2),data(:,1));    % angle
        theta(theta < -360) = theta(theta < -360) + 360; % normalize range
        theta(theta > 360) = theta(theta > 360) - 360;   % normalize range
        cond = logical((d < sensor.fov.range).*...       % in range
            (-sensor.fov.theta/2<theta).*(theta<sensor.fov.theta/2));
        
        % remove out of range points
        data(~cond,:) = nan;
    end
    
    function obj = init_road()
        obj.T = 80;            % (m) road periodicity
        obj.x = 0:0.2:240;     % (m) road x array
        obj.y = @(x) ...       % (m) road y array
            obj.T/3*sin(2*pi/obj.T*x).*cos(2*pi/obj.T/3*x);
    end

    function obj = init_mov_lane(road, off)
        
        % offset a point by off (objects)
        obj.ux =  @(x,x1) -(road.y(x1) - road.y(x)); % x derivative
        obj.uy =  @(x,x1) (x1-x);                % y derivative
        obj.unx = @(x,x1)...                     % perpendiculate vector x
            (obj.ux(x,x1).*off)./(sqrt(obj.ux(x,x1).^2 + obj.uy(x,x1).^2));
        obj.uny = @(x,x1)...                     % perpendiculate vector y
            (obj.uy(x,x1).*off)./(sqrt(obj.ux(x,x1).^2 + obj.uy(x,x1).^2));
        obj.x =   @(x,x1)      x    + obj.unx(x,x1); % shifted point x
        obj.y =   @(x,x1) road.y(x) + obj.uny(x,x1); % shifted point y
    end

    function obj = init_lane(road,off)
        
        % offset an array by off (lanes)
        obj.off = off;                   % fixed offset
        obj.ux =  @(x) -diff(road.y(x)); % x derivative
        obj.uy =  @(x) diff(x);          % y derivative
        obj.unx = @(x,off)...            % perpendiculate vector x
            (obj.ux(x).*off)./(sqrt(obj.ux(x).^2 + obj.uy(x).^2));
        obj.uny = @(x,off)...            % perpendiculate vector y
            (obj.uy(x).*off)./(sqrt(obj.ux(x).^2 + obj.uy(x).^2));
        obj.x =   @(x)...                % shifted vector x
            x + [obj.unx(x(1:2),obj.off) obj.unx(x,obj.off)];
        obj.y =   @(x)...                % shifted vector y
            road.y(x) + [obj.uny(x(1:2),obj.off) obj.uny(x,obj.off)];
    end

    function [lane,road_edges] = init_lanes(road)
        
        off_0 = 1.25;   % car half-width

        edge_c = init_lane(road,off_0);     % road center line
        edge_l = init_lane(road,5*off_0);   % road left line
        edge_r = init_lane(road,-3*off_0);  % road right line
        % concatenate lines
        road_edges.x = @(x) [edge_l.x(x) nan edge_c.x(x) nan edge_r.x(x)];
        road_edges.y = @(x) [edge_l.y(x) nan edge_c.y(x) nan edge_r.y(x)];
        
        lane_l = init_lane(road,3*off_0);   % lane left line
        lane_r = init_lane(road,-off_0);    % lane right line
        % concatenate lines
        lane.x = @(x) [lane_l.x(x) nan lane_r.x(x)];
        lane.y = @(x) [lane_l.y(x) nan lane_r.y(x)];
    end

    function [road_area, X, Y]= init_road_area(xl,yl)
        n = 240;
        x = linspace(xl(1),xl(2),n);
        y = linspace(yl(1),yl(2),n);
        [X,Y] = meshgrid(x,y);        
        road_area.X = X;
        road_area.Y = Y;
    end
    
    function obj = init_ego(x_t, road)
        obj = [];                % ego properties
        obj.v = 5;              % (m/s) ego speed
        obj.x = @(t,x_1) x_t(obj.v,t,x_1); % (m) ego x position;
        obj.y = @(x) road.y(x); % (m) ego y position
        obj.x_1 = 0;            % (m) ego's last x
        % ego cube (visualization)
        obj.cube.dimensions = [4 1.9 1.6];
        obj.cube.center = [0 0 0];
        obj.cube.theta = 0;
        obj.cube = init_cube(obj.cube);
        % set sensor field od view properties
        obj.sensor.n = 4;              % number of sensors
        obj.sensor.fov.range = 80;     % sensor range
        obj.sensor.fov.theta = 150;    % sensor angular range
        % initialize sensors
        obj.sensor.theta(1:obj.sensor.n) =... % sensor orientation
            45 + 360/obj.sensor.n*(0:(obj.sensor.n-1));
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
        obj.sensor.key = {'FL', 'RL', 'RR', 'FR'};
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
    
    function obj = init_mov_objects(n, direction, x_t, ego, road, dt)        
        % moving objects
        obj.n = n;
        obj.v = direction*ego.v*(rand(n,1)+0.5); % (m/s) moving object speed
        obj.t0 = ...            % random starting position
            rand(n,1)*diff(road.x([1 end]))./obj.v;
        obj.off = -2.5*(1:n)';
        obj.lane = init_mov_lane(road, obj.off);
        obj.x_t = @(t,x_1)...     % (m) moving object x position;
            x_t(obj.v,t+obj.t0,x_1); 
        obj.x = @(t,dt,x_1) obj.lane.x(obj.x_t(t,x_1),obj.x_t(t+dt,x_1)); % (m) moving object y position
        obj.y = @(t,dt,x_1) obj.lane.y(obj.x_t(t,x_1),obj.x_t(t+dt,x_1)); % (m) moving object y position
        obj.x_1 = zeros(n,1); % (m) moving object last x
        % object cube (visualization)
        for ii = 1 : n
            obj.cube(ii).dimensions = [4 1.9 1.6];
            obj.cube(ii).theta = [];
            obj.cube(ii).x = [];
            obj.cube(ii).y = [];
            obj.cube(ii).z = [];
            obj.cube(ii).idx = [];
        end
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

    function [interface,road,lane,road_edge,road_area,ego,stand,mov,onc] =...
            init_plots(interface,road,lane,road_edge,road_area,ego,stand,mov,onc)
        set(interface.main_figure.ax_static,... % static axes limits
            'xlim', [road.x(1) road.x(end)],...
            'ylim', [-1 1]*road.T/2);
        xylim =...
            get(interface.main_figure.sliders.ax_dynamic_zoom,'max')  -...
            get(interface.main_figure.sliders.ax_dynamic_zoom,'value')+...
            get(interface.main_figure.sliders.ax_dynamic_zoom,'min');
        set(interface.main_figure.ax_dynamic,...% dynamic axes limits
            'xlim', [-1 1]*xylim,...
            'ylim', [-1 1]*xylim,...
            'view', [0,90],...
            'projection', 'orthographic');
        set(interface.main_figure.sliders.ax_dynamic_rot,'value',0)
        set(interface.main_figure.sliders.ax_dynamic_tilt,'value',90)
        ax_dynamic_perspective_Callback(...
            interface.main_figure.sliders.ax_dynamic_ypan)

        % initialze common variables
        r_x = road.x;           % road x array
        r_y = road.y(r_x);      % road y array
        l_x = lane.x(r_x);      % lane x array
        l_y = lane.y(r_x);      % lane y array
        re_x = road_edge.x(r_x);      % road edge x array
        re_y = road_edge.y(r_x);      % road edge y array
        
        e_x = ego.x(0,ego.x_1); % ego x position
        e_y = ego.y(e_x);       % ego y position
        
        s_x = stand.x;          % standing object x array
        s_y = stand.y;          % standing object y array
        % find objects standing in the road
        [ra_l,ra_r] = find_road_points(re_x,re_y,ego,e_y,s_x);
        % remove objects standing in the road
        stand.x(logical((ra_r < s_y).*(s_y < ra_l))) = [];
        stand.y(logical((ra_r < s_y).*(s_y < ra_l))) = [];
        
        % initialize road area
        xl = [0 r_x(end)];
        yl = [-1 1]*road.T/2;
        road_area = find_road_area(...
            interface,road_area,re_x,re_y,ego,e_y,xl,yl);

        % static axes plots:
        road_area.m.static = surf(...
            road_area.X(1,:),road_area.Y(:,1),road_area.map,...
            'edgecolor','none',...
            'facecolor',[50 150 0]/270,...
            'facealpha',0.5,...
            'parent',interface.main_figure.ax_static);
        
        road.m.static = plot(interface.main_figure.ax_static,...
            r_x, r_y,'w','linewidth',2,'visible','off');
        lane.m.static = plot(interface.main_figure.ax_static,...
            l_x, l_y,'--w','linewidth',2);
        road_egde.m.static = plot(interface.main_figure.ax_static,...
            re_x, re_y,'w','linewidth',2);
        ego.m.static =  plot(interface.main_figure.ax_static,...
            0,0,'or','linewidth',2);
        stand.m.static = plot(interface.main_figure.ax_static,...
            stand.x, stand.y,'og','linewidth',2);
        mov.m.static =  plot(interface.main_figure.ax_static,...
            0,0,'o','color',[1 0.5 0],'linewidth',2);
        onc.m.static =  plot(interface.main_figure.ax_static,...
            0,0,'oc','linewidth',2);        
       
        % dynamic axes plots
        %   road area
        road_area.m.dynamic = surf(...
            [0 0],[0 0],zeros(2),...
            'edgecolor','none',...
            'facecolor',[50 150 0]/270,...
            'facealpha',0.5,...
            'parent',interface.main_figure.ax_dynamic);  

        % sensor fov
        for ii = 1 : ego.sensor.n
            % dynamic axes
            ego.sensor.m(ii) = patch(... % draw sensor FoV
                ego.sensor.fov.draw.circ{ii}(:,2),...
                ego.sensor.fov.draw.circ{ii}(:,1),...
                'w','FaceAlpha',.15,...
                'visible','off',...
                'parent', interface.main_figure.ax_dynamic);
            % sensor specific FoV
            ego.sensor.s(ii) = patch(... % draw sensor FoV
                ego.sensor.fov.draw.fov(:,2),...
                ego.sensor.fov.draw.fov(:,1),...
                'w','FaceAlpha',.3,...
                'parent', interface.main_figure.ax_sensor(ii));
            ego.sensor.d{ii}.road = plot(...
                interface.main_figure.ax_sensor(ii),...
                0,0,'w','linewidth',2);
            ego.sensor.d{ii}.stand = plot(...
                interface.main_figure.ax_sensor(ii),...
                0,0,'og','linewidth',2);
            ego.sensor.d{ii}.mov = plot(...
                interface.main_figure.ax_sensor(ii),...
                0,0,'o','color',[1 0.5 0],'linewidth',2);
            ego.sensor.d{ii}.onc = plot(...
                interface.main_figure.ax_sensor(ii),...
                0,0,'oc','linewidth',2);
        end
              
        %   simulated objects
        road.m.dynamic = plot(interface.main_figure.ax_dynamic,...
            0,0,'w','linewidth',2,'visible','off');
        lane.m.dynamic = plot(interface.main_figure.ax_dynamic,...
            0,0,'--w','linewidth',2);
        road_edge.m.dynamic = plot(interface.main_figure.ax_dynamic,...
            0,0,'w','linewidth',2);
        ego.m.dynamic = patch(...           % draw ego cube
            ego.cube.x(ego.cube.idx),...
            ego.cube.y(ego.cube.idx),...
            ego.cube.z(ego.cube.idx),...
            'r','facealpha',0.5,...
            'parent',interface.main_figure.ax_dynamic);
        stand.m.dynamic = plot(interface.main_figure.ax_dynamic,...
            0,0,'og','linewidth',2);
        mov.m.dynamic =  plot(interface.main_figure.ax_dynamic,...
            0,0,'o','color',[1 0.5 0],'linewidth',2);
        for ii = 1 : onc.n
            onc.m.dynamic(ii) = patch(...       % draw moving object cubes
                0,0,0,...
                'c','facealpha',0.5,...
                'parent',interface.main_figure.ax_dynamic);
        end
        %   user-detected objects
        stand.m.dynamic_user = plot(interface.main_figure.ax_dynamic,...
            0,0,'sg','visible','off','markersize',15);
        mov.m.dynamic_user = plot(interface.main_figure.ax_dynamic,...
            0,0,'s','color',[1 0.5 0],'visible','off','markersize',15);
        onc.m.dynamic_user = plot(interface.main_figure.ax_dynamic,...
            0,0,'sc','visible','off','markersize',15);
        
        interface.main_figure.ax_init = true;
    end

    % ***************************** interface *****************************
    
    function update_plot_static(...
            ego,ego_x,ego_y,mov,mov_x,mov_y,onc,onc_x,onc_y)
        % update static axes
        set(ego.m.static,'xData',ego_x,'yData',ego_y)
        set(mov.m.static,'xData',mov_x,'yData',mov_y)
        set(onc.m.static,'xData',onc_x,'yData',onc_y)
    end

    function update_plot_dynamic(...
            road,road_x,road_y,...
            lane,lane_x,lane_y,...
            road_edge,road_edge_x,road_edge_y,...
            road_area,road_area_x,road_area_y,...
            ego,stand,stand_x,stand_y,mov,mov_x,mov_y,onc,onc_x,onc_y)
        
        % update dynamic plot data
        set(road.m.dynamic, 'xData',road_y, 'yData',road_x)
        set(lane.m.dynamic, 'xData',lane_y, 'yData',lane_x)
        set(road_edge.m.dynamic, 'xData',road_edge_y, 'yData',road_edge_x)
        set(road_area.m.dynamic, 'xData',road_area_y, 'yData',road_area_x,...
            'cdata',road_area.map,'zdata',road_area.map)
        set(stand.m.dynamic,'xData',stand_y,'ydata',stand_x);
        set(mov.m.dynamic,  'xData',mov_y,  'yData',mov_x)
        for ii = 1 : onc.n
            set(onc.m.dynamic(ii),... % draw moving object cubes
                'xData',onc.cube(ii).y(onc.cube(ii).idx),...
                'yData',onc.cube(ii).x(onc.cube(ii).idx),...
                'zData',onc.cube(ii).z(onc.cube(ii).idx));
        end
        
        % update sensor-specific data
        for ii = 1 : ego.sensor.n
            set(ego.sensor.d{ii}.road,...
                'xData',ego.sensor.data(ii).road(:,2),...
                'yData',ego.sensor.data(ii).road(:,1))
            set(ego.sensor.d{ii}.stand,...
                'xData',ego.sensor.data(ii).stand(:,2),...
                'yData',ego.sensor.data(ii).stand(:,1))
            set(ego.sensor.d{ii}.mov,...
                'xData',ego.sensor.data(ii).mov(:,2),...
                'yData',ego.sensor.data(ii).mov(:,1))
            set(ego.sensor.d{ii}.onc,...
                'xData',ego.sensor.data(ii).onc(:,2),...
                'yData',ego.sensor.data(ii).onc(:,1))
        end
    end
    
    function init_interface()
        % try to close already opened figure, if existent
        try
            f = evalin('base','sim_world_data.interface.main_figure.f');
            close(f)
            delete(f)
        catch
            % nothign
        end
        
        interface = [];
        interface.colors.background       = [1 1 1]*0.25;
        interface.colors.plot_background  = [1 1 1]*0.4;
        interface.colors.plot_lines       = [1 1 1]*1;
        interface.colors.panel_background = [1 1 1]*0.3;
        interface.colors.font             = [1 1 1]*1;
        interface.colors.code_background  = [1 1 1]*0.2;
        interface.colors.code_font        = [0.5 1 0.5];
        interface.main_figure.f = figure(...
            'color',interface.colors.background,...
            'position',[1 31 1920 973],... %[450 80 1080 720]
            'CloseRequestFcn',@f_CloseRequestFcn);
        
        % ************************ playback panel ************************
        
        interface.main_figure.panels.playback_main = uipanel(...
            'parent',interface.main_figure.f,...
            'position',[0.15,0.03,0.84,0.94],...
            'title','Main playback');
        init_ui_style(interface.main_figure.panels.playback_main,interface)
        
        % static axes
        interface.main_figure.ax_static = axes(...
            'parent',interface.main_figure.panels.playback_main,...
            'position',[0.54 0.73 0.54 0.21]);
        init_axes_style(interface.main_figure.ax_static,interface)
        
        % dynamic axes
        interface.main_figure.ax_dynamic = axes(...
            'parent',interface.main_figure.panels.playback_main,...
            'position',[0.645 0.08 0.32 0.58],'xdir','reverse');
        init_axes_style(interface.main_figure.ax_dynamic,interface)
        
        % zoom slider
        interface.main_figure.sliders.ax_dynamic_zoom = uicontrol(...
            'parent',interface.main_figure.panels.playback_main,...
            'units','normalized',...
            'style','slider',...
            'min', 10,...
            'max', 100,...
            'value',90,...
            'position', [0.645 0.67 0.33 0.02],...
            'callback',@ax_dynamic_perspective_Callback);
        init_ui_style(...
            interface.main_figure.sliders.ax_dynamic_zoom,interface)
        
        % rotation slider
        interface.main_figure.sliders.ax_dynamic_rot = uicontrol(...
            'parent',interface.main_figure.panels.playback_main,...
            'units','normalized',...
            'style','slider',...
            'min', -180,...
            'max', 180,...
            'value',0,...
            'position', [0.645 0.05 0.33 0.02],...
            'callback',@ax_dynamic_perspective_Callback);
        init_ui_style(...
            interface.main_figure.sliders.ax_dynamic_rot,interface)
        
        % tilt slider
        interface.main_figure.sliders.ax_dynamic_tilt = uicontrol(...
            'parent',interface.main_figure.panels.playback_main,...
            'units','normalized',...
            'style','slider',...
            'min', 0,...
            'max', 90,...
            'value',90,...
            'position', [0.98 0.08 0.012 0.58],...
            'callback',@ax_dynamic_perspective_Callback);
        init_ui_style(...
            interface.main_figure.sliders.ax_dynamic_tilt,interface)
        
        % y pan slider
        interface.main_figure.sliders.ax_dynamic_ypan = uicontrol(...
            'parent',interface.main_figure.panels.playback_main,...
            'units','normalized',...
            'style','slider',...
            'min', -0.5,...
            'max', 0.5,...
            'value',0,...
            'position', [0.62 0.08 0.012 0.58],...
            'callback',@ax_dynamic_perspective_Callback);
        init_ui_style(...
            interface.main_figure.sliders.ax_dynamic_ypan,interface)
        
        % sensor axes
        w = 0.29;       % axes width
        h = 0.21;        % axes height
        off_x = 0.02;   % x offset
        off_y = 0.05;   % y offset
        xy_off = [0       0 w+off_x w+off_x;...
                  h+off_y 0 0       h+off_y]';
        ttl = {'FL','RL','RR','FR'};
        for ii = 1 : 4
            interface.main_figure.ax_sensor(ii) = axes(...
                'parent',interface.main_figure.panels.playback_main,...
                'position',[0.02+xy_off(ii,1) 0.05+xy_off(ii,2) w h],...
                'xdir','reverse');
            init_axes_style(interface.main_figure.ax_sensor(ii),interface)
            title(ttl{ii},'color','w')
        end
        
        % user coding text box
        interface.main_figure.edits.user_code = uicontrol(...
            'parent',interface.main_figure.panels.playback_main,...
            'units','normalized',...
            'style','edit',...
            'string','Error: Failed to load file',...
            'position', [0.01, 0.55, 0.6, 0.40],...
            'max',100,...
            'HorizontalAlignment','left',...
            'backgroundcolor',interface.colors.code_background,...
            'foregroundcolor',interface.colors.code_font,...
            'fontsize',11,...
            'fontname','monospaced');
        
        % choose function dropdown menu
        interface.main_figure.popups.user_code_selectFunction = uicontrol(...
            'parent',interface.main_figure.panels.playback_main,...
            'units','normalized',...
            'style','popup',...
            'string',{'Select function','Reconstruct 360'},...
            'value',2,...
            'position', [0.01, 0.96, 0.09, 0.03],...
            'callback',@user_code_selectFunction_Callback);
        init_ui_style(...
            interface.main_figure.popups.user_code_selectFunction,interface)
        
        % filename config
        interface.files.reconstruct_360_space = 'reconstruct_360_space.m';
        
        % load file to user code textbox
        sim_world_data.interface = interface;
        assignin('base','sim_world_data',sim_world_data) % store in base ws
        user_code_selectFunction_Callback([]);
        interface = evalin('base','sim_world_data.interface'); % recover from base ws
        
        % load file button
        interface.main_figure.buttons.user_code_load = uicontrol(...
            'parent',interface.main_figure.panels.playback_main,...
            'units','normalized',...
            'style','pushbutton',...
            'string','Load',...
            'position', [0.11, 0.96, 0.05, 0.03],...
            'callback',@user_code_load_Callback);
        init_ui_style(...
            interface.main_figure.buttons.user_code_load,interface)
        
        % save file button
        interface.main_figure.buttons.user_code_save = uicontrol(...
            'parent',interface.main_figure.panels.playback_main,...
            'units','normalized',...
            'style','pushbutton',...
            'string','Save',...
            'enable','off',...
            'position', [0.17, 0.96, 0.05, 0.03],...
            'callback',@user_code_save_Callback);
        init_ui_style(...
            interface.main_figure.buttons.user_code_save,interface)
        
        % save as file button
        interface.main_figure.buttons.user_code_saveas = uicontrol(...
            'parent',interface.main_figure.panels.playback_main,...
            'units','normalized',...
            'style','pushbutton',...
            'string','Save as',...
            'position', [0.23, 0.96, 0.05, 0.03],...
            'callback',@user_code_saveas_Callback);
        init_ui_style(...
            interface.main_figure.buttons.user_code_saveas,interface)
        
        % ************************ control panel ************************
        
        interface.main_figure.panels.controls_main = uipanel(...
            'parent',interface.main_figure.f,...
            'position',[0.01,0.03,0.13,0.94],...
            'title','Main controls');
        init_ui_style(interface.main_figure.panels.controls_main,interface)
        
        % Start / pause / resume button
        interface.main_figure.buttons.controls_main_play = uicontrol(...
            'parent',interface.main_figure.panels.controls_main,...
            'units','normalized',...
            'style','toggle',...
            'string','Start',...
            'position', [0.05, 0.9, 0.4, 0.035],...
            'callback',@controls_main_play_Callback);
        init_ui_style(...
            interface.main_figure.buttons.controls_main_play,interface)
        
        % Reset button
        interface.main_figure.buttons.controls_main_reset = uicontrol(...
            'parent',interface.main_figure.panels.controls_main,...
            'units','normalized',...
            'style','pushbutton',...
            'string','Reset',...
            'position', [0.55, 0.9, 0.4, 0.035],...
            'callback',@controls_main_reset_Callback);
        init_ui_style(...
            interface.main_figure.buttons.controls_main_reset,interface)
        
        % ******************** sensor selection panel ********************
        interface.main_figure.panels.controls_main_showSensor = uipanel(...
            'parent',interface.main_figure.panels.controls_main,...
            'position',[0.03,0.77,0.94,0.1],...
            'title','Show reconstructued sensors');
        init_ui_style(...
            interface.main_figure.panels.controls_main_showSensor,...
            interface)
        
        % choose sensor menu
        interface.main_figure.popups.controls_main_showSensor = uicontrol(...
            'parent',interface.main_figure.panels.controls_main_showSensor,...
            'units','normalized',...
            'style','popup',...
            'string',{'Hide all','Select','Show all'},...
            'value',1,...
            'position', [0.03, 0.9, 0.96, 0.03],...
            'callback',@controls_main_showSensor_Callback);
        init_ui_style(...
            interface.main_figure.popups.controls_main_showSensor,interface)
        
        % choose sensor checkboxes
        interface.main_figure.checkboxes.controls_main_showSensor_FL = uicontrol(...
            'parent',interface.main_figure.panels.controls_main_showSensor,...
            'units','normalized',...
            'style','checkbox',...
            'string',{'Front Left (FL)'},...
            'value',0,...
            'enable','off',...
            'position', [0.03, 0.35, 0.46, 0.15]);
        init_ui_style(...
            interface.main_figure.checkboxes.controls_main_showSensor_FL,...
            interface)
        interface.main_figure.checkboxes.controls_main_showSensor_FR = uicontrol(...
            'parent',interface.main_figure.panels.controls_main_showSensor,...
            'units','normalized',...
            'style','checkbox',...
            'string',{'Front Right (FR)'},...
            'value',0,...
            'enable','off',...
            'position', [0.53, 0.35, 0.46, 0.15]);
        init_ui_style(...
            interface.main_figure.checkboxes.controls_main_showSensor_FR,...
            interface)
        interface.main_figure.checkboxes.controls_main_showSensor_RL = uicontrol(...
            'parent',interface.main_figure.panels.controls_main_showSensor,...
            'units','normalized',...
            'style','checkbox',...
            'string',{'Rear Left (RL)'},...
            'value',0,...
            'enable','off',...
            'position', [0.03, 0.10, 0.46, 0.15]);
        init_ui_style(...
            interface.main_figure.checkboxes.controls_main_showSensor_RL,...
            interface)
        interface.main_figure.checkboxes.controls_main_showSensor_RR = uicontrol(...
            'parent',interface.main_figure.panels.controls_main_showSensor,...
            'units','normalized',...
            'style','checkbox',...
            'string',{'Rear Right (RR)'},...
            'value',0,...
            'enable','off',...
            'position', [0.53, 0.10, 0.46, 0.15]);
        init_ui_style(...
            interface.main_figure.checkboxes.controls_main_showSensor_RR,...
            interface)
        
        % assign interface to base workspace
        sim_world_data.interface = interface;
        assignin('base','sim_world_data',sim_world_data)
        
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

    function init_ui_style(h, interface)
        set(h,...
            'backgroundcolor',interface.colors.panel_background,...
            'foregroundcolor',interface.colors.font)
    end
    
    function [ra_l,ra_r] = find_road_points(...
            road_edge_x,road_edge_y,ego,ego_y,p_x)
        %   find left edge
        splits = find(isnan(road_edge_x)) + [-1 1];
        ra_l = [road_edge_x(1:splits(1))' road_edge_y(1:splits(1))'];
        
        %   find right edge
        ra_r = [road_edge_x(splits(2):end)' road_edge_y(splits(2):end)'];
        
        %   interpolate y coordinates
        ra_l = interp1(ra_l(:,1),ra_l(:,2),p_x+ego.x_1)-ego_y;
        ra_r = interp1(ra_r(:,1),ra_r(:,2),p_x+ego.x_1)-ego_y;
    end

    function road_area = find_road_area(...
            interface,road_area,road_edge_x,road_edge_y,ego,ego_y,xl,yl)
        
        if nargin() < 7
            % initialize road area
            xl = get(interface.main_figure.ax_dynamic,'xlim');
            xl = mean(xl) + [-1 1]*diff(xl)*1.1; % expand to fit y span
            [~,road_area.X, road_area.Y] = init_road_area(xl,xl);
        else
            [~,road_area.X, road_area.Y] = init_road_area(xl,yl);
        end
        
        [ra_l,ra_r] = find_road_points(...
            road_edge_x,road_edge_y,ego,ego_y,...
                road_area.X(1,:));
        
        road_area.map = -0.2*ones(size(road_area.X));
        % subtract road from map for transparency
        road_area.map(logical(...
            (bsxfun(@times,ra_r,ones(size(road_area.X)))<road_area.Y).*...
            (road_area.Y<bsxfun(@times,ra_l,ones(size(road_area.X))))...
            )) = nan;
        
    end

    function init_user_code(interface, fname)
        fID = fopen(fname);         % file ID
        tline = fgetl(fID);         % first line
        fTxt = {};                  % fle tezt
        while ischar(tline)
            fTxt{end+1} = tline;
            tline = fgetl(fID);
        end
        fclose(fID);
        
        % assign text to user code textbox
        set(interface.main_figure.edits.user_code,...
            'string', fTxt)
    end

    function save_user_code(interface, fname)
        fID = fopen(fname,'w');         % file ID
        eTxt = get(interface.main_figure.edits.user_code,'string');
        
        fTxt = '';
        
        for ii = 1 : length(eTxt)
           fprintf(fID,'%s\n', eTxt{ii});
        end

        fclose(fID);
    end

    function reset_plots(interface)
        cla(interface.main_figure.ax_static);
        init_axes_style(interface.main_figure.ax_static, interface)
        cla(interface.main_figure.ax_dynamic);
        init_axes_style(interface.main_figure.ax_dynamic, interface)
        for ii = 1 : length(interface.main_figure.ax_sensor)
            cla(interface.main_figure.ax_sensor(ii));
            init_axes_style(interface.main_figure.ax_sensor(ii), interface)
        end
    end

    function output_sim_data(...
            interface,road,lane,road_edge,road_area,...
            ego,stand,mov,onc,t,dt,road_tail)
        sim_world_data.road          = road;
        sim_world_data.lane          = lane;
        sim_world_data.road_edge     = road_edge;
        sim_world_data.road_area     = road_area;
        sim_world_data.ego           = ego;
        sim_world_data.stand         = stand;
        sim_world_data.mov           = mov;
        sim_world_data.onc           = onc;
        sim_world_data.interface     = interface;
        sim_world_data.sim.t         = t;
        sim_world_data.sim.dt        = dt;
        sim_world_data.sim.road_tail = road_tail;
        assignin('base','sim_world_data',sim_world_data)
    end

    function [interface,road,lane,road_edge,road_area,...
            ego,stand,mov,onc,t,dt,road_tail] =...
            read_sim_data()
        sim_world_data = evalin('base','sim_world_data');
        road      = sim_world_data.road;
        lane      = sim_world_data.lane;
        road_edge = sim_world_data.road_edge;
        road_area = sim_world_data.road_area;
        ego       = sim_world_data.ego;
        stand     = sim_world_data.stand;
        mov       = sim_world_data.mov;
        onc       = sim_world_data.onc;
        interface = sim_world_data.interface;
        t         = sim_world_data.sim.t;
        dt        = sim_world_data.sim.dt;
        road_tail = sim_world_data.sim.road_tail;
    end
    
    function cube = init_cube(cube,dimensions,center,theta)
        
        rd = @(x,y,t) [x(:),y(:)]*[cosd(t) -sind(t) ;... % rotation matrix
                                   sind(t)  cosd(t)];
        if nargin == 1
            dimensions = cube.dimensions;   % cube dimensions
            center = cube.center;           % cube center position
            theta = cube.theta;             % cube orientaton
        end
        % basic cube geometry
        coord = [-1 -1 0;1 -1 0;1 1 0;-1 1 0;-1 -1 2;1 -1 2;1 1 2;-1 1 2];
        % apply input properties
        coord = bsxfun(@times,coord/2,dimensions);
        coord = bsxfun(@plus,...
            [rd(coord(:,1),coord(:,2),theta+90),coord(:,3)],...
            center);
        % vertex indices
        cube.idx = [4 8 5 1 4; 1 5 6 2 1; 2 6 7 3 2;...
                    3 7 8 4 3; 5 8 7 6 5; 1 4 3 2 1]';
        cube.x = coord(:,1);    % cube x coordinates
        cube.y = coord(:,2);    % cube y coordinates
        cube.z = coord(:,3);    % cube z coordinates
    end
    
    function draw_cube
        
    end

% ************************** interface callbacks **************************

    function controls_main_play_Callback(source,~)
        interface = evalin('base','sim_world_data.interface');
        switch get(source,'value')
            case 1
                set(source,'string','Pause');
                set(interface.main_figure.buttons.controls_main_reset,...
                    'enable','off')
            case 0
                set(source,'string','Resume');
                set(interface.main_figure.buttons.controls_main_reset,...
                    'enable','on')
        end
        [interface,road,lane,road_edge,road_area,ego,stand,mov,onc,t,dt,road_tail] =...
            read_sim_data();
        master_run(interface,road,lane,road_edge,road_area,...
            ego,stand,mov,onc,road_tail,dt,t);
    end

    function ax_dynamic_perspective_Callback(source,~)
        
        % get interface from base workspace
        interface = evalin('base','sim_world_data.interface');
        vv = get(interface.main_figure.ax_dynamic,'view');  % axes view
        value = get(source,'value');    % slider value
        vmin = get(source,'min');       % slider min value
        vmax = get(source,'max');       % slider max value
        vinv = (vmax-value+vmin);       % inverted value
        switch source
            case interface.main_figure.sliders.ax_dynamic_zoom
                yoff = 2* get(... % y offset introduced by y pan slider
                    interface.main_figure.sliders.ax_dynamic_ypan,'value');
                set(interface.main_figure.ax_dynamic,...
                    'xlim',[-1 1]*vinv,...
                    'ylim',([-1 1] - yoff)*vinv)
                controls_main_play_Callback(...
                    interface.main_figure.buttons.controls_main_play)
            case interface.main_figure.sliders.ax_dynamic_rot
                set(interface.main_figure.ax_dynamic,...
                    'view',[vinv vv(2)],...
                    'projection','perspective')
            case interface.main_figure.sliders.ax_dynamic_tilt
                set(interface.main_figure.ax_dynamic,...
                    'view',[vv(1) value],...
                    'projection','perspective')
            case interface.main_figure.sliders.ax_dynamic_ypan
                yl = get(interface.main_figure.ax_dynamic,'ylim'); % ylim
                set(interface.main_figure.ax_dynamic,'ylim',...
                    [-1 1]*diff(yl)/2 + diff(yl)*vinv)
        end
    end

    function func = ui_get_user_code_Function(source)
        
        % load interface from base workspace
        interface = evalin('base','sim_world_data.interface');
        
        % get selected function
        if isempty(source),
            source = interface.main_figure.popups.user_code_selectFunction;
        end
        funcs = get(source,'string');
        val = get(source, 'value');
        
        func = funcs{val};
    end

    function controls_main_reset_Callback(~,~)
        % get interface from base workspace
        interface = evalin('base','sim_world_data.interface');
        
        % stop running simulation
        set(interface.main_figure.buttons.controls_main_play,...
            'value',0,...
            'string','Start')
        
        % initialize simulation time
        dt = 0.05;  % (s) time resolution
        t = 0;      % (s) time
    
        % initialize road map
        road = init_road();
        road_tail = road.x(round(end/2));	% (m) road tail behind ego

        % motion equation for ego and moving objects
        x_t = @(v,t,x_1) v.*t - road.x(end).*floor(x_1/road.x(end));

        % initialize ego
        ego = init_ego(x_t, road);

        % initialize standing objects
        stand_n_objects = 40;
        stand = init_stand(stand_n_objects,road);

        % initialize moving and oncoming objects
        mov_n_objects = 2;      % number of moving objects
        mov = init_mov_objects(mov_n_objects-1, 1,x_t,ego,road,dt);% moving obj
        onc = init_mov_objects(mov_n_objects,-1,x_t,ego,road,dt);% oncoming obj
        
        % initialize lanes and road edges
        [lane,road_edges] = init_lanes(road);
        
        % initialize road area
        road_area = init_road_area(...
            [0 road.x(end)],[-road.x(end)/2 road.x(end)/2]);
        
        % reset plots if already existent
        if isfield(interface.main_figure,'ax_init') % not yet initialized
            reset_plots(interface);
        end
        
        % initialize simulation animation variables
        [interface, road,lane,road_edges,road_area,ego,stand,mov,onc] = ...
                    init_plots(interface,road,lane,road_edges,road_area,...
                    ego,stand,mov,onc);
        
        % output data to base workspace
        output_sim_data(...
            interface,road,lane,road_edges,road_area,...
            ego,stand,mov,onc,t,dt,road_tail)
        
        % run one simulation cycle to finish initialization
        controls_main_play_Callback(...
            interface.main_figure.buttons.controls_main_play)
        
    end
    
    function user_code_selectFunction_Callback(source,~)
        
        % load interface from base workspace
        sim_world_data = evalin('base','sim_world_data');
        interface = sim_world_data.interface;
        
        proceed = true;
        switch ui_get_user_code_Function(source)
            case 'Reconstruct 360'
                fname = interface.files.reconstruct_360_space;
            otherwise
                proceed = false;
        end
        
        if proceed
            % load code from file
            init_user_code(interface, fname)

            % assign to base workspace
            sim_world_data.interface = interface;
            assignin('base','sim_world_data', sim_world_data)
        end
    end

    function user_code_load_Callback(~,~)
        %prompt user to select file
        [fname, pname] = uigetfile('*.m');
        
        if fname
            % join path to filename
            fname = [pname '\' fname];
            
            % load interface from base workspace
            sim_world_data = evalin('base','sim_world_data');
            interface = sim_world_data.interface;

            switch ui_get_user_code_Function([])
                case 'Reconstruct 360'
                    interface.files.reconstruct_360_space = fname;
            end

            % update interface in base workspace
            
            sim_world_data.interface = interface;
            assignin('base','sim_world_data',sim_world_data)

            user_code_selectFunction_Callback()
        end
        
    end

    function user_code_save_Callback(~,~)
        
        % load interface from base workspace
        interface = evalin('base','sim_world_data.interface');
        fname = interface.files.reconstruct_360_space;
        
        % save file
        save_user_code(interface, fname)
    end

    function user_code_saveas_Callback(~,~)
        %prompt user to select file
        [fname, pname] = uiputfile('*.m');
        
        if fname
            % join path to filename
            fname = [pname '\' fname];
            
            % load interface from base workspace
            sim_world_data = evalin('base','sim_world_data');
            interface = sim_world_data.interface;

            switch ui_get_user_code_Function([])
                case 'Reconstruct 360'
                    interface.files.reconstruct_360_space = fname;
            end
            
            % save file
            save_user_code(interface, fname)
            
            set(interface.main_figure.buttons.user_code_save,'enable','on');

            % update interface in base workspace
            sim_world_data.interface = interface;
            assignin('base','sim_world_data',sim_world_data)
        end
    end
    
    function controls_main_showSensor_Callback(source,~)
        % get interface from base workspace
        interface = evalin('base','sim_world_data.interface');
        
        str = get(source,'string');
        val = get(source,'value');
        option = str{val};
       
        switch option
            case 'Hide all' % hide all sensor detections
                set(interface.main_figure.checkboxes.controls_main_showSensor_FL,...
                    'value',0,'enable','off')
                set(interface.main_figure.checkboxes.controls_main_showSensor_FR,...
                    'value',0,'enable','off')
                set(interface.main_figure.checkboxes.controls_main_showSensor_RL,...
                    'value',0,'enable','off')
                set(interface.main_figure.checkboxes.controls_main_showSensor_RR,...
                    'value',0,'enable','off')
            case 'Select'   % allow user selection
                set(interface.main_figure.checkboxes.controls_main_showSensor_FL,...
                    'enable','on')
                set(interface.main_figure.checkboxes.controls_main_showSensor_FR,...
                    'enable','on')
                set(interface.main_figure.checkboxes.controls_main_showSensor_RL,...
                    'enable','on')
                set(interface.main_figure.checkboxes.controls_main_showSensor_RR,...
                    'enable','on')
            case 'Show all'
                set(interface.main_figure.checkboxes.controls_main_showSensor_FL,...
                    'value',1,'enable','off')
                set(interface.main_figure.checkboxes.controls_main_showSensor_FR,...
                    'value',1,'enable','off')
                set(interface.main_figure.checkboxes.controls_main_showSensor_RL,...
                    'value',1,'enable','off')
                set(interface.main_figure.checkboxes.controls_main_showSensor_RR,...
                    'value',1,'enable','off')
        end
    end

    function f_CloseRequestFcn(source,~)
        set(source,'visible','off')
    end
    
end
