function sim_world()
    
    % initialize interface
    init_interface();
    
    % ******************* initialize object properties *******************
    controls_main_reset_Callback()
    
    % *********************** function definitions ***********************
    
    function master_run(interface,source,road,lane,road_edge,road_area,...
            ego,stand,mov,onc,road_tail,dt,t)
        
        override = true;    % override bool to implement do while
        
        while (get(interface.figures.main.buttons.controls_main_play,'value') ...
                || override)

            % run simulation
            [road,lane,road_edge,road_area,ego,stand,mov,onc,road_tail] = ...
                simulation_run(interface,t,dt,road,lane,road_edge,road_area,...
                ego,stand,mov,onc,road_tail);
            
            % fill inputs for user reconstruction
            [sensor_f, ego_f] = fill_input_reconstruct_360_space(...
                interface,ego,t,dt);
            
            % recover 360 degree coordinate space
            [obj_u,stand_u,mov_u,onc_u] = eval(...
                [interface.files.reconstruct_360_space.func,...
                '(sensor_f,ego_f)']);
            
            % calculate z coordinates for user detections
            [obj_u,stand_u,mov_u,onc_u] = calculate_user_z(...
                obj_u,stand_u,mov_u,onc_u,...
                road_area,ego,interface);
            
            % draw recovered coordinates
            if get(interface.figures.main.buttons.controls_main_play,'value')...
                    || source == interface.figures.main.buttons.controls_main_step
                update_plot_dynamic_user(...
                    ego,obj_u,stand,stand_u,mov,mov_u,onc,onc_u);
            end

            % increment time
            t = t + dt;
            pause(0.01)

            % output data to base workspace
            output_sim_data(...
                interface,road,lane,road_edge,road_area,...
                ego,stand,mov,onc,t,dt,road_tail)
            
            override = false;
        end
    end
    
    function [road,lane,road_edge,road_area,ego,stand,mov,onc,road_tail] =...
            simulation_run(interface,t,dt,...
            road,lane,road_edge,road_area,ego,stand,mov,onc,road_tail)
        
        % ****************** update data for static plot ******************
        ego_x =  ego.x(t,ego.x_1);
        ego_y =  ego.y(ego_x);
        ego.x_1 = ego.x(t,0);
        ego.y_1 = ego_y;
        
        mov_x =  mov.x(t,dt,mov.x_1);
        mov_y =  mov.y(t,dt,mov.x_1);
        mov.theta = atan2d(mov_y - mov.y(t-dt,dt,mov.x_1),...
            mov_x - mov.x(t-dt,dt,mov.x_1));
        mov.x_1 = mov.x(t,dt,0);
        
        onc_x =  onc.x(t,dt,onc.x_1);
        onc_y =  onc.y(t,dt,onc.x_1);
        onc.theta = atan2d(onc_y - onc.y(t-dt,dt,onc.x_1),...
            onc_x - onc.x(t-dt,dt,onc.x_1));
        onc.x_1 = onc.x(t,dt,0);
        
        road_x = road.x + ego.x_1-road_tail;
        road_y = road.y(road_x);
        
        lane_x = lane.x(road_x);
        lane_y = lane.y(road_x);
        
        road_edge_x = road_edge.x(road_x(1:5:end));
        road_edge_y = road_edge.y(road_x(1:5:end));
        
        % update road area
        road_area = find_road_area(...
            interface,road,road_area,road_edge_x,road_edge_y,ego,ego_y);
        
        % update static axis
        update_plot_static(ego,ego_x,ego_y,mov,mov_x,mov_y,onc,onc_x,onc_y)
        
        % ***************** update data for dynamic plot *****************
        
        % transform road coordinates
        [road_x,road_y,theta] = dynamic_transform_coordinates(...
            road_x,road_y,ego.x_1,ego.y(ego.x_1));
        ego.Dtheta = ego.theta - theta; % angular variation
        ego.theta = theta;              % ego orientation
        
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
        fov = [get(interface.figures.main.axes.dynamic,'ylim');... axes fov
            get(interface.figures.main.axes.dynamic,'xlim')]';
        stand = update_stand_object_cube(stand,stand_x,stand_y,theta,fov);
        
        % transform moving object corrdinates
        m_shift = road.x(end)*((ego_x-mov_x) > road.x(round(end/2))) +... 
                 -road.x(end)*((mov_x-ego_x) > road.x(round(end/2)));
        [mov_x,mov_y] = dynamic_transform_coordinates(...
            mov_x+m_shift,mov_y,ego_x,ego_y, theta);
        mov = update_mov_object_cube(mov,mov_x,mov_y,theta,fov);
        
        % transform oncoming object corrdinates
        o_shift = road.x(end)*((ego_x-onc_x) > road.x(round(end/2))) +... 
                 -road.x(end)*((onc_x-ego_x) > road.x(round(end/2)));
        [onc_x,onc_y] = dynamic_transform_coordinates(...
            onc_x+o_shift,onc_y,ego_x,ego_y, theta);
        onc = update_mov_object_cube(onc,onc_x,onc_y,theta,fov);
        
        % transform road map coordinates
        [ra_x,ra_y] = dynamic_transform_coordinates(...
                road_area.X(:),road_area.Y(:),0,0, theta);
        
        % reconstruct road area coordinates
        switch road_area.type
            case 'patch'
                road_area_x = ra_x;
                road_area_y = ra_y;
                road_area_z = [];
            case 'surf'
                road_area_x = reshape(ra_x,size(road_area.X));
                road_area_y = reshape(ra_y,size(road_area.Y));
                road_area_z = road_area.Z(...
                    road_area.X,ego.x_1,road_area.Y,ego_y);
        end
        
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
        update_plot_dynamic(...
            interface,...
            road,road_x,road_y,...
            lane,lane_x,lane_y,...
            road_edge,road_edge_x,road_edge_y,...
            road_area,road_area_x,road_area_y,road_area_z,...
            ego,...
            stand,stand_x,stand_y,...
            mov,mov_x,mov_y,...
            onc,onc_x,onc_y,fov)
    end

    function [sensor_f,ego_f] = fill_input_reconstruct_360_space(...
            interface,ego,t,dt)
        % copy selected data from the sensor
        sensor_f.n = ego.sensor.n;                    % number of sensors
        sensor_f.fov.range = ego.sensor.fov.range;    % fov dist range
        sensor_f.fov.theta = ego.sensor.fov.theta;    % fov angular range
        sensor_f.theta = ego.sensor.theta;            % fov orientation
        sensor_f.pos = ego.sensor.key;                % sensor position
        
        % path to handle of sensor fov drawing
        sensor_path = ...
            'interface.figures.main.checkboxes.controls_main_showSensor_';
        
        % copy detected object data
        for ii = 1 : ego.sensor.n
            sensor_switch = [sensor_path ego.sensor.key{ii}];
            if get(eval(sensor_switch),'value')
                sensor_f.active(ii) = true;
                sensor_f.data(ii).obj = ...   % bunch all sensor objects
                    [ego.sensor.data(ii).stand(...
                    ~isnan(ego.sensor.data(ii).stand(:,1)),:);...
                    ego.sensor.data(ii).mov(...
                    ~isnan(ego.sensor.data(ii).mov(:,1)),:);...
                    ego.sensor.data(ii).onc(...
                    ~isnan(ego.sensor.data(ii).onc(:,1)),:)]; 
                
                sensor_f.data(ii).stand = ...   % standing objects
                    ego.sensor.data(ii).stand(...
                    ~isnan(ego.sensor.data(ii).stand(:,1)),:); 
                sensor_f.data(ii).mov = ...     % moving objects
                    ego.sensor.data(ii).mov(...
                    ~isnan(ego.sensor.data(ii).mov(:,1)),:);
                sensor_f.data(ii).onc = ....    % oncoming objects
                    ego.sensor.data(ii).onc(...
                    ~isnan(ego.sensor.data(ii).onc(:,1)),:);
                set(ego.sensor.m(ii),'visible','on')
            else
                sensor_f.active(ii) = false;
                set(ego.sensor.m(ii),'visible','off')
            end
        end
        
        % rotation matrix (degrees)
        rd = @(x,y,t) [x(:),y(:)]*[cosd(t) -sind(t);... 
                                   sind(t)  cosd(t)]';
        ego_f.Dtheta = ego.Dtheta/pi*180;       % angular variation
        ego_x = ego.x(t,0);                     % current x position
        ego_x_1 = ego.x(t-dt,0);                % previous x position
        % calculate ego xy velocity
        v_xy = rd(ego_x-ego_x_1,ego.y(ego_x)-ego.y(ego_x_1),...
            (pi/2-ego.theta)/pi*180)/dt;
        ego_f.v_xy = v_xy([2 1]) .* [1 -1];
        ego_f.v = sqrt(ego_f.v_xy(1)^2 + ego_f.v_xy(2)^2); % ego speed
        ego_f.dt = dt;                          % cycle duration
    end

    function update_plot_dynamic_user(...
            ego,obj_u,stand,stand_u,mov,mov_u,onc,onc_u)
        
        % get interface from base workspace
        interface = evalin('base','sim_world_data.interface');
        
        if sum([...
                get(interface.figures.main.checkboxes.controls_main_showSensor_FL,'value'),...
                get(interface.figures.main.checkboxes.controls_main_showSensor_FR,'value'),...
                get(interface.figures.main.checkboxes.controls_main_showSensor_RL,'value'),...
                get(interface.figures.main.checkboxes.controls_main_showSensor_RR,'value')])
            if ~isempty(obj_u)
                set(ego.m.dynamic_user,...
                    'visible','on',...
                    'xdata',obj_u(:,2),'ydata',obj_u(:,1),'zdata',obj_u(:,3)+2)
            else
                set(ego.m.dynamic_user,'visible','off')
            end
            if ~isempty(stand_u)
                set(stand.m.dynamic_user,...
                    'visible','on',...
                    'xdata',stand_u(:,2),'ydata',stand_u(:,1),'zdata',stand_u(:,3)+2)
            else
                set(stand.m.dynamic_user,'visible','off')
            end
            if ~isempty(mov_u)
                set(mov.m.dynamic_user,...
                    'visible','on',...
                    'xdata',mov_u(:,2),'ydata',mov_u(:,1),'zdata',mov_u(:,3)+2)
            else
                set(mov.m.dynamic_user,'visible','off')
            end
            if ~isempty(onc_u)
                set(onc.m.dynamic_user,...
                    'visible','on',...
                    'xdata',onc_u(:,2),'ydata',onc_u(:,1),'zdata',onc_u(:,3)+2)
            else
                set(onc.m.dynamic_user,'visible','off')
            end
        else
            set(ego.m.dynamic_user,'visible','off')
            set(stand.m.dynamic_user,'visible','off')
            set(mov.m.dynamic_user,'visible','off')
            set(onc.m.dynamic_user,'visible','off')
        end
    end

    function [obj_u,stand_u,mov_u,onc_u] = calculate_user_z(obj_u,stand_u,mov_u,onc_u,...
                road_area,ego,interface)
            if ~isempty(obj_u)
                if get(interface.figures.main.sliders.ax_dynamic_tilt,'value')...
                        < 70
                    % calculate standing object z in their original position
                    %   1) cancel ego rotation
                    [obj_x,obj_y] = dynamic_transform_coordinates(...
                        obj_u(:,1),obj_u(:,2),0,0,-ego.theta);
                    [stand_x,stand_y] = dynamic_transform_coordinates(...
                        stand_u(:,1),stand_u(:,2),0,0,-ego.theta);
                    %   2) calculate road_area z in the object positions
                    obj_u = [obj_u road_area.Z(...
                        obj_x,ego.x_1,obj_y,ego.y_1)];
                    stand_u = [stand_u road_area.Z(...
                        stand_x,ego.x_1,stand_y,ego.y_1)];

                else % approximate standing object z to zero
                    obj_u = [obj_u zeros(size(obj_u,1),1)];
                    stand_u = [stand_u zeros(size(stand_u,1),1)];
                end

                % moving and oncoming objects must have z = 0
                mov_u = [mov_u zeros(size(mov_u,1),1)];
                onc_u = [onc_u zeros(size(onc_u,1),1)];
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
    
    function obj = init_road(interface)
        obj.T = 80;            % (m) road periodicity
        obj.x = 0:0.2:240;     % (m) road x array
        obj.y = @(x) ...       % (m) road y array
            obj.T/3*sin(2*pi/obj.T*x).*cos(2*pi/obj.T/3*x);
        
        % 3D terrain properties
        sliders = interface.figures.main.sliders; % interface sliders
        obj.terrain.n = 240;   % surf number of pixels
        obj.terrain.a = ...    % terrain dip amplitude
            get(sliders.controls_main_parameters_terrain_h,'value');
        obj.terrain.c = 50;    % terrain dip width
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

    function [road_area, X, Y] = init_road_area(xl,yl,road,type,x,ra_l,ra_r)
       
        % set default road_area type to surf
        if nargin() < 4 
            type = 'surf';
        end
                
        switch type 
            case 'patch'    % flat surface
                % nan-free range
                rg = ~logical(isnan(ra_r) + isnan(ra_l));
                % define lines around road edge
                %   define X and Y outputs
                X =...
                    [xl(1) x(rg) xl(2) xl(1)...
                     xl(1) x(rg) xl(2) xl(1)];
                Y =...
                    [yl(2) ra_l(rg) yl(2) yl(2)...
                     yl(1) ra_r(rg) yl(1) yl(1)];
                
                % store X and Y in raod_area struct 
                road_area.X = X;
                road_area.Y = Y;
                
                % dummy function for non-existent terrain topography
                road_area.Z = @(X,x0,Y,y0) []; 
            
            case 'surf'     % 3D terrain
                % X,T matrixes for surf plot
                [X,Y] = meshgrid(linspace(xl(1),xl(2),road.terrain.n),...
                                 linspace(yl(1),yl(2),road.terrain.n));        
                road_area.X = X;                    % x matrix
                road_area.Y = Y;                    % y matrix
                
                % terrain topography equations
                road_area.Z_ = @(X,Y)...            % 2D Gaussian
                    road.terrain.a*exp(-((Y-road.y(X))/road.terrain.c).^2);
                
                % subtract amplitude and apply 3.75 m correction
                road_area.Z = @(X,x0,Y,y0)...
                    road_area.Z_(X+x0,Y+y0-3.75) - road.terrain.a;
                
        end
        road_area.type = type;
    end
    
    function obj = init_ego(x_t, road)
        obj = [];                % ego properties
        obj.v = 10;              % (m/s) ego speed
        obj.x = @(t,x_1) x_t(obj.v,t,x_1); % (m) ego x position;
        obj.y = @(x) road.y(x); % (m) ego y position
        obj.x_1 = 0;            % (m) ego's last x
        obj.theta = 0;          % ego orientation
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

    function obj = init_stand(n,road,road_area)
        obj.n = n;  % standing (static) objects properties
        % range for standing objects
        rg_x = [min(road.x) max(road.x)]; 
        rg_y = [-1 1]*road.T ;
        % randomly generate standing object positions
        obj.x = rand(n,1)*diff(rg_x) + rg_x(1);
        obj.y = rand(n,1)*diff(rg_y) + rg_y(1); 
        obj.z = road_area.Z(obj.x,0,obj.y,0);
        % object cube (visualization)
        for ii = 1 : n
            obj.cube(ii).dimensions = [2 2 4];
            obj.cube(ii).theta = [];
            obj.cube(ii).x = [];
            obj.cube(ii).y = [];
            obj.cube(ii).z = [];
            obj.cube(ii).idx = [];
        end
    end
    
    function obj = init_mov_objects(n, direction, x_t, ego, road)        
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
        set(interface.figures.main.axes.static,... % static axes limits
            'xlim', [road.x(1) road.x(end)],...
            'ylim', [-1 1]*road.T/2);
        xylim =...
            get(interface.figures.main.sliders.ax_dynamic_zoom,'max')  -...
            get(interface.figures.main.sliders.ax_dynamic_zoom,'value')+...
            get(interface.figures.main.sliders.ax_dynamic_zoom,'min');
        set(interface.figures.main.axes.dynamic,...% dynamic axes limits
            'xlim', [-1 1]*xylim,...
            'ylim', [-1 1]*xylim,...
            'view', [0,90],...
            'projection', 'orthographic');
        set(interface.figures.main.sliders.ax_dynamic_rot,'value',0)
        set(interface.figures.main.sliders.ax_dynamic_tilt,'value',90)
        axes_dynamic_perspective_Callback(...
            interface.figures.main.sliders.ax_dynamic_ypan)

        % initialze common variables
        r_x = road.x;           % road x array
        r_y = road.y(r_x);      % road y array
        l_x = lane.x(r_x);      % lane x array
        l_y = lane.y(r_x);      % lane y array
        re_x = road_edge.x(r_x);      % road edge x array
        re_y = road_edge.y(r_x);      % road edge y array
        
        % extrapolate re_x and re_x to full x range
        [re_l,re_r,re_c] = split_road_edges(re_x,re_y);
        xi = r_x(1:10:end)';        % lower resolution array for plotting
        re_l = [xi interp1(re_l(:,1),re_l(:,2),xi,'pchirp','extrap')];
        re_r = [xi interp1(re_r(:,1),re_r(:,2),xi,'pchirp','extrap')];
        re_c = [xi interp1(re_c(:,1),re_c(:,2),xi,'pchirp','extrap')];
        %reconstruct road edges
        re_x = [re_l(:,1);nan;re_c(:,1);nan;re_r(:,1)]';
        re_y = [re_l(:,2);nan;re_c(:,2);nan;re_r(:,2)]';
        
        e_x = ego.x(0,ego.x_1); % ego x position
        e_y = ego.y(e_x);       % ego y position
        
        s_x = stand.x;          % standing object x array
        s_y = stand.y;          % standing object y array
        
        % find objects standing in the road
        [ra_l,ra_r] = find_road_points(re_x,re_y,ego,e_y,s_x);
        
        % remove objects standing in the road
        stand.x(logical((ra_r < s_y).*(s_y < ra_l))) = [];
        stand.y(logical((ra_r < s_y).*(s_y < ra_l))) = [];
        stand.z(logical((ra_r < s_y).*(s_y < ra_l))) = [];
        stand.n = length(stand.y);
        
        % initialize road area
        xl = [0 r_x(end)];
        yl = [-1 1]*road.T/2;
        road_area = find_road_area(...
            interface,road,road_area,re_x,re_y,ego,e_y,xl,yl);

        % static axes plots:
        road_area.m.static = patch(road_area.X,road_area.Y,[50 150 0]/270,...
            'facealpha',0.7,...
            'parent',interface.figures.main.axes.static);
        colormap([bsxfun(@times,[50 150 0]/270,ones(100,3));[1 1 1]*0.3])
        
        road.m.static = plot(interface.figures.main.axes.static,...
            r_x, r_y,'w','linewidth',2,'visible','off');
        lane.m.static = plot(interface.figures.main.axes.static,...
            l_x, l_y,'--w','linewidth',2);
        road_egde.m.static = plot(interface.figures.main.axes.static,...
            re_x, re_y,'w','linewidth',2);
        ego.m.static =  plot(interface.figures.main.axes.static,...
            0,0,'or','linewidth',2);
        stand.m.static = plot(interface.figures.main.axes.static,...
            stand.x, stand.y,'og','linewidth',2);
        mov.m.static =  plot(interface.figures.main.axes.static,...
            0,0,'o','color',[1 0.5 0],'linewidth',2);
        onc.m.static =  plot(interface.figures.main.axes.static,...
            0,0,'oc','linewidth',2);        
       
        % dynamic axes plots
        %   road area
        road_area.m.dynamic_surf = surf(...
            [0 0],[0 0],zeros(2),...
            'edgecolor','none',...
            'facealpha',0.7,...
            'parent',interface.figures.main.axes.dynamic);
        road_area.m.dynamic_surf_edge = surf(...
            [0 0],[0 0],zeros(2),...
            'facecolor','none',...
            'edgecolor',[0.7 1 0]*0.9,... 
            'parent',interface.figures.main.axes.dynamic);
        road_area.m.dynamic_patch = patch(...
            0,0,[50 150 0]/270,...
            'edgecolor','none',...
            'facealpha',0.7,...
            'parent',interface.figures.main.axes.dynamic);

        % sensor fov
        for ii = 1 : ego.sensor.n
            % dynamic axes
            ego.sensor.m(ii) = patch(... % draw sensor FoV
                ego.sensor.fov.draw.circ{ii}(:,2),...
                ego.sensor.fov.draw.circ{ii}(:,1),...
                'w','FaceAlpha',.15,...
                'visible','off',...
                'parent', interface.figures.main.axes.dynamic);
            % sensor specific FoV
            ego.sensor.s(ii) = patch(... % draw sensor FoV
                ego.sensor.fov.draw.fov(:,2),...
                ego.sensor.fov.draw.fov(:,1),...
                'w','FaceAlpha',.3,...
                'parent', interface.figures.main.axes.sensor(ii));
            ego.sensor.d{ii}.road = plot(...
                interface.figures.main.axes.sensor(ii),...
                0,0,'w','linewidth',2);
            ego.sensor.d{ii}.stand = plot(...
                interface.figures.main.axes.sensor(ii),...
                0,0,'og','linewidth',2);
            ego.sensor.d{ii}.mov = plot(...
                interface.figures.main.axes.sensor(ii),...
                0,0,'o','color',[1 0.5 0],'linewidth',2);
            ego.sensor.d{ii}.onc = plot(...
                interface.figures.main.axes.sensor(ii),...
                0,0,'oc','linewidth',2);
        end
              
        %   simulated objects
        road.m.dynamic = plot(interface.figures.main.axes.dynamic,...
            0,0,'w','linewidth',2,'visible','off');
        lane.m.dynamic = plot(interface.figures.main.axes.dynamic,...
            0,0,'--w','linewidth',2);
        road_edge.m.dynamic = plot(interface.figures.main.axes.dynamic,...
            0,0,'w','linewidth',2);
        ego.m.dynamic = patch(...              % draw ego cube
            ego.cube.x(ego.cube.idx),...
            ego.cube.y(ego.cube.idx),...
            ego.cube.z(ego.cube.idx),...
            'r','facealpha',0.5,...
            'parent',interface.figures.main.axes.dynamic);
        for ii = 1 : stand.n
            stand.m.dynamic(ii) = patch(...    % draw standing object cubes
                0,0,0,...
                'g','facealpha',0.5,...
                'parent',interface.figures.main.axes.dynamic);
        end
        for ii = 1 : mov.n
            mov.m.dynamic(ii) = patch(...      % draw moving object cubes
                0,0,0,...
                'facecolor',[1 0.5 0],...
                'facealpha',0.5,...
                'parent',interface.figures.main.axes.dynamic);
        end
        for ii = 1 : onc.n
            onc.m.dynamic(ii) = patch(...       % draw moving object cubes
                0,0,0,...
                'c','facealpha',0.5,...
                'parent',interface.figures.main.axes.dynamic);
        end
        %   user-detected objects
        ego.m.dynamic_user = plot3(interface.figures.main.axes.dynamic,...
            0,0,0,'sr','visible','off','markersize',15,'linewidth',2);
        stand.m.dynamic_user = plot3(interface.figures.main.axes.dynamic,...
            0,0,0,'sg','visible','off','markersize',15,'linewidth',2);
        mov.m.dynamic_user = plot3(interface.figures.main.axes.dynamic,...
            0,0,0,'s','color',[1 0.5 0],'visible','off','markersize',15,...
            'linewidth',2);
        onc.m.dynamic_user = plot3(interface.figures.main.axes.dynamic,...
            0,0,0,'sc','visible','off','markersize',15,'linewidth',2);
        
        interface.figures.main.ax_init = true;
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
            interface,...
            road,road_x,road_y,...
            lane,lane_x,lane_y,...
            road_edge,road_edge_x,road_edge_y,...
            road_area,road_area_x,road_area_y,road_area_z,...
            ego,...
            stand,stand_x,stand_y,...
            mov,mov_x,mov_y,...
            onc,onc_x,onc_y,fov)
        
        % update dynamic plot data
        set(road.m.dynamic, 'xData',road_y, 'yData',road_x)
        set(lane.m.dynamic, 'xData',lane_y, 'yData',lane_x)
        set(road_edge.m.dynamic, 'xData',road_edge_y, 'yData',road_edge_x)
        
        % update rooad area depending on user-defined tilt angle
        switch road_area.type
            case 'patch'
                terrain_3D = false;
                set(road_area.m.dynamic_patch,'visible','on',...
                    'xData',road_area_y,'yData',road_area_x,...
                    'zData',-0.2*ones(size(road_area_y)))
                set(road_area.m.dynamic_surf_edge,'visible','off')
                set(road_area.m.dynamic_surf,'visible','off')
            case 'surf'
                terrain_3D = true;
                set(road_area.m.dynamic_surf,...
                    'visible','on',...
                    'xData',road_area_y, 'yData',road_area_x,...
                    'cdata',road_area.map,'zdata',road_area_z)
                if get(interface.figures.main.sliders.ax_dynamic_tilt,'value') < 70
                    d = 1:3:size(road_area_y,1);
                    edge = road_area_z(d,d);
                    edge(road_area.map(d,d) == 0) = nan;
                    set(road_area.m.dynamic_surf_edge,...
                        'visible','on',...
                        'xData',road_area_y(d,d),'yData',road_area_x(d,d),...
                        'cdata',road_area.map(d,d),'zdata',edge)
                else
                    set(road_area.m.dynamic_surf_edge,'visible','off')
                end
                set(road_area.m.dynamic_patch,'visible','off')
        end

        for ii = 1 : stand.n
            if (fov(1,1) <= stand_x(ii)) && (stand_x(ii) <= fov(2,1)) && ...
                    (fov(1,2) <= stand_y(ii)) && (stand_y(ii) <= fov(2,2))
                set(stand.m.dynamic(ii),... % draw standing object cubes
                    'visible','on',...
                    'xData',stand.cube(ii).y(stand.cube(ii).idx),...
                    'yData',stand.cube(ii).x(stand.cube(ii).idx),...
                    'zData',stand.cube(ii).z(stand.cube(ii).idx)+...
                    stand.z(ii)*terrain_3D);
            else
                set(stand.m.dynamic(ii),'visible','off');
            end
        end
        for ii = 1 : mov.n
            if (fov(1,1) <= mov_x(ii)) && (mov_x(ii) <= fov(2,1)) && ...
                    (fov(1,2) <= mov_y(ii)) && (mov_y(ii) <= fov(2,2))
                set(mov.m.dynamic(ii),... % draw moving object cubes
                    'visible','on',...
                    'xData',mov.cube(ii).y(mov.cube(ii).idx),...
                    'yData',mov.cube(ii).x(mov.cube(ii).idx),...
                    'zData',mov.cube(ii).z(mov.cube(ii).idx));
            else
                set(mov.m.dynamic(ii),'visible','off');
            end
        end
        for ii = 1 : onc.n
            if (fov(1,1) <= onc_x(ii)) && (onc_x(ii) <= fov(2,1)) && ...
                    (fov(1,2) <= onc_y(ii)) && (onc_y(ii) <= fov(2,2))
                set(onc.m.dynamic(ii),... % draw oncoming object cubes
                    'visible','on',...
                    'xData',onc.cube(ii).y(onc.cube(ii).idx),...
                    'yData',onc.cube(ii).x(onc.cube(ii).idx),...
                    'zData',onc.cube(ii).z(onc.cube(ii).idx));
            else
                set(onc.m.dynamic(ii),'visible','off');
            end
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
            f1 = evalin('base','sim_world_data.interface.figures.visu.f');
            f2 = evalin('base','sim_world_data.interface.figures.main.f');
            close(f1)
            close(f2)
            delete(f1)
            delete(f2)
        catch
            % nothing
        end
        
        interface = [];
        interface.colors.background       = [1 1 1]*0.25;
        interface.colors.plot_background  = [1 1 1]*0.4;
        interface.colors.plot_lines       = [1 1 1]*1;
        interface.colors.panel_background = [1 1 1]*0.3;
        interface.colors.font             = [1 1 1]*1;
        interface.colors.code_background  = [1 1 1]*0.2;
        interface.colors.code_font        = [0.9 1 0.8];
        interface.figures.visu.f = figure(...
            'color',interface.colors.background,...
            'NumberTitle', 'off',...
            'name','ADAS SimWorld: visualizatrion',...
            'position',[1 31 1920 1020],...
            'MenuBar', 'none',...
            'visible','off',...
            'CloseRequestFcn',@f_CloseRequestFcn);
        colormap([bsxfun(@times,[50 150 0]/270,ones(100,3));[1 1 1]*0.3])
        interface.figures.main.f = figure(...
            'NumberTitle', 'off',...
            'name','ADAS SimWorld',...
            'color',interface.colors.background,...
            'MenuBar', 'none',...
            'position',[1 31 1920 1020],... %[450 80 1080 720]
            'CloseRequestFcn',@f_CloseRequestFcn);
        
        % ************************ playback panel ************************
        
        interface.figures.main.panels.playback_main = uipanel(...
            'parent',interface.figures.main.f,...
            'position',[0.15,0.03,0.84,0.94],...
            'title','Main playback');
        init_ui_style(interface.figures.main.panels.playback_main,interface)
        
        % static axes
        interface.figures.main.axes.static = axes(...
            'parent',interface.figures.main.panels.playback_main,...
            'position',[0.54 0.73 0.54 0.21]);
        init_axes_style(interface.figures.main.axes.static,interface)
        
        % dynamic axes
        interface.figures.main.axes.dynamic_pos = [0.645 0.08 0.32 0.58];
        interface.figures.main.axes.dynamic = axes(...
            'parent',interface.figures.main.panels.playback_main,...
            'position',interface.figures.main.axes.dynamic_pos,...
            'xdir','reverse');
        init_axes_style(interface.figures.main.axes.dynamic,interface)
        
        % zoom slider
        interface.figures.main.sliders.ax_dynamic_zoom = uicontrol(...
            'parent',interface.figures.main.panels.playback_main,...
            'units','normalized',...
            'style','slider',...
            'min', 10,...
            'max', 100,...
            'value',90,...
            'position', [0.645 0.67 0.33 0.02],...
            'callback',@axes_dynamic_perspective_Callback);
        init_ui_style(...
            interface.figures.main.sliders.ax_dynamic_zoom,interface)
        
        % rotation slider
        interface.figures.main.sliders.ax_dynamic_rot = uicontrol(...
            'parent',interface.figures.main.panels.playback_main,...
            'units','normalized',...
            'style','slider',...
            'min', -180,...
            'max', 180,...
            'value',0,...
            'position', [0.645 0.05 0.33 0.02],...
            'callback',@axes_dynamic_perspective_Callback);
        init_ui_style(...
            interface.figures.main.sliders.ax_dynamic_rot,interface)
        
        % tilt slider
        interface.figures.main.sliders.ax_dynamic_tilt = uicontrol(...
            'parent',interface.figures.main.panels.playback_main,...
            'units','normalized',...
            'style','slider',...
            'min', 0,...
            'max', 90,...
            'value',90,...
            'position', [0.98 0.08 0.012 0.58],...
            'callback',@axes_dynamic_perspective_Callback);
        init_ui_style(...
            interface.figures.main.sliders.ax_dynamic_tilt,interface)
        
        % y pan slider
        interface.figures.main.sliders.ax_dynamic_ypan = uicontrol(...
            'parent',interface.figures.main.panels.playback_main,...
            'units','normalized',...
            'style','slider',...
            'min', -0.5,...
            'max', 0.5,...
            'value',0,...
            'position', [0.62 0.08 0.012 0.58],...
            'callback',@axes_dynamic_perspective_Callback);
        init_ui_style(...
            interface.figures.main.sliders.ax_dynamic_ypan,interface)
        
        % pop button
        interface.figures.main.buttons.ax_dynamic_pop = uicontrol(...
            'parent',interface.figures.main.panels.playback_main,...
            'units','normalized',...
            'style','pushbutton',...
            'string','Pop',...
            'position', [0.92 0.015 0.04 0.025],...
            'callback',@ax_dynamic_pop_Callback);
        init_ui_style(...
            interface.figures.main.buttons.ax_dynamic_pop,interface)
        
        % sensor axes
        w = 0.29;       % axes width
        h = 0.21;        % axes height
        off_x = 0.02;   % x offset
        off_y = 0.05;   % y offset
        xy_off = [0       0 w+off_x w+off_x;...
                  h+off_y 0 0       h+off_y]';
        ttl = {'FL','RL','RR','FR'};
        for ii = 1 : 4
            interface.figures.main.axes.sensor(ii) = axes(...
                'parent',interface.figures.main.panels.playback_main,...
                'position',[0.02+xy_off(ii,1) 0.05+xy_off(ii,2) w h],...
                'xdir','reverse');
            init_axes_style(interface.figures.main.axes.sensor(ii),interface)
            title(ttl{ii},'color','w')
        end
        
        % user coding text box
        interface.figures.main.edits.user_code = uicontrol(...
            'parent',interface.figures.main.panels.playback_main,...
            'units','normalized',...
            'style','edit',...
            'string','Error: Failed to load file',...
            'position', [0.01, 0.55, 0.6, 0.40],...
            'max',1000,...
            'HorizontalAlignment','left',...
            'backgroundcolor',interface.colors.code_background,...
            'foregroundcolor',interface.colors.code_font,...
            'fontsize',10,...
            'fontname','monospaced');
        
        % choose function dropdown menu
        interface.figures.main.popups.user_code_selectFunction = uicontrol(...
            'parent',interface.figures.main.panels.playback_main,...
            'units','normalized',...
            'style','popup',...
            'string',{'Select function','Reconstruct 360'},...
            'value',2,...
            'position', [0.01, 0.96, 0.09, 0.03],...
            'callback',@user_code_selectFunction_Callback);
        init_ui_style(...
            interface.figures.main.popups.user_code_selectFunction,interface)
        
        % filename config
        interface.files.reconstruct_360_space.path = 'usr/reconstruct_360_space.m';
        interface.files.reconstruct_360_space.func = 'reconstruct_360_space';
        
        % load file to user code textbox
        sim_world_data.interface = interface;
        assignin('base','sim_world_data',sim_world_data) % store in base ws
        user_code_selectFunction_Callback([]);
        interface = evalin('base','sim_world_data.interface'); % recover from base ws
        
        % load file button
        interface.figures.main.buttons.user_code_load = uicontrol(...
            'parent',interface.figures.main.panels.playback_main,...
            'units','normalized',...
            'style','pushbutton',...
            'string','Load',...
            'position', [0.11, 0.96, 0.05, 0.03],...
            'callback',@user_code_load_Callback);
        init_ui_style(...
            interface.figures.main.buttons.user_code_load,interface)
        
        % save file button
        interface.figures.main.buttons.user_code_save = uicontrol(...
            'parent',interface.figures.main.panels.playback_main,...
            'units','normalized',...
            'style','pushbutton',...
            'string','Save',...
            'enable','off',...
            'position', [0.17, 0.96, 0.05, 0.03],...
            'callback',@user_code_save_Callback);
        init_ui_style(...
            interface.figures.main.buttons.user_code_save,interface)
        
        % save as file button
        interface.figures.main.buttons.user_code_saveas = uicontrol(...
            'parent',interface.figures.main.panels.playback_main,...
            'units','normalized',...
            'style','pushbutton',...
            'string','Save as',...
            'position', [0.23, 0.96, 0.05, 0.03],...
            'callback',@user_code_saveas_Callback);
        init_ui_style(...
            interface.figures.main.buttons.user_code_saveas,interface)
        
        % autosave checkbox
        interface.figures.main.checkboxes.user_code_autosave = uicontrol(...
            'parent',interface.figures.main.panels.playback_main,...
            'units','normalized',...
            'style','checkbox',...
            'string','Autosave',...
            'enable','off',...
            'position', [0.29, 0.96, 0.05, 0.03]);
        init_ui_style(...
            interface.figures.main.checkboxes.user_code_autosave ,interface)
        
        % ************************ control panel ************************
        
        interface.figures.main.panels.controls_main = uipanel(...
            'parent',interface.figures.main.f,...
            'position',[0.01,0.03,0.13,0.94],...
            'title','Main controls');
        init_ui_style(interface.figures.main.panels.controls_main,interface)
        
        % Start / pause / resume button
        interface.figures.main.buttons.controls_main_play = uicontrol(...
            'parent',interface.figures.main.panels.controls_main,...
            'units','normalized',...
            'style','toggle',...
            'string','Start',...
            'position', [0.05, 0.9, 0.25, 0.035],...
            'callback',@controls_main_play_Callback);
        init_ui_style(...
            interface.figures.main.buttons.controls_main_play,interface)
        
        % Step button
        interface.figures.main.buttons.controls_main_step = uicontrol(...
            'parent',interface.figures.main.panels.controls_main,...
            'units','normalized',...
            'style','toggle',...
            'string','Step',...
            'position', [0.35, 0.9, 0.25, 0.035],...
            'callback',@controls_main_step_Callback);
        init_ui_style(...
            interface.figures.main.buttons.controls_main_step,interface)
        
        % Reset button
        interface.figures.main.buttons.controls_main_reset = uicontrol(...
            'parent',interface.figures.main.panels.controls_main,...
            'units','normalized',...
            'style','pushbutton',...
            'string','Reset',...
            'position', [0.70, 0.9, 0.25, 0.035],...
            'callback',@controls_main_reset_Callback);
        init_ui_style(...
            interface.figures.main.buttons.controls_main_reset,interface)
        
        % ******************** sensor selection panel ********************
        interface.figures.main.panels.controls_main_showSensor = uipanel(...
            'parent',interface.figures.main.panels.controls_main,...
            'position',[0.03,0.77,0.94,0.1],...
            'title','Show reconstructued sensors');
        init_ui_style(...
            interface.figures.main.panels.controls_main_showSensor,...
            interface)
        
        % choose sensor menu
        interface.figures.main.popups.controls_main_showSensor = uicontrol(...
            'parent',interface.figures.main.panels.controls_main_showSensor,...
            'units','normalized',...
            'style','popup',...
            'string',{'Hide all','Select','Show all'},...
            'value',1,...
            'position', [0.03, 0.9, 0.96, 0.03],...
            'callback',@controls_main_showSensor_Callback);
        init_ui_style(...
            interface.figures.main.popups.controls_main_showSensor,interface)
        
        % choose sensor checkboxes
        interface.figures.main.checkboxes.controls_main_showSensor_FL = uicontrol(...
            'parent',interface.figures.main.panels.controls_main_showSensor,...
            'units','normalized',...
            'style','checkbox',...
            'string',{'Front Left (FL)'},...
            'value',0,...
            'enable','off',...
            'position', [0.03, 0.35, 0.46, 0.15],...
            'callback',@controls_main_showSensor_cb_Callback);
        init_ui_style(...
            interface.figures.main.checkboxes.controls_main_showSensor_FL,...
            interface)
        interface.figures.main.checkboxes.controls_main_showSensor_FR = uicontrol(...
            'parent',interface.figures.main.panels.controls_main_showSensor,...
            'units','normalized',...
            'style','checkbox',...
            'string',{'Front Right (FR)'},...
            'value',0,...
            'enable','off',...
            'position', [0.53, 0.35, 0.46, 0.15],...
            'callback',@controls_main_showSensor_cb_Callback);
        init_ui_style(...
            interface.figures.main.checkboxes.controls_main_showSensor_FR,...
            interface)
        interface.figures.main.checkboxes.controls_main_showSensor_RL = uicontrol(...
            'parent',interface.figures.main.panels.controls_main_showSensor,...
            'units','normalized',...
            'style','checkbox',...
            'string',{'Rear Left (RL)'},...
            'value',0,...
            'enable','off',...
            'position', [0.03, 0.10, 0.46, 0.15],...
            'callback',@controls_main_showSensor_cb_Callback);
        init_ui_style(...
            interface.figures.main.checkboxes.controls_main_showSensor_RL,...
            interface)
        interface.figures.main.checkboxes.controls_main_showSensor_RR = uicontrol(...
            'parent',interface.figures.main.panels.controls_main_showSensor,...
            'units','normalized',...
            'style','checkbox',...
            'string',{'Rear Right (RR)'},...
            'value',0,...
            'enable','off',...
            'position', [0.53, 0.10, 0.46, 0.15],...
            'callback',@controls_main_showSensor_cb_Callback);
        init_ui_style(...
            interface.figures.main.checkboxes.controls_main_showSensor_RR,...
            interface)
        
        %********************* simulation parameters *********************
        
        interface.figures.main.panels.controls_main_parameters = uipanel(...
            'parent',interface.figures.main.panels.controls_main,...
            'position',[0.03,0.5,0.94,0.2],...
            'title','Simulation rendering (reset to apply)');
        init_ui_style(...
            interface.figures.main.panels.controls_main_parameters,...
            interface)
        
        % simulation time resolution
        controls_main_parameters_dt_yanchor = 0.85;
        
        % simulation time resolution text
        interface.figures.main.texts.controls_main_parameters_dt =...
            uicontrol(...
            'parent',interface.figures.main.panels.controls_main_parameters,...
            'units','normalized',...
            'style','text',...
            'string', 'Time resolution (s)', ...
            'position', [0.03 controls_main_parameters_dt_yanchor 0.40 0.1]);
        init_ui_style(...
            interface.figures.main.texts.controls_main_parameters_dt,...
            interface)
        
        % simulation time resolution slider
        interface.figures.main.sliders.controls_main_parameters_dt =...
            uicontrol(...
            'parent',interface.figures.main.panels.controls_main_parameters,...
            'units','normalized',...
            'style','slider',...
            'min', 0.005,...
            'max', 0.1,...
            'value',0.02,...
            'SliderStep', [0.01, 0.02]/(0.1-0.005),...
            'position', [0.03 controls_main_parameters_dt_yanchor-0.1 0.75 0.1],...
            'callback',@controls_main_parameters_dt_Callback);
        init_ui_style(...
            interface.figures.main.sliders.controls_main_parameters_dt,...
            interface)
        
        % simulation time resolution edit
        interface.figures.main.edits.controls_main_parameters_dt =...
            uicontrol(...
            'parent',interface.figures.main.panels.controls_main_parameters,...
            'units','normalized',...
            'style','edit',...
            'string', '0.02', ...
            'position', [0.81 controls_main_parameters_dt_yanchor-0.1 0.16 0.1],...
            'callback',@controls_main_parameters_dt_Callback);
        init_ui_style(...
            interface.figures.main.edits.controls_main_parameters_dt,...
            interface)
        
        % number of standing objects
        controls_main_parameters_stand_n_yanchor = 0.6 ;
        
        % number of standing objects text
        interface.figures.main.texts.controls_main_parameters_stand_n =...
            uicontrol(...
            'parent',interface.figures.main.panels.controls_main_parameters,...
            'units','normalized',...
            'style','text',...
            'string', 'Number of standing objects', ...
            'position', [0.03 controls_main_parameters_stand_n_yanchor 0.60 0.1]);
        init_ui_style(...
            interface.figures.main.texts.controls_main_parameters_stand_n,...
            interface)
        
        % number of standing objects slider
        interface.figures.main.sliders.controls_main_parameters_stand_n =...
            uicontrol(...
            'parent',interface.figures.main.panels.controls_main_parameters,...
            'units','normalized',...
            'style','slider',...
            'min', 0,...
            'max', 80,...
            'value',40,...
            'SliderStep', [1/80, 0.1],...
            'position', [0.03 controls_main_parameters_stand_n_yanchor-0.1 0.75 0.1],...
            'callback',@controls_main_parameters_stand_n_Callback);
        init_ui_style(...
            interface.figures.main.sliders.controls_main_parameters_stand_n,...
            interface)
        
        % number of standing objects edit
        interface.figures.main.edits.controls_main_parameters_stand_n =...
            uicontrol(...
            'parent',interface.figures.main.panels.controls_main_parameters,...
            'units','normalized',...
            'style','edit',...
            'string', '40', ...
            'position', [0.81 controls_main_parameters_stand_n_yanchor-0.1 0.16 0.1],...
            'callback',@controls_main_parameters_stand_n_Callback);
        init_ui_style(...
            interface.figures.main.edits.controls_main_parameters_stand_n,...
            interface)
        
        % 3D terrain depth
        controls_main_parameters_terrain_h_yanchor = 0.35 ;
        
        % 3D terrain depth text
        interface.figures.main.texts.controls_main_parameters_terrain_h =...
            uicontrol(...
            'parent',interface.figures.main.panels.controls_main_parameters,...
            'units','normalized',...
            'style','text',...
            'string', '3D terrain depth', ...
            'position', [0.03 controls_main_parameters_terrain_h_yanchor 0.35 0.1]);
        init_ui_style(...
            interface.figures.main.texts.controls_main_parameters_terrain_h,...
            interface)
        
        % 3D terrain depth slider
        interface.figures.main.sliders.controls_main_parameters_terrain_h =...
            uicontrol(...
            'parent',interface.figures.main.panels.controls_main_parameters,...
            'units','normalized',...
            'style','slider',...
            'min', 0,...
            'max', 40,...
            'value',20,...
            'SliderStep', [1, 5]/40,...
            'position', [0.03 controls_main_parameters_terrain_h_yanchor-0.1 0.75 0.1],...
            'callback',@controls_main_parameters_terrain_h_Callback);
        init_ui_style(...
            interface.figures.main.sliders.controls_main_parameters_terrain_h,...
            interface)
        
        % 3D terrain depth edit
        interface.figures.main.edits.controls_main_parameters_terrain_h =...
            uicontrol(...
            'parent',interface.figures.main.panels.controls_main_parameters,...
            'units','normalized',...
            'style','edit',...
            'string', '20', ...
            'position', [0.81 controls_main_parameters_terrain_h_yanchor-0.1 0.16 0.1],...
            'callback',@controls_main_parameters_terrain_h_Callback);
        init_ui_style(...
            interface.figures.main.edits.controls_main_parameters_terrain_h,...
            interface)
        
        % assign interface to base workspace
        sim_world_data.interface = interface;
        assignin('base','sim_world_data',sim_world_data)
        
    end

    function init_axes_style(ax, interface)
       set(ax,...
           'color',interface.colors.plot_background,...
           'xcolor',interface.colors.plot_lines,...
           'ycolor',interface.colors.plot_lines,...
           'zcolor',interface.colors.plot_lines)
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
    
    function [ra_l,ra_r,ra_c] = split_road_edges(road_edge_x,road_edge_y)
        
        % find left edge
        splits = find(isnan(road_edge_x)) + [-1 1];
        ra_l = [road_edge_x(1:splits(1))' road_edge_y(1:splits(1))'];
        
        % find right edge
        ra_r = [road_edge_x(splits(2):end)' road_edge_y(splits(2):end)'];
        
        % find road center
        ra_c = [road_edge_x(splits(1)+2:splits(2)-2)'...
                road_edge_y(splits(1)+2:splits(2)-2)'];
    end

    function [ra_l,ra_r] = find_road_points(...
            road_edge_x,road_edge_y,ego,ego_y,p_x)
        
        % split road esges
        [ra_l,ra_r] = split_road_edges(road_edge_x,road_edge_y);
        
        % interpolate y coordinates
        ra_l = interp1(ra_l(:,1),ra_l(:,2),p_x+ego.x_1,'pchirp','extrap')-ego_y;
        ra_r = interp1(ra_r(:,1),ra_r(:,2),p_x+ego.x_1,'pchirp','extrap')-ego_y;
    end

    function road_area = find_road_area(...
            interface,road,road_area,road_edge_x,road_edge_y,ego,ego_y,xl,yl)
        
        if nargin() < 8
            % dynamic x, y limits
            yl = get(interface.figures.main.axes.dynamic,'ylim');
            yl = mean(yl) + [-1 1]*diff(yl)*1.1; % expand to fit y span   
            % limit x interpolation range
            xl = min(yl,road.x(end)/2);
        end
               
        % choose road area type
        if get(interface.figures.main.sliders.ax_dynamic_tilt,'value') > 70
            type = 'patch';
            x = xl(1):diff(road.x(1:2))*15:xl(2);  % initialize x array
            % find road area coverage
            [ra_l,ra_r] = find_road_points(... 
                road_edge_x,road_edge_y,ego,ego_y,x);
                
            [~,road_area.X, road_area.Y] = init_road_area(...
                xl,yl,road,type,x,ra_l,ra_r);
        else
            type = 'surf';
            % initialize road area
            [~,road_area.X, road_area.Y] = init_road_area(xl,yl,road,type);
            
            % find road area coverage
            [ra_l,ra_r] = find_road_points(...
                    road_edge_x,road_edge_y,ego,ego_y,...
                        road_area.X(1,:));
            
            road_area.map = -0.2*ones(size(road_area.X));
            % subtract road from map for transparency
            road_area.map(logical(...
                (bsxfun(@times,ra_r,ones(size(road_area.X)))<=road_area.Y).*...
                (road_area.Y<bsxfun(@times,ra_l,ones(size(road_area.X))))...
                )) = 0;
        end
        
        % set road area type
        road_area.type = type;
        
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
        set(interface.figures.main.edits.user_code,...
            'string', fTxt)
    end

    function save_user_code(interface, fname)
        fID = fopen(fname,'w');         % file ID
        eTxt = get(interface.figures.main.edits.user_code,'string');
        
        for ii = 1 : length(eTxt)
           fprintf(fID,'%s\n', eTxt{ii});
        end

        fclose(fID);
    end

    function reset_plots(interface)
        cla(interface.figures.main.axes.static);
        init_axes_style(interface.figures.main.axes.static, interface)
        cla(interface.figures.main.axes.dynamic);
        init_axes_style(interface.figures.main.axes.dynamic, interface)
        for ii = 1 : length(interface.figures.main.axes.sensor)
            cla(interface.figures.main.axes.sensor(ii));
            init_axes_style(interface.figures.main.axes.sensor(ii), interface)
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

    function obj = update_stand_object_cube(obj,obj_x,obj_y,theta,fov)
        
        for ii = 1 : obj.n % object cubes (visualization)
            if (fov(1,1) <= obj_x(ii)) && (obj_x(ii) <= fov(2,1)) && ...
                    (fov(1,2) <= obj_y(ii)) && (obj_y(ii) <= fov(2,2))
                obj.cube(ii).center = [obj_x(ii),obj_y(ii),0];
                obj.cube(ii).theta  = theta/pi*180 + 90;
                obj.cube(ii)        = init_cube(obj.cube(ii));
            end
        end
    end

    function obj = update_mov_object_cube(obj,obj_x,obj_y,theta,fov)
        
        for ii = 1 : obj.n % object cubes (visualization)
            if (fov(1,1) <= obj_x(ii)) && (obj_x(ii) <= fov(2,1)) && ...
                    (fov(1,2) <= obj_y(ii)) && (obj_y(ii) <= fov(2,2))
                obj.cube(ii).center = [obj_x(ii),obj_y(ii),0];
                obj.cube(ii).theta  = theta/pi*180 - obj.theta(ii)+90;
                obj.cube(ii)        = init_cube(obj.cube(ii));
            end
        end
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

% ************************** interface callbacks **************************

    function controls_main_play_Callback(source,~)
        interface = evalin('base','sim_world_data.interface');
        switch get(source,'value')
            case 1
                set(interface.figures.main.buttons.controls_main_play,...
                    'string','Pause');
                set(interface.figures.main.buttons.controls_main_step,...
                    'enable','off')
                set(interface.figures.main.buttons.controls_main_reset,...
                    'enable','off')
                if get(interface.figures.main.checkboxes.user_code_autosave,...
                        'value')
                    wb = waitbar(0,'Saving file');
                    user_code_save_Callback();
                    for ii =1:50
                        waitbar(ii/50) % wait a moment for file to save
                    end
                    clear reconstruct_360_space
                    close(wb)
                end
            case 0
                set(interface.figures.main.buttons.controls_main_play,...
                    'string','Resume');
                set(interface.figures.main.buttons.controls_main_step,...
                    'enable','on')
                set(interface.figures.main.buttons.controls_main_reset,...
                    'enable','on')
        end
        [interface,road,lane,road_edge,road_area,ego,stand,mov,onc,t,dt,road_tail] =...
            read_sim_data();
        master_run(interface,source,road,lane,road_edge,road_area,...
            ego,stand,mov,onc,road_tail,dt,t);
    end

    function controls_main_step_Callback(source,~)
        interface = evalin('base','sim_world_data.interface');
        set(interface.figures.main.buttons.controls_main_step,'value',0)
        controls_main_play_Callback(...
            source)
    end

    function axes_dynamic_perspective_Callback(source,~)
        
        % get interface from base workspace
        interface = evalin('base','sim_world_data.interface');
        vv = get(interface.figures.main.axes.dynamic,'view');  % axes view
        value = get(source,'value');    % slider value
        vmin = get(source,'min');       % slider min value
        vmax = get(source,'max');       % slider max value
        vinv = (vmax-value+vmin);       % inverted value
        switch source
            case interface.figures.main.sliders.ax_dynamic_zoom
                yoff = 2* get(... % y offset introduced by y pan slider
                    interface.figures.main.sliders.ax_dynamic_ypan,'value');
                set(interface.figures.main.axes.dynamic,...
                    'xlim',[-1 1]*vinv,...
                    'ylim',([-1 1] - yoff)*vinv)
                controls_main_play_Callback(...
                    interface.figures.main.buttons.controls_main_play)
            case interface.figures.main.sliders.ax_dynamic_rot
                set(interface.figures.main.axes.dynamic,...
                    'view',[vinv vv(2)],...
                    'projection','perspective')
            case interface.figures.main.sliders.ax_dynamic_tilt
                set(interface.figures.main.axes.dynamic,...
                    'view',[vv(1) value],...
                    'projection','perspective')
            case interface.figures.main.sliders.ax_dynamic_ypan
                yl = get(interface.figures.main.axes.dynamic,'ylim'); % ylim
                set(interface.figures.main.axes.dynamic,'ylim',...
                    [-1 1]*diff(yl)/2 + diff(yl)*vinv)
        end
    end

    function ax_dynamic_pop_Callback(source,~)
        
        % get interface from main workspace
        interface = evalin('base','sim_world_data.interface');
        
        % get dynamic axes
        ax_dynamic = interface.figures.main.axes.dynamic;
        fig_visu = interface.figures.visu.f;
        pan_playback = interface.figures.main.panels.playback_main;
        
        % change parent
        switch get(ax_dynamic,'parent')
            case pan_playback
                set(ax_dynamic,...
                    'parent',fig_visu,...
                    'position',[0.1 0.1 0.8 0.8])
                set(fig_visu,'visible','on')
                set(source,'string','Pull')
            case fig_visu
                set(ax_dynamic,...
                    'parent',pan_playback,...
                    'position',interface.figures.main.axes.dynamic_pos)
                set(fig_visu,'visible','off')
                set(source,'string','Pop')
        end
    end       

    function func = ui_get_user_code_Function(source)
        
        % load interface from base workspace
        interface = evalin('base','sim_world_data.interface');
        
        % get selected function
        if isempty(source),
            source = interface.figures.main.popups.user_code_selectFunction;
        end
        funcs = get(source,'string');
        val = get(source, 'value');
        
        func = funcs{val};
    end

    function controls_main_reset_Callback(~,~)
        % get interface from base workspace
        interface = evalin('base','sim_world_data.interface');
        
        % stop running simulation
        set(interface.figures.main.buttons.controls_main_play,...
            'value',0,...
            'string','Start')
        
        % initialize simulation time
        dt = get(... % (s) time resolution
            interface.figures.main.sliders.controls_main_parameters_dt,...
            'value');  
        t = 0;       % (s) time
    
        % initialize road map
        road = init_road(interface);
        road_tail = road.x(round(end/2));	% (m) road tail behind ego
        
        % initialize lanes and road edges
        [lane,road_edges] = init_lanes(road);
        
        % initialize road area
        road_area = init_road_area(...
            [0 road.x(end)],[-road.x(end)/2 road.x(end)/2],road);

        % motion equation for ego and moving objects
        x_t = @(v,t,x_1) v.*t - road.x(end).*floor(x_1/road.x(end));

        % initialize ego
        ego = init_ego(x_t, road);

        % initialize standing objects
        stand_n_objects = get(...
            interface.figures.main.sliders.controls_main_parameters_stand_n,...
            'value');
        stand = init_stand(stand_n_objects,road,road_area);

        % initialize moving and oncoming objects
        mov_n_objects = 2;      % number of moving objects
        mov = init_mov_objects(mov_n_objects-1, 1,x_t,ego,road);% moving obj
        onc = init_mov_objects(mov_n_objects,-1,x_t,ego,road);% oncoming obj
        
        % reset plots if already existent
        if isfield(interface.figures.main,'ax_init') % not yet initialized
            reset_plots(interface);
        end
        
        % initialize simulation animation variables
        [interface,road,lane,road_edge,road_area,ego,stand,mov,onc] = ...
                    init_plots(interface,road,lane,road_edges,road_area,...
                    ego,stand,mov,onc);
        
        % output data to base workspace
        output_sim_data(...
            interface,road,lane,road_edge,road_area,...
            ego,stand,mov,onc,t,dt,road_tail)
        
        % run one simulation cycle to finish initialization
        controls_main_play_Callback(...
            interface.figures.main.buttons.controls_main_play)
        
        % reset start button string
        set(interface.figures.main.buttons.controls_main_play,...
            'string','Start')
        
    end
    
    function user_code_selectFunction_Callback(source,~)
        
        % load interface from base workspace
        sim_world_data = evalin('base','sim_world_data');
        interface = sim_world_data.interface;
        
        proceed = true;
        switch ui_get_user_code_Function(source)
            case 'Reconstruct 360'
                path = interface.files.reconstruct_360_space.path;
            otherwise
                proceed = false;
        end
        
        if proceed
            % load code from file
            init_user_code(interface, path)

            % assign to base workspace
            sim_world_data.interface = interface;
            assignin('base','sim_world_data', sim_world_data)
        end
    end

    function user_code_load_Callback(~,~)
        %prompt user to select file
        [filename, pathname] = uigetfile('*.m');
        
        if filename
            
            % add folder to search path
            addpath(pathname)
            
            % build bath name
            path = [pathname filename];
            
            % get function name
            func = filename(1:end-2);
            
            % load interface from base workspace
            sim_world_data = evalin('base','sim_world_data');
            interface = sim_world_data.interface;

            switch ui_get_user_code_Function([])
                case 'Reconstruct 360'
                    interface.files.reconstruct_360_space.path = path;
                    interface.files.reconstruct_360_space.func = func;
            end
            
            % enable save button
            set(interface.figures.main.buttons.user_code_save,'enable','on');

            % enable autosave checkbox
            set(interface.figures.main.checkboxes.user_code_autosave,...
                'enable','on')
            
            % update interface in base workspace
            sim_world_data.interface = interface;
            assignin('base','sim_world_data',sim_world_data)

            user_code_selectFunction_Callback(...
                interface.figures.main.popups.user_code_selectFunction)
        end
        
    end

    function user_code_save_Callback(~,~)
        
        % load interface from base workspace
        interface = evalin('base','sim_world_data.interface');
        path = interface.files.reconstruct_360_space.path;
        
        % save file
        save_user_code(interface, path)
    end

    function user_code_saveas_Callback(~,~)
        %prompt user to select file
        [filename, pathname] = uiputfile('*.m');
        
        if filename
            % add folder to search path
            addpath(pathname)
            
            % build bath name
            path = [pathname filename];
            
            % get function name
            func = filename(1:end-2);
            
            % load interface from base workspace
            sim_world_data = evalin('base','sim_world_data');
            interface = sim_world_data.interface;

            switch ui_get_user_code_Function([])
                case 'Reconstruct 360'
                    interface.files.reconstruct_360_space.path = path;
                    interface.files.reconstruct_360_space.func = func;
            end
            
            % save file
            save_user_code(interface, path)
            
            % enable save button
            set(interface.figures.main.buttons.user_code_save,'enable','on');

            % enable autosave checkbox
            set(interface.figures.main.checkboxes.user_code_autosave,...
                'enable','on')
            
            % update interface in base workspace
            sim_world_data.interface = interface;
            assignin('base','sim_world_data',sim_world_data)
            
            
        end
    end
    
    function controls_main_showSensor_Callback(source,~)
        % get interface from base workspace
        checkboxes = evalin('base',...
            'sim_world_data.interface.figures.main.checkboxes');
        
        str = get(source,'string');
        val = get(source,'value');
        option = str{val};
       
        switch option
            case 'Hide all' % hide all sensor detections
                set(checkboxes.controls_main_showSensor_FL,...
                    'value',0,'enable','off')
                set(checkboxes.controls_main_showSensor_FR,...
                    'value',0,'enable','off')
                set(checkboxes.controls_main_showSensor_RL,...
                    'value',0,'enable','off')
                set(checkboxes.controls_main_showSensor_RR,...
                    'value',0,'enable','off')
            case 'Select'   % allow user selection
                set(checkboxes.controls_main_showSensor_FL,...
                    'enable','on')
                set(checkboxes.controls_main_showSensor_FR,...
                    'enable','on')
                set(checkboxes.controls_main_showSensor_RL,...
                    'enable','on')
                set(checkboxes.controls_main_showSensor_RR,...
                    'enable','on')
            case 'Show all' % show all sensor detections
                set(checkboxes.controls_main_showSensor_FL,...
                    'value',1,'enable','off')
                set(checkboxes.controls_main_showSensor_FR,...
                    'value',1,'enable','off')
                set(checkboxes.controls_main_showSensor_RL,...
                    'value',1,'enable','off')
                set(checkboxes.controls_main_showSensor_RR,...
                    'value',1,'enable','off')
        end
        % apply changes
        controls_main_showSensor_cb_Callback(...
            checkboxes.controls_main_showSensor_FL)
        controls_main_showSensor_cb_Callback(...
            checkboxes.controls_main_showSensor_FR)
        controls_main_showSensor_cb_Callback(...
            checkboxes.controls_main_showSensor_RL)
        controls_main_showSensor_cb_Callback(...
            checkboxes.controls_main_showSensor_RR)
    end

    function controls_main_showSensor_cb_Callback(source,~)
        
        % get checkboxes interface from base workspace
        checkboxes = evalin('base',...
            'sim_world_data.interface.figures.main.checkboxes');
        
        % get sensor info from base workspace
        sensor = evalin('base','sim_world_data.ego.sensor');
        
        if get(source,'value')  % get checkbox state
            state = 'on';
        else
            state = 'off';
        end
        
        % identify corresponding FoV
        switch source
            case checkboxes.controls_main_showSensor_FL
                mi = find(strcmp(sensor.key,'FL'));
            case checkboxes.controls_main_showSensor_FR
                mi = find(strcmp(sensor.key,'FR'));
            case checkboxes.controls_main_showSensor_RL
                mi = find(strcmp(sensor.key,'RL'));
            case checkboxes.controls_main_showSensor_RR
                mi = find(strcmp(sensor.key,'RR'));
        end
        
        % change FoV visibiltity
        set(sensor.m(mi),'visible',state)
    end
    
    function match_slider_2_edit(source,slider,edit,func)
        
        % match value on slider-edit pair
        switch source
            case slider
                value = func(get(slider,'value'));
                % display value in text box
                set(edit,'string',num2str(value))
                set(source,'value',value)
            case edit
                value = max(get(slider,'min'),min(get(slider,'max'),...
                    func(str2double(get(edit,'string')))));
                % set slider value
                set(source,'string',num2str(value));
                set(slider,'value',value)
        end
    end

    function controls_main_parameters_dt_Callback(source,~)
        % get controls from base workspace
        ctrl = evalin('base','sim_world_data.interface.figures.main');
        
        % round value to 2 decimal places
        func = @(v) round(v,2);
        
        % match slider to edit
        match_slider_2_edit(...
            source,...
            ctrl.sliders.controls_main_parameters_dt,...
            ctrl.edits.controls_main_parameters_dt,...
            func);
    end

    function controls_main_parameters_stand_n_Callback(source,~)
        
        % get controls from base workspace
        ctrl = evalin('base','sim_world_data.interface.figures.main');
        
        % match slider to edit
        match_slider_2_edit(...
            source,...
            ctrl.sliders.controls_main_parameters_stand_n,...
            ctrl.edits.controls_main_parameters_stand_n,...
            @round);
        
    end

    function controls_main_parameters_terrain_h_Callback(source,~)
        % get controls from base workspace
        ctrl = evalin('base','sim_world_data.interface.figures.main');
        
        % round to two decimal places. Push to zero if < 5
        func = @(v) round(v,2).* (v >= 5);
        
        % match slider to edit
        match_slider_2_edit(...
            source,...
            ctrl.sliders.controls_main_parameters_terrain_h,...
            ctrl.edits.controls_main_parameters_terrain_h,...
            func);
    end

    function f_CloseRequestFcn(source,~)
        
        % get interface from main workspace
        interface = evalin('base','sim_world_data.interface');
        
        % hide figure
        set(source,'visible','off')
        
        switch source
            case interface.figures.visu.f
                                set(interface.figures.main.axes.dynamic,...
                    'parent',interface.figures.main.panels.playback_main,...
                    'position',interface.figures.main.axes.dynamic_pos)
                set(interface.figures.main.buttons.ax_dynamic_pop,...
                    'string','Pop')
        end
    end
    
end
