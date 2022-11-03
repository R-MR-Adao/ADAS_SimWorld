function sim_world_data = update_sim(sim_world_data)

    % expose public functions
     sim_world_data.funcs.sim.master_run = ...
         @(sim_world_data,interface,funcs,source,road,lane,...
         road_edge,road_area,ego,stand,mov,onc,road_tail,dt,t) master_run(...
            sim_world_data,interface,funcs,source,road,lane,...
            road_edge,road_area,ego,stand,mov,onc,road_tail,dt,t);
    
    sim_world_data.funcs.sim.simulation_run = ...
        @(interface,funcs,t,dt,road,lane,road_edge,road_area,...
        ego,stand,mov,onc,road_tail) simulation_run(...
            interface,funcs,t,dt,road,lane,road_edge,road_area,...
            ego,stand,mov,onc,road_tail);
    
    sim_world_data.funcs.sim.split_road_edges =...
        @(road_edge_x,road_edge_y) split_road_edges(...
            road_edge_x,road_edge_y);
    
    sim_world_data.funcs.sim.find_road_points =...
        @(road_edge_x,road_edge_y,ego,ego_y,p_x) find_road_points(...
            road_edge_x,road_edge_y,ego,ego_y,p_x);
        
    sim_world_data.funcs.sim.find_road_area = ...
        @(varargin) find_road_area(varargin);
    
    sim_world_data.funcs.sim.read_sim_data = @() read_sim_data();
        
    sim_world_data.funcs.sim.output_sim_data = @(...
        sim_world_data,interface,road,lane,road_edge,road_area,...
        ego,stand,mov,onc,t,dt,road_tail) output_sim_data(...
            sim_world_data,interface,road,lane,road_edge,road_area,...
            ego,stand,mov,onc,t,dt,road_tail);
        
    sim_world_data.funcs.sim.dynamic_transform_coordinates =...
        @(varargin) dynamic_transform_coordinates(varargin);
    
    % *********************** function definitions ***********************
        
    function master_run(sim_world_data,interface,funcs,...
            source,road,lane,road_edge,road_area,...
            ego,stand,mov,onc,road_tail,dt,t)
        % ADAS SimWorld: Main simulation run function
        
        override = true;    % override bool to implement do while
        
        while (get(interface.figures.main.buttons.controls_main_play,'value') ...
                || override)

            % run simulation
            [road,lane,road_edge,road_area,ego,stand,mov,onc,road_tail] = ...
                simulation_run(interface,funcs,t,dt,road,lane,road_edge,road_area,...
                ego,stand,mov,onc,road_tail);
            
            % update widgets
            interface.widgets.speedometer = ...
                funcs.widgets.update_speedometer(...
                interface.widgets.speedometer,ego,t,dt);
            
            % fill inputs for user reconstruction
            [sensor_f, ego_f] = funcs.user_api.fill_input_reconstruct_360_space(...
                interface,ego,t,dt);
            
            % recover 360 degree coordinate space
            [obj_u,stand_u,mov_u,onc_u] =...
                funcs.user_api.execute_reconstruct_360_space(...
                interface,sensor_f,ego_f);
            
            % calculate z coordinates for user detections
            [obj_u,stand_u,mov_u,onc_u] = funcs.user_api.calculate_user_z(...
                funcs,road,obj_u,stand_u,mov_u,onc_u,...
                road_area,ego,interface);
            
            % draw recovered coordinates
            if get(interface.figures.main.buttons.controls_main_play,'value')...
                    || source == interface.figures.main.buttons.controls_main_step
                funcs.user_api.update_plot_dynamic_user(...
                    ego,obj_u,stand,stand_u,mov,mov_u,onc,onc_u);
            end

            % increment time
            t = t + dt;
            pause(0.012)

            % output data to base workspace
            output_sim_data(sim_world_data,...
                interface,road,lane,road_edge,road_area,...
                ego,stand,mov,onc,t,dt,road_tail)
            
            override = false;
        end
    end

    function [road,lane,road_edge,road_area,ego,stand,mov,onc,road_tail] =...
            simulation_run(interface,funcs,t,dt,...
            road,lane,road_edge,road_area,ego,stand,mov,onc,road_tail)
        % ADAS SimWorld: update SimWorld simulation
        
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
            {interface,funcs,road,road_area,road_edge_x,road_edge_y,ego,ego_y});
        
        % update static axis
        funcs.graphics.update_plot_static(ego,ego_x,ego_y,mov,mov_x,mov_y,onc,onc_x,onc_y)
        
        % ***************** update data for dynamic plot *****************
        
        % transform road coordinates
        [road_x,road_y,theta] = dynamic_transform_coordinates(...
            {road_x,road_y,ego.x_1,ego.y(ego.x_1)});
        ego.Dtheta = ego.theta - theta; % angular variation
        ego.theta = theta;              % ego orientation
        
        % transform lane coordinates
        [lane_x,lane_y] = dynamic_transform_coordinates(...
            {lane_x,lane_y,ego.x_1,ego.y(ego.x_1), theta});
        
        % transform road edge coordinates
        [road_edge_x,road_edge_y] = dynamic_transform_coordinates(...
            {road_edge_x,road_edge_y,ego.x_1,ego.y(ego.x_1), theta});
        
        % transform standing object coordinates
        s_shift = road.x(end)*((ego_x-stand.x) > road.x(round(end/2))) +... 
                 -road.x(end)*((stand.x-ego_x) > road.x(round(end/2)));
        [stand_x,stand_y] = dynamic_transform_coordinates(...
            {stand.x+s_shift,stand.y,ego_x,ego_y, theta});
        fov = [get(interface.figures.main.axes.dynamic,'ylim');... axes fov
            get(interface.figures.main.axes.dynamic,'xlim')]';
        stand = update_stand_object_cube(funcs,stand,stand_x,stand_y,theta,fov);
        
        % transform moving object corrdinates
        m_shift = road.x(end)*((ego_x-mov_x) > road.x(round(end/2))) +... 
                 -road.x(end)*((mov_x-ego_x) > road.x(round(end/2)));
        [mov_x,mov_y] = dynamic_transform_coordinates(...
            {mov_x+m_shift,mov_y,ego_x,ego_y, theta});
        mov = update_mov_object_cube(funcs,mov,mov_x,mov_y,theta,fov);
        
        % transform oncoming object corrdinates
        o_shift = road.x(end)*((ego_x-onc_x) > road.x(round(end/2))) +... 
                 -road.x(end)*((onc_x-ego_x) > road.x(round(end/2)));
        [onc_x,onc_y] = dynamic_transform_coordinates(...
            {onc_x+o_shift,onc_y,ego_x,ego_y, theta});
        onc = update_mov_object_cube(funcs,onc,onc_x,onc_y,theta,fov);
        
        % transform road map coordinates
        [ra_x,ra_y] = dynamic_transform_coordinates(...
                {road_area.X(:),road_area.Y(:),0,0, theta});
        
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
        funcs.graphics.update_plot_dynamic(...
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


    function [ra_l,ra_r,ra_c] = split_road_edges(road_edge_x,road_edge_y)
        % ADAS SimWorld: Split road edges into its three lines
        
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
        % ADAS SimWorld: Find arrays that make up the road edges
        
        % split road esges
        [ra_l,ra_r] = split_road_edges(road_edge_x,road_edge_y);
        
        % interpolate y coordinates
        ra_l = interp1(ra_l(:,1),ra_l(:,2),p_x+ego.x_1,'pchirp','extrap')-ego_y;
        ra_r = interp1(ra_r(:,1),ra_r(:,2),p_x+ego.x_1,'pchirp','extrap')-ego_y;
    end

    function road_area = find_road_area(varargin)
        % ADAS SimWorld: Find the area comprised by the road
        
        interface   = varargin{1}{1};
        funcs       = varargin{1}{2};
        road        = varargin{1}{3};
        road_area   = varargin{1}{4};
        road_edge_x = varargin{1}{5};
        road_edge_y = varargin{1}{6};
        ego         = varargin{1}{7};
        ego_y       = varargin{1}{8};
        
        if length(varargin{1}) < 9
            % dynamic x, y limits
            yl = get(interface.figures.main.axes.dynamic,'ylim');
            yl = mean(yl) + [-1 1]*diff(yl)*1.1; % expand to fit y span   
            % limit x interpolation range
            xl = min(yl,road.x(end)/2);
        else
            xl    = varargin{1}{9};
            yl    = varargin{1}{10};
        end
               
        % choose road area type
        if road.terrain.a > 0 &&...
                get(...
                interface.figures.main.sliders.ax_dynamic_tilt,'value') < 70
            type = 'surf';
            % initialize road area
            [~,road_area.X, road_area.Y] = funcs.sim.init_road_area(xl,yl,road,type);
            
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
        else
            type = 'patch';
            x = xl(1):diff(road.x(1:2))*15:xl(2);  % initialize x array
            % find road area coverage
            [ra_l,ra_r] = find_road_points(... 
                road_edge_x,road_edge_y,ego,ego_y,x);
                
            [~,road_area.X, road_area.Y] = funcs.sim.init_road_area(...
                xl,yl,road,type,x,ra_l,ra_r);
        end
        
        % set road area type
        road_area.type = type;
        
    end

    function [interface,road,lane,road_edge,road_area,...
            ego,stand,mov,onc,t,dt,road_tail] =...
            read_sim_data()
        % ADAS SimWorld: Read data from the base workspace
        
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

    function output_sim_data(sim_world_data,...
            interface,road,lane,road_edge,road_area,...
            ego,stand,mov,onc,t,dt,road_tail)
        % ADAS SimWorld: Output data to the base workspace
        
        sim_world_data.interface     = interface;
        sim_world_data.road          = road;
        sim_world_data.lane          = lane;
        sim_world_data.road_edge     = road_edge;
        sim_world_data.road_area     = road_area;
        sim_world_data.ego           = ego;
        sim_world_data.stand         = stand;
        sim_world_data.mov           = mov;
        sim_world_data.onc           = onc;
        sim_world_data.sim.t         = t;
        sim_world_data.sim.dt        = dt;
        sim_world_data.sim.road_tail = road_tail;
        assignin('base','sim_world_data',sim_world_data)
    end
    
    function [x,y,theta] = dynamic_transform_coordinates(varargin)
        % ADAS SimWorld: Determine and/or apply matrix rotation
        
        x     = varargin{1}{1};
        y     = varargin{1}{2};
        x_ref = varargin{1}{3};
        y_ref = varargin{1}{4};
        
        % rotation matrix
        r = @(x,y,t) [x(:),y(:)]*[cos(t) -sin(t);...
                                  sin(t)  cos(t)]';
        % rmove offset
        x = x - x_ref;
        y = y - y_ref;
        
        if length(varargin{1})  < 5
            %find root
            [~,i_root] = min(abs(x));

            % calculate rotation angle
            dx = x(i_root+1) - x(i_root);
            dy = y(i_root+1) - y(i_root);
            theta = -atan2(dy,dx);
        else
            theta = varargin{1}{5};
        end
        
        % apply rotation
        xy = r(x,y,theta);
        
        % return results
        x = xy(:,1);
        y = xy(:,2);
        
    end

    %************************* internal functions *************************

    function obj = update_stand_object_cube(funcs,obj,obj_x,obj_y,theta,fov)
        % ADAS SimWorld: Update the standing objects cube points
        
        for ii = 1 : obj.n % object shapes (visualization)
            if (fov(1,1) <= obj_x(ii)) && (obj_x(ii) <= fov(2,1)) && ...
                    (fov(1,2) <= obj_y(ii)) && (obj_y(ii) <= fov(2,2))
                obj.shape(ii).center = [obj_x(ii),obj_y(ii),0];
                obj.shape(ii).theta  = theta/pi*180;
                obj.shape(ii)        = funcs.sim.init_tree(obj.shape(ii));
            end
        end
    end

    function obj = update_mov_object_cube(funcs,obj,obj_x,obj_y,theta,fov)
        % ADAS SimWorld: Update the moving objects cube points
        
        for ii = 1 : obj.n % object cubes (visualization)
            if (fov(1,1) <= obj_x(ii)) && (obj_x(ii) <= fov(2,1)) && ...
                    (fov(1,2) <= obj_y(ii)) && (obj_y(ii) <= fov(2,2))
                obj.cube(ii).center = [obj_x(ii),obj_y(ii),0];
                obj.cube(ii).theta  = theta/pi*180 + obj.theta(ii);
                obj.cube(ii)        = funcs.sim.init_cube(obj.cube(ii));
            end
        end
    end

    function data = sensor_data_find(sensor,ii,data_x,data_y)
        % ADAS SimWorld: Find data in each sensor's FoV
        
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

end
