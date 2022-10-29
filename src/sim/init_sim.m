function sim_world_data = init_sim(sim_world_data)

    % expose public functions
    sim_world_data.funcs.sim.init_road = ...
        @(interface) init_road(interface);
    
    sim_world_data.funcs.sim.init_mov_lane = ...
        @(road, off) init_mov_lane(road, off);
    
    sim_world_data.funcs.sim.init_lane = ...
        @(road,off) init_lane(road,off);
    
    sim_world_data.funcs.sim.init_lanes = ...
        @(road) init_lanes(road);
    
    sim_world_data.funcs.sim.init_road_area = ...
        @(varargin) init_road_area(...
        varargin);
    
    sim_world_data.funcs.sim.init_ego = ...
        @(x_t, road) init_ego(x_t, road);
    
    sim_world_data.funcs.sim.init_stand = ...
        @(n,road,road_area) init_stand(n,road,road_area);
    
    sim_world_data.funcs.sim.init_mov_objects = ...
        @(n, direction, x_t, ego, road) init_mov_objects(...
        n, direction, x_t, ego, road);
    
    sim_world_data.funcs.sim.init_cube = ...
        @(varargin) init_cube(varargin);
   
    % *********************** function definitions ***********************
    
    function obj = init_road(interface)
        % ADAS SimWorld: Initialize road
        
        obj.T = 80;            % (m) road periodicity
        obj.x = 0:0.2:240;     % (m) road x array
        obj.y = @(x) ...       % (m) road y array
            obj.T/3*sin(2*pi/obj.T*x).*cos(2*pi/obj.T/3*x);
        
        % 3D terrain properties
        sliders = interface.figures.main.sliders; % interface sliders
        obj.terrain.n = ...    % terrain number of pixels
            get(sliders.controls_main_parameters_terrain_n,'value');
        obj.terrain.a = ...    % terrain dip amplitude
            get(sliders.controls_main_parameters_terrain_h,'value');
        obj.terrain.c = 50;    % terrain dip width
    end

    function obj = init_mov_lane(road, off)
        % ADAS SimWorld: Initialize lane of moving objects
        
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
        % ADAS SimWorld: Initialize road lane
        
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
        % ADAS SimWorld: Initialize road lanes
        
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

    function [road_area, X, Y] = init_road_area(varargin)
        % ADAS SimWorld: Initialize road area
        
        xl   = varargin{1}{1};
        yl   = varargin{1}{2};
        road = varargin{1}{3};
       
        % set default road_area type to surf
        if length(varargin{1}) < 4 
            type = 'surf';
        else
            type = varargin{1}{4};
            x    = varargin{1}{5};
            ra_l = varargin{1}{6};
            ra_r = varargin{1}{7};
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
        % ADAS SimWorld: Initialize ego vehicle
        
        obj = [];                % ego properties
        obj.v = 8;              % (m/s) ego speed
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
        % ADAS SimWorld: Initialize standing objects
        
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
        % ADAS SimWorld: Initialize moving objects
        
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

    function cube = init_cube(varargin)
        % ADAS SimWorld: Initialize cube 3D shape
        
        if length(varargin{1}) > 1
            cube       = varargin{1}{1};
            dimensions = varargin{1}{2};
            center     = varargin{1}{3};
            theta      = varargin{1}{4};
        else
            if iscell(varargin{1})
                cube = varargin{1}{1};
            else
                cube = varargin{1};
            end
            dimensions = cube.dimensions;   % cube dimensions
            center = cube.center;           % cube center position
            theta = cube.theta;             % cube orientaton
        end
        
        rd = @(x,y,t) [x(:),y(:)]*[cosd(t) -sind(t) ;... % rotation matrix
                                   sind(t)  cosd(t)];

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

end