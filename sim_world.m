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
    end
    
    output_sim_data(interface,road,ego,stand,mov,onc) 
    
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
        set(ego.m.static,'xData',ego_y,'yData',ego_x)
        ego.x_1 = ego.x(t,0);
        set(mov.m.static,'xData',mov_y,'yData',mov_x)
        mov.x_1 = mov.x(t,0);
        set(onc.m.static,'xData',onc_y,'yData',onc_x)
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
        
        % update dynamic axes
        set(road.m.dynamic, 'xData',road_y, 'yData',road_x)
        set(stand.m.dynamic,'xData',stand_y,'ydata',stand_x);
        set(mov.m.dynamic,  'xData',mov_y,  'yData',mov_x)
        set(onc.m.dynamic,  'xData',onc_y,  'yData',onc_x)
        
        % increment time
        t = t + dt;
        pause(dt)
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
        obj.sensor.fov.draw.s = ...    % sensor drawing parameter
            (-0.5:0.02:0.5)'*obj.sensor.fov.theta;
        % initialize sensor specific coverage circles
        for jj = 1 : obj.sensor.n
            obj.sensor.fov.draw.circ{jj} = obj.sensor.fov.range*...
                [[0, 0];...
                [cosd(obj.sensor.fov.draw.s+obj.sensor.theta(jj)),...
                sind(obj.sensor.fov.draw.s+obj.sensor.theta(jj))];...
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
            'xlim', [-1 1]*road.T/2,...
            'ylim', [road.x(1) road.x(end)]);
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
            r_y, r_x,'w','linewidth',2);
        ego.m.static =  plot(interface.main_figure.ax_static,...
            e_y,e_x,'or','linewidth',2);
        stand.m.static = plot(interface.main_figure.ax_static,...
            stand.y, stand(1).x,'og','linewidth',2);
        mov.m.static =  plot(interface.main_figure.ax_static,...
            m_y,m_x,'o','color',[1 0.5 0],'linewidth',2);
        onc.m.static =  plot(interface.main_figure.ax_static,...
            o_y,o_x,'oc','linewidth',2);

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

        % draw sensor FoV
        for ii = 1 : ego.sensor.n
            ego.sensor.m(ii) = patch(...
                ego.sensor.fov.draw.circ{ii}(:,1),...
                ego.sensor.fov.draw.circ{ii}(:,2),...
                'w','FaceAlpha',.3);
        end
    end

    % ***************************** interface *****************************
    
    function interface = init_interface()
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