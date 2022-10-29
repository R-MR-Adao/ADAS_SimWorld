function sim_world_data = user_code_api(sim_world_data)
    
    sim_world_data.funcs.user_api.fill_input_reconstruct_360_space =...
        @(interface,ego,t,dt) fill_input_reconstruct_360_space(...
            interface,ego,t,dt);
        
    sim_world_data.funcs.user_api.execute_reconstruct_360_space = ...
        @(interface,sensor_f,ego_f) execute_reconstruct_360_space(...
        interface,sensor_f,ego_f);
        
    sim_world_data.funcs.user_api.calculate_user_z = ...
        @(funcs,road,obj_u,stand_u,mov_u,onc_u,road_area,ego,interface)...
        calculate_user_z(...
            funcs,road,obj_u,stand_u,mov_u,onc_u,road_area,ego,interface);
    
    sim_world_data.funcs.user_api.update_plot_dynamic_user = ...
        @(ego,obj_u,stand,stand_u,mov,mov_u,onc,onc_u) ...
        update_plot_dynamic_user(...
            ego,obj_u,stand,stand_u,mov,mov_u,onc,onc_u);
        
    % *********************** function definitions ***********************

    function [sensor_f,ego_f] = fill_input_reconstruct_360_space(...
            interface,ego,t,dt)
        % ADAS SimWorld: Fill inputs for user code
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
                sensor_f.data{ii} = ...   % bunch all sensor objects
                    [ego.sensor.data(ii).stand(...
                    ~isnan(ego.sensor.data(ii).stand(:,1)),:);...
                    ego.sensor.data(ii).mov(...
                    ~isnan(ego.sensor.data(ii).mov(:,1)),:);...
                    ego.sensor.data(ii).onc(...
                    ~isnan(ego.sensor.data(ii).onc(:,1)),:)]; 
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
    
    function [obj_u,stand_u,mov_u,onc_u] = execute_reconstruct_360_space(...
            interface,sensor_f,ego_f)
        switch nargout(interface.files.reconstruct_360_space.func)
            case 1                       % returns only rotated objects
                obj_u = eval(...
                    [interface.files.reconstruct_360_space.func,...
                    '(sensor_f)']);
                % define user (un)classified object lists as empty arrs
                stand_u = [];
                mov_u   = [];
                onc_u   = [];
            case 4                       % returns classified objects
                [obj_u,stand_u,mov_u,onc_u] = eval(...
                    [interface.files.reconstruct_360_space.func,...
                    '(sensor_f,ego_f)']);
        end
    end

    function [obj_u,stand_u,mov_u,onc_u] = calculate_user_z(funcs,...
            road,obj_u,stand_u,mov_u,onc_u,...
                road_area,ego,interface)
            % ADAS SimWorld: Calculate z of user-identified objects
            
            if ~isempty(obj_u)
                if road.terrain.a > 0 && ...
                        get(interface.figures.main.sliders.ax_dynamic_tilt,'value')...
                        < 70                        
                    % calculate standing object z in their original position
                    %   1) cancel ego rotation
                    [obj_x,obj_y] = funcs.sim.dynamic_transform_coordinates(...
                        obj_u(:,1),obj_u(:,2),0,0,-ego.theta);
                    %   2) calculate road_area z in the object positions
                    obj_u = [obj_u road_area.Z(...
                        obj_x,ego.x_1,obj_y,ego.y_1)];
                    if ~isempty(stand_u)
                        %   1) cancel ego rotation
                        [stand_x,stand_y] = funcs.sim.dynamic_transform_coordinates(...
                            stand_u(:,1),stand_u(:,2),0,0,-ego.theta);
                        %   2) calculate road_area z in the object positions
                        stand_u = [stand_u road_area.Z(...
                            stand_x,ego.x_1,stand_y,ego.y_1)];
                    else
                        stand_u = [stand_u zeros(size(stand_u,1),1)];
                    end

                else % approximate standing object z to zero
                    obj_u = [obj_u zeros(size(obj_u,1),1)];
                    stand_u = [stand_u zeros(size(stand_u,1),1)];
                end

                % moving and oncoming objects must have z = 0
                mov_u = [mov_u zeros(size(mov_u,1),1)];
                onc_u = [onc_u zeros(size(onc_u,1),1)];
            end
    end
    
    function update_plot_dynamic_user(...
            ego,obj_u,stand,stand_u,mov,mov_u,onc,onc_u)
        % ADAS SimWorld: Update plots based on user code results
        
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

end
