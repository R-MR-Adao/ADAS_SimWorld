function sim_world_data = gui_callbacks(sim_world_data)
    
    % expose public functions
    sim_world_data.funcs.gui_callbacks.controls_main_reset_Callback =...
         @(source,eventData) controls_main_reset_Callback();
    
     sim_world_data.funcs.gui_callbacks.user_code_selectFunction_Callback = ...
        @(source,eventData) user_code_selectFunction_Callback(source);
    
    sim_world_data.funcs.gui_callbacks.axes_dynamic_perspective_Callback =...
        @(source,eventData) axes_dynamic_perspective_Callback(source);
     
    sim_world_data.funcs.gui_callbacks.controls_main_play_Callback =...
        @(source,eventData) controls_main_play_Callback(source);
    
    sim_world_data.funcs.gui_callbacks.controls_main_step_Callback = ...
        @(source,eventData) controls_main_step_Callback(source);
    
    sim_world_data.funcs.gui_callbacks.controls_main_showSensor_Callback = ...
        @(source,eventData) controls_main_showSensor_Callback(source);
    
    sim_world_data.funcs.gui_callbacks.ax_dynamic_pop_Callback = ...
        @(source,eventData) ax_dynamic_pop_Callback(source);
    
    sim_world_data.funcs.gui_callbacks.user_code_load_Callback = ...
        @(source,eventData) user_code_load_Callback();
    
    sim_world_data.funcs.gui_callbacks.user_code_save_Callback = ...
        @(source,eventData) user_code_save_Callback();
    
    sim_world_data.funcs.gui_callbacks.user_code_saveas_Callback = ...
        @(source,eventData) user_code_saveas_Callback();
    
    sim_world_data.funcs.gui_callbacks.controls_main_showSensor_cb_Callback = ...
        @(source,eventData) controls_main_showSensor_cb_Callback(source);
    
    sim_world_data.funcs.gui_callbacks.controls_main_parameters_dt_Callback = ...
        @(source,eventData) controls_main_parameters_dt_Callback(source);
    
    sim_world_data.funcs.gui_callbacks.controls_main_parameters_stand_n_Callback = ...
        @(source,eventData) controls_main_parameters_stand_n_Callback(source);
    
    sim_world_data.funcs.gui_callbacks.controls_main_parameters_terrain_h_Callback = ...
        @(source,eventData) controls_main_parameters_terrain_h_Callback(source);
    
    sim_world_data.funcs.gui_callbacks.controls_main_parameters_terrain_n_Callback = ...
        @(source,eventData) controls_main_parameters_terrain_n_Callback(source);
    
    sim_world_data.funcs.gui_callbacks.f_CloseRequestFcn = ...
        @(source,eventData) f_CloseRequestFcn(source);
    
    % *********************** function definitions ***********************
    
    function user_code_selectFunction_Callback(source,~)
        % ADAS SimWorld: Callback of user function selector
        
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
  
    function controls_main_reset_Callback(~,~)
        % ADAS SimWorld: Reset simulation
        
        % get interface from base workspace
        sim_world_data = evalin('base','sim_world_data');
        interface = sim_world_data.interface;
        funcs = sim_world_data.funcs;
        
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
        road = funcs.sim.init_road(interface);
        road_tail = road.x(round(end/2));	% (m) road tail behind ego
        
        % initialize lanes and road edges
        [lane,road_edges] = funcs.sim.init_lanes(road);
        
        % initialize road area
        road_area = funcs.sim.init_road_area(...
            [0 road.x(end)],[-road.x(end)/2 road.x(end)/2],road);

        % motion equation for ego and moving objects
        x_t = @(v,t,x_1) v.*t - road.x(end).*floor(x_1/road.x(end));

        % initialize ego
        ego = funcs.sim.init_ego(x_t, road);

        % initialize standing objects
        stand_n_objects = get(...
            interface.figures.main.sliders.controls_main_parameters_stand_n,...
            'value');
        stand = funcs.sim.init_stand(stand_n_objects,road,road_area);

        % initialize moving and oncoming objects
        mov_n_objects = 2;      % number of moving objects
        mov = funcs.sim.init_mov_objects(...
            mov_n_objects-1, 1,x_t,ego,road);% moving obj
        onc = funcs.sim.init_mov_objects(...
            mov_n_objects,-1,x_t,ego,road);% oncoming obj
        
        % reset plots if already existent
        if isfield(interface.figures.main,'ax_init') % not yet initialized
            funcs.interface.reset_plots(interface);
        end
        
        % initialize simulation animation variables
        [interface,road,lane,road_edge,road_area,ego,stand,mov,onc] = ...
            funcs.graphics.init_plots(...
            interface,funcs,road,lane,road_edges,road_area,ego,stand,mov,onc);
        
        % reset speedometer widget
        interface.widgets.speedometer.d_total = 0;
                
        % output data to base workspace
        funcs.sim.output_sim_data(sim_world_data,...
            interface,road,lane,road_edge,road_area,...
            ego,stand,mov,onc,t,dt,road_tail)
        
        % run one simulation cycle to finish initialization
        funcs.gui_callbacks.controls_main_play_Callback(...
            interface.figures.main.buttons.controls_main_play)
        
        % reset start button string
        set(interface.figures.main.buttons.controls_main_play,...
            'string','Start')
        
    end

    function axes_dynamic_perspective_Callback(source,~)
        % ADAS SimWorld: Change plot perspective
        
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

    function controls_main_play_Callback(source,~)
        % ADAS SimWorld: Play simulation
        
        sim_world_data = evalin('base','sim_world_data');
        interface = sim_world_data.interface;
        funcs = sim_world_data.funcs;
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
                    clear interface.files.reconstruct_360_space.func
                    wb = waitbar(0,'Saving file');
                    user_code_save_Callback();
                    for ii =1:75
                        waitbar(ii/75) % wait a moment for file to save
                    end
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
            funcs.sim.read_sim_data();
         funcs.sim.master_run(sim_world_data,interface,funcs,source,road,lane,road_edge,road_area,...
            ego,stand,mov,onc,road_tail,dt,t);
    end

    function controls_main_step_Callback(source,~)
        % ADAS SimWorld: Simulate a single step
        
        interface = evalin('base','sim_world_data.interface');
        set(interface.figures.main.buttons.controls_main_step,'value',0)
        controls_main_play_Callback(...
            source)
    end

    function controls_main_showSensor_Callback(source,~)
        % ADAS SimWorld: Select sensor display options
        
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

    function ax_dynamic_pop_Callback(source,~)
        % ADAS SimWorld: Pop main plot to visu figure
        
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

    function user_code_load_Callback(~,~)
        % ADAS SimWorld: Load user code file
        
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
        % ADAS SimWorld: Save user code file
        
        % load interface from base workspace
        interface = evalin('base','sim_world_data.interface');
        path = interface.files.reconstruct_360_space.path;
        
        % save file
        save_user_code(interface, path)
    end

    function user_code_saveas_Callback(~,~)
        % ADAS SimWorld: Save user code file browse
        
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

    function controls_main_showSensor_cb_Callback(source,~)
        % ADAS SimWorld: Sensor display checkboxes
        
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

    function controls_main_parameters_dt_Callback(source,~)
        % ADAS SimWorld: Choose render parameters - time resolution
        
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
        % ADAS SimWorld: Choose render parameters - number of standing objs
        
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
        % ADAS SimWorld: Choose render parameters - terrain depth
        
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

    function controls_main_parameters_terrain_n_Callback(source,~)
        % ADAS SimWorld: Choose render parameters - terrain pixels
        
        % get controls from base workspace
        ctrl = evalin('base','sim_world_data.interface.figures.main');
        
        % match slider to edit
        match_slider_2_edit(...
            source,...
            ctrl.sliders.controls_main_parameters_terrain_n,...
            ctrl.edits.controls_main_parameters_terrain_n,...
            @round);
        
    end

    function f_CloseRequestFcn(source,~)
        % ADAS SimWorld: Close figure
        
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

    %************************* Internal functions *************************
    
    function func = ui_get_user_code_Function(source)
        % ADAS SimWorld: get user-selected functioin
        
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

    function init_user_code(interface, fname)
        % ADAS SimWorld: initialize user code textbox
        
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

    function match_slider_2_edit(source,slider,edit,func)
        % ADAS SimWorld: Sync render option slider and textbox
        
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

    function save_user_code(interface, fname)
        % ADAS SimWorld: Save user code file
        
        fID = fopen(fname,'w');         % file ID
        eTxt = get(interface.figures.main.edits.user_code,'string');
        
        for ii = 1 : length(eTxt)
           fprintf(fID,'%s\n', eTxt{ii});
        end

        fclose(fID);
    end
end
