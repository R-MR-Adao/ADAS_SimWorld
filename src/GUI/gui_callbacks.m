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
        funcs = evalin('base','sim_world_data.funcs');
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
            funcs.sim.read_sim_data();
         funcs.sim.master_run(sim_world_data,interface,funcs,source,road,lane,road_edge,road_area,...
            ego,stand,mov,onc,road_tail,dt,t);
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
end