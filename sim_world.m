function sim_world()
    
    % initialize interface
    sim_world_data = init_interface();
    
    % initialize simulation functions
    sim_world_data = init_sim(sim_world_data);
    sim_world_data = update_sim(sim_world_data);
    
    %initialize graphics functions
    sim_world_data = init_graphics(sim_world_data);
    sim_world_data = update_graphics(sim_world_data);
    
    % initialize user code api
    sim_world_data = user_code_api(sim_world_data);
    
    assignin('base','sim_world_data',sim_world_data)
    
    % ******************* initialize object properties *******************
    sim_world_data.funcs.gui_callbacks.controls_main_reset_Callback()
    
    % *********************** function definitions ***********************
    


    % ***************************** interface *****************************

    function save_user_code(interface, fname)
        fID = fopen(fname,'w');         % file ID
        eTxt = get(interface.figures.main.edits.user_code,'string');
        
        for ii = 1 : length(eTxt)
           fprintf(fID,'%s\n', eTxt{ii});
        end

        fclose(fID);
    end


% ************************** interface callbacks **************************

    function controls_main_step_Callback(source,~)
        interface = evalin('base','sim_world_data.interface');
        set(interface.figures.main.buttons.controls_main_step,'value',0)
        controls_main_play_Callback(...
            source)
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

    function controls_main_parameters_terrain_n_Callback(source,~)
        
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
