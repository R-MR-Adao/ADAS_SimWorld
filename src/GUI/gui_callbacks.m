function sim_world_data = gui_callbacks(sim_world_data)
    
    % expose public functions
    sim_world_data.funcs.gui_callbacks.user_code_selectFunction_Callback = ...
        @(source,eventData) user_code_selectFunction_Callback(source);
    
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