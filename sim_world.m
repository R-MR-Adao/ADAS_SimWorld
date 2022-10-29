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
    
    % output data to base workspace
    assignin('base','sim_world_data',sim_world_data)
    
    % initialize object properties
    sim_world_data.funcs.gui_callbacks.controls_main_reset_Callback()
    
end
