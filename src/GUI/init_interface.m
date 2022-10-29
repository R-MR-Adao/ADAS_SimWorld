function sim_world_data = init_interface()

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

    % expose public functions
    sim_world_data.funcs.interface.reset_plots = ...
        @(interface) reset_plots(interface);
    
    % import gui callback functions
    sim_world_data = gui_callbacks(sim_world_data);
    
    % import widget functions
    sim_world_data = widget_speedometer(sim_world_data);
    
    % initialize interface
    init();
    
    % *********************** function definitions ***********************
    
    function init()
        % ADAS SimWorld: Initialize interface

        interface = [];
        interface.colors.background       = [1 1 1]*0.25;
        interface.colors.plot_background  = [1 1 1]*0.4;
        interface.colors.plot_lines       = [1 1 1]*1;
        interface.colors.panel_background = [1 1 1]*0.3;
        interface.colors.font             = [1 1 1]*1;
        interface.colors.code_background  = [1 1 1]*0.2;
        interface.colors.code_font        = [0.9 1 0.8];

        % visu figure
        interface.figures.visu.f = figure(...
            'color',interface.colors.background,...
            'NumberTitle', 'off',...
            'name','ADAS SimWorld: visualizatrion',...
            'position',[1 31 1920 1020],...
            'MenuBar', 'none',...
            'visible','off',...
            'CloseRequestFcn',@f_CloseRequestFcn);
        colormap([bsxfun(@times,[50 150 0]/270,ones(100,3));[1 1 1]*0.3])
        init_fig_add_logo(interface.figures.visu.f)

        % main figure
        interface.figures.main.f = figure(...
            'NumberTitle', 'off',...
            'name','ADAS SimWorld',...
            'color',interface.colors.background,...
            'MenuBar', 'none',...
            'position',[1 31 1920 995],... 
            'CloseRequestFcn',@f_CloseRequestFcn);
        init_fig_add_logo(interface.figures.main.f)
        colormap([bsxfun(@times,[50 150 0]/270,ones(100,3));[1 1 1]*0.3])

        % About button
        uimenu(interface.figures.main.f,...
            'label','About',...
            'callback',@MenuSelected);

        % Set current directory
        p = mfilename('fullpath');
        cd(p(1:end-length('src\GUI\init_interface')))

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
        sim_world_data.funcs.gui_callbacks.user_code_selectFunction_Callback([]);
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
            'position', [0.05, 0.93, 0.25, 0.035],...
            'callback',@controls_main_play_Callback);
        init_ui_style(...
            interface.figures.main.buttons.controls_main_play,interface)

        % Step button
        interface.figures.main.buttons.controls_main_step = uicontrol(...
            'parent',interface.figures.main.panels.controls_main,...
            'units','normalized',...
            'style','toggle',...
            'string','Step',...
            'position', [0.35, 0.93, 0.25, 0.035],...
            'callback',@controls_main_step_Callback);
        init_ui_style(...
            interface.figures.main.buttons.controls_main_step,interface)

        % Reset button
        interface.figures.main.buttons.controls_main_reset = uicontrol(...
            'parent',interface.figures.main.panels.controls_main,...
            'units','normalized',...
            'style','pushbutton',...
            'string','Reset',...
            'position', [0.70, 0.93, 0.25, 0.035],...
            'callback',sim_world_data.funcs.gui_callbacks.controls_main_reset_Callback);
        init_ui_style(...
            interface.figures.main.buttons.controls_main_reset,interface)

        % ******************** sensor selection panel ********************
        interface.figures.main.panels.controls_main_showSensor = uipanel(...
            'parent',interface.figures.main.panels.controls_main,...
            'position',[0.03,0.8,0.94,0.1],...
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
            'position',[0.03,0.58,0.94,0.2],...
            'title','Simulation rendering (reset to apply)');
        init_ui_style(...
            interface.figures.main.panels.controls_main_parameters,...
            interface)

        % simulation time resolution
        controls_main_parameters_dt_yanchor = 0.9;

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
            'value',0.04,...
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
            'string', '0.04', ...
            'position', [0.81 controls_main_parameters_dt_yanchor-0.1 0.16 0.1],...
            'callback',@controls_main_parameters_dt_Callback);
        init_ui_style(...
            interface.figures.main.edits.controls_main_parameters_dt,...
            interface)

        % number of standing objects
        controls_main_parameters_stand_n_yanchor = 0.65 ;

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
        controls_main_parameters_terrain_h_yanchor = 0.40 ;

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

        % 3D terrain pixels
        controls_main_parameters_terrain_n_yanchor = 0.15 ;

        % 3D terrain pixels text
        interface.figures.main.texts.controls_main_parameters_terrain_n =...
            uicontrol(...
            'parent',interface.figures.main.panels.controls_main_parameters,...
            'units','normalized',...
            'style','text',...
            'string', '3D terrain pixels', ...
            'position', [0.03 controls_main_parameters_terrain_n_yanchor 0.35 0.1]);
        init_ui_style(...
            interface.figures.main.texts.controls_main_parameters_terrain_n,...
            interface)

        % 3D terrain pixels slider
        interface.figures.main.sliders.controls_main_parameters_terrain_n =...
            uicontrol(...
            'parent',interface.figures.main.panels.controls_main_parameters,...
            'units','normalized',...
            'style','slider',...
            'min', 50,...
            'max', 500,...
            'value',275,...
            'SliderStep', [5, 10]/450,...
            'position', [0.03 controls_main_parameters_terrain_n_yanchor-0.1 0.75 0.1],...
            'callback',@controls_main_parameters_terrain_n_Callback);
        init_ui_style(...
            interface.figures.main.sliders.controls_main_parameters_terrain_n,...
            interface)

        % 3D terrain pixels edit
        interface.figures.main.edits.controls_main_parameters_terrain_n =...
            uicontrol(...
            'parent',interface.figures.main.panels.controls_main_parameters,...
            'units','normalized',...
            'style','edit',...
            'string', '275', ...
            'position', [0.81 controls_main_parameters_terrain_n_yanchor-0.1 0.16 0.1],...
            'callback',@controls_main_parameters_terrain_n_Callback);
        init_ui_style(...
            interface.figures.main.edits.controls_main_parameters_terrain_n,...
            interface)

        % speedometer animation 
        interface = sim_world_data.funcs.widgets.init_speedometer(interface);

        % assign interface to base workspace
        sim_world_data.interface = interface;
        assignin('base','sim_world_data',sim_world_data)
    
    end
    
    %******************** nested function definitions ********************
    
    
    function MenuSelected(~,~)
        % ADAS SimWorld: about button callback

        CC0 = imread('cfg\CC0.png');
        mb = msgbox(...
            {'Created by Ricardo M. R. Adao, 2022',...
            'https://github.com/R-MR-Adao/ADAS_SimWorld',...
            '',...
            ['This work is licensed under a Creative Commons ',...
            'CC0 1.0 Universal (CC0 1.0) Public Domain Dedication']},...
            'ADAS SimWorld',...
            'custom',CC0);
        init_fig_add_logo(mb)
    end

    function init_fig_add_logo(f)
        % ADAS SimWorld: add logo

        warning('off','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');
        jframe=get(f,'javaframe');
        jIcon=javax.swing.ImageIcon('cfg\logo_rmra.png');
        jframe.setFigureIcon(jIcon);
    end
    
    function init_ui_style(h, interface)
        % ADAS SimWorld: set ui control properties
        set(h,...
            'backgroundcolor',interface.colors.panel_background,...
            'foregroundcolor',interface.colors.font)
    end

    function init_axes_style(ax, interface)
        % ADAS SimWorld: set axes properties
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

    function reset_plots(interface)
        % ADAS SimWorld: reset plots
        
        cla(interface.figures.main.axes.static);
        init_axes_style(interface.figures.main.axes.static, interface)
        cla(interface.figures.main.axes.dynamic);
        init_axes_style(interface.figures.main.axes.dynamic, interface)
        for ii = 1 : length(interface.figures.main.axes.sensor)
            cla(interface.figures.main.axes.sensor(ii));
            init_axes_style(interface.figures.main.axes.sensor(ii), interface)
        end
    end

end
