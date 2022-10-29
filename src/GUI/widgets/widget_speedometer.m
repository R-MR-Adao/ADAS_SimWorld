function sim_world_data = widget_speedometer(sim_world_data)

    % expose public functions
    sim_world_data.funcs.widgets.init_speedometer =...
        @(interface) init_speedometer(interface);
    
    sim_world_data.funcs.widgets.init_speedometer_needle_xy =...
        @(tl,v,mx) init_speedometer_needle_xy(tl,v,mx);
    
    sim_world_data.funcs.widgets.update_speedometer =...
        @(speedometer,ego,t,dt) update_speedometer(speedometer,ego,t,dt);
    
    % *********************** function definitions ***********************
    
    function interface = init_speedometer(interface)
        % ADAS SimWorld: Initialize speedmeter widget
        
        interface.widgets.speedometer.axes = axes(...
            'xtick',[],...
            'ytick',[],...
            'color','none',...
            'xcolor','none',...
            'ycolor','none',...
            'color',interface.colors.panel_background,...
            'position',[0.05 0.015 0.9 0.17],...
            'parent',interface.figures.main.panels.controls_main);
        hold on
        axis image
        box off
        
        % theta arrays
        tl = [0 pi] + [-1 1]*pi/10;                 % theta limits
        interface.widgets.speedometer.tlim = tl;    % store thetalim
        mx = 180;                                   % speed limit
        interface.widgets.speedometer.slim = mx;    % store speed limit
        n = mx/4;                                     % fine to gross ratio
        theta_fine = tl(1):(pi*(1 + 0.2))/n:tl(2);  % fine theta
        theta = theta_fine(1:n/9:end);              % gross theta
        
        % extend meter line
        p_off = pi/25;                                  % angular offset
        p_theta = [tl(1)-p_off theta_fine tl(2)+p_off]; % extended theta ar
        
        % large ticks
        l = 0.1;                                           % tick length
        tick_theta = [theta; theta; nan(size(theta))];     % thick theta
        tick = bsxfun(@plus,ones(size(theta)),[0;-l;nan]); % tick data
        
        % fine ticks
        h = 0.67;                                   % fine tick distance
        tick_theta_fine = ...                       % fine tick theta
            [theta_fine; theta_fine; nan(size(theta_fine))];
        tick_fine = ...                             % fine tick data
            bsxfun(@plus,h*ones(size(theta_fine)),[0;-l;nan]);
        
        % draw speedometer needle
        xy = init_speedometer_needle_xy(tl,0,mx);   % get needle xy
        
        % draw widget components
        patch(cos(p_theta),sin(p_theta),...               % frame
            [1 1 1]*0.3,...
            'edgecolor','w',...
            'linewidth',2,...
            'parent',interface.widgets.speedometer.axes);
        plot(interface.widgets.speedometer.axes,...      % tick
            tick(:).*cos(tick_theta(:)),...
            tick(:).*sin(tick_theta(:)),'w',...
            'linewidth',2)
        plot(interface.widgets.speedometer.axes,...      % fine tick
            tick_fine(:).*cos(tick_theta_fine(:)),...
            tick_fine(:).*sin(tick_theta_fine(:)),'g')
        
        % draw tick numbers
        txt_h = 0.8;                                % text height
        for t = theta                               % iterate over angles
            val = mx*(1-(t-tl(1))/diff(tl));        % tick value
            c_off = 0.07*(length(num2str(val))-1);  % character offset
            text(txt_h*cos(t)-c_off,...
                txt_h*sin(t),...
                num2str(val),...
                'color','w',...
                'fontsize',11,...
                'parent',interface.widgets.speedometer.axes)
        end
        
        % draw needle
        interface.widgets.speedometer.needle = patch(... % needle
            xy(:,1),xy(:,2),'g',...
            'edgecolor','w',...
            'edgealpha','0.5',...
            'facealpha',0.5,...
            'parent',interface.widgets.speedometer.axes);
        
        % draw odometer frame
        d_w = 0.21;     % odometer frame width
        d_h = 0.08;     % odometer frame hieght
        patch([-d_w -d_w d_w d_w],[-d_h d_h d_h -d_h]-0.20,[1 1 1]*0.2)
        
        % draw odometer numbers
        interface.widgets.speedometer.d_total = 0;  % total distance
        d_n = 5;                                    % number of digits
        interface.widgets.speedometer.d_ndigi = d_n; % store n digits
        d_str = init_speedometer_dist_str(d_n,0);
        d_x = -0.205;                               % string position x
        d_y = -0.2;                                 % string position y
        interface.widgets.speedometer.distance = text(...
            d_x,d_y,d_str,...
            'color','w',...
            'fontsize',12,...
            'fontname','monospaced',...
            'parent',interface.widgets.speedometer.axes);
        
        % add unit labels
        text(-0.1,0.25,'km/h','color','w')
        text(0.27,-0.2,'m','color','w')
        
    end
    
    function xy = init_speedometer_needle_xy(tl,v,mx)
        % ADAS SimWorld: Initialize speedometer needle
        
       % speedometer needle
        w = 0.03;                       % needle width
        s = w:-0.005:-w;                % parameterization array
        x = [-w 0 s];                   % needle x
        y = [0 1 -sqrt(w^2 - s.^2)];    % needle y

        % 2D rotation
        rd = @(x,y,t) [x(:),y(:)]*[cosd(t) -sind(t);... rotation matrix
                sind(t)  cosd(t)]';
        
        % calculate and apply rotation
        th = diff(tl)/pi*180*(mx/2-v)/mx;   % needle orientation
        xy = rd(x,y,th);                    % rotate needle xy 
    end

    function d_str = init_speedometer_dist_str(d_n,d)
        % ADAS SimWorld: Obtain speedometer string
        
        d = mod(round(d),10^d_n);                   % round and limit
        d_s = num2str(d);                           % d string
        d_z = num2str(zeros(d_n-length(d_s),1));    % zeros string
        d_str = [d_z' d_s];                         % odometer string
    end

    function speedometer = update_speedometer(speedometer,ego,t,dt)
        % ADAS SimWorld: Update speedometer distance display
        
        % calculate ego speed
        x = ego.x(t,0);             % current x
        x_1 = ego.x(t-dt,0);        % previous x (t_k-1)
        dx = x - x_1;               % x displacement
        dy = ego.y(x) - ego.y(x_1); % y displacement
        dd = sqrt(dx^2 + dy^2);     % displacement
        v = dd/dt/(1000/3600);      % (km/h) speed
        
        % update total travelled distance
        speedometer.d_total = speedometer.d_total + dd;
        
        % draw speedometer needle
        xy = init_speedometer_needle_xy(...
            speedometer.tlim, v, speedometer.slim);
        
        % update needle srawing
        set(speedometer.needle,...
            'xData',xy(:,1),...
            'yData',xy(:,2))
        
        % update distance meter
        d_str = init_speedometer_dist_str(...
            speedometer.d_ndigi, speedometer.d_total);
        
        % update distance drawing
        set(speedometer.distance,...
            'string',d_str)
    end

end
