function [obj,stand,mov,onc] = reconstruct_360_space(sensor,ego)
    % Reconstruct 360 EM space around the ego from the sensor measurements
    % @param    sensor An n-sensors-long array containing the standing,
    %               moving, and oncoming object detections in each sensor's
    %               Field of View
    % @returns  obj [x(:) y(:)] shaped arrays containing the object
    %               detections in the reconstructed space
    %           stand object list classified as standing
    %           mov   object list classified as moving (with of us)
    %           onc   object list classified as oncomming (towards us)  
    %            

    % get identified object list from previous cycle
    obj_prev = get_obj_prev(sensor,ego);    % object list fromprevious cycle

    %       [x,y]   % code outputs
    obj =   [   ];  % list of objects with transformed cooedinates
    stand = [   ];  % list of classified standing objects (do not move)
    mov =   [   ];  % list of classified moving objects   (move away)
    onc =   [   ];  % list of classified oncoming objects (move towards us)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                   Apply coordinate transformation                   %
    %                      The asignment begins here!                     %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % rotation matrix (degrees)
    rd = @(x,y,t) [x(:),y(:)]*[cosd(t) -sind(t);...
        sind(t)  cosd(t)]';
    
    for ii = 1 : sensor.n
        if sensor.active(ii)
            theta = sensor.theta(ii);               % sensr mounting angle
            obj_x = sensor.data{ii}(:,1);       % object position x
            obj_y = sensor.data{ii}(:,2);       % object position y
            obj = cat(1,obj,rd(obj_x,obj_y,theta)); % object rotated
        end
    end

    if ~isempty(obj)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %              Calculate object dynamic properties                %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %threshold definitions
        thresh_d_max = 2.5;       % maximum accepted object displacement
        thresh_v_stand_max = 0.2; % m/s max standing object speed
        thresh_v_mov_min = 7;     % m/s min moving object x velocity
        
        % compensate ego rotation in previous cycle
        obj_prev = rd(obj_prev(:,1),obj_prev(:,2),ego.Dtheta);
        
        % calculate displacement matrix
        d_x = bsxfun(@minus,obj(:,1), obj_prev(:,1)');  % displacement x
        d_y = bsxfun(@minus,obj(:,2), obj_prev(:,2)');  % displacement x
        d_mat = sqrt(d_x.^2 + d_y.^2);                  % displacement abs

        % find the closest object from the previous cycle
        [dmin,imin] = min(abs(d_mat),[],2);                 % min d and ind
        ind = sub2ind(size(d_x), (1:length(imin))', imin);  % matrix ind

        % calculate object x y velocity
        obj_v = bsxfun(@plus,[d_x(ind),d_y(ind)]/ego.dt,ego.v_xy);% vel xy

        % identify tracked objects
        obj_tracked = obj(dmin <= thresh_d_max,:);  % tracked objects list
        obj_v(dmin > thresh_d_max,:) = [];          % tracked objects speed

        % calculate absolute velocity
        obj_speed = sqrt(obj_v(:,1).^2 + obj_v(:,2).^2); % obect speed
        obj_v_sign = obj_speed .* sign(obj_v(:,1)); % x-dir signed speed
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                     Object classification                       %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % standing objetcs
        stand = obj_tracked(obj_speed < thresh_v_stand_max,:);

        % moving objects
        mov = obj_tracked(obj_v_sign > thresh_v_mov_min,:);
        
        % oncoming objects
        onc = obj_tracked(obj_v_sign < -thresh_v_mov_min,:);
        
        % output updated object list to base workspace
        assignin('base','obj_prev',obj);
    end
end
