function [obj,stand,mov,onc] = reconstruct_360_space(sensor,ego)
    % Reconstruct 360 EM space around the ego from the sensor measurements
    % @param    sensor An n-sensors-long array containing the standing,
    %           moving, and oncoming object detections in each sensor's FoV
    % @returns stand,mov,onc [x(:) y(:)]shaped arrays containing the
    %          standing, moving and oncoming detections in the
    %          reconstructed space

    % access sensor detections in base workspace
    assignin('base','sensor',sensor);
    assignin('base','ego',ego);

    % get identified object list from previous cycle
    obj_prev = get_obj_prev(sensor);

    % rotation matrix (degrees)
    rd = @(x,y,t) [x(:),y(:)]*[cosd(t) -sind(t);...
        sind(t)  cosd(t)]';

    %       [x,y]
    obj =   [   ];
    stand = [   ];
    mov =   [   ];
    onc =   [   ];

    for ii = 1 : sensor.n
        if sensor.active(ii)
            theta = sensor.theta(ii);

            obj_x = sensor.data(ii).obj(:,1);
            obj_y = sensor.data(ii).obj(:,2);
            obj = cat(1,obj,rd(obj_x,obj_y,theta));

        end
    end

    if ~isempty(obj)

        % compensate ego rotation in previous cycle
        obj_prev = rd(obj_prev(:,1),obj_prev(:,2),ego.Dtheta);

        d_x = bsxfun(@minus,obj(:,1), obj_prev(:,1)');
        d_y = bsxfun(@minus,obj(:,2), obj_prev(:,2)');
        d_mat = sqrt(d_x.^2 + d_y.^2);

        % find the closest object from the previous cycle
        [dmin,imin] = min(abs(d_mat),[],2);

        % remove objects no longer in the sensors FoV
        %imin(imin > length(imin)) = [];

        % obtain object index
        ind = sub2ind(size(d_x), (1:length(imin))', imin);

        % calculate object x y velocity
        obj_v_xy = bsxfun(@plus,[d_x(ind) d_y(ind)]/ego.dt,ego.v_xy);

        % eliminate untracked objects
        d_max_thresh = 2.5;                 % maximum accepted displacement
        obj_v_xy(dmin > d_max_thresh,:) = [];

        % calculate absolute velocity
        obj_v = sqrt(obj_v_xy(:,1).^2 + obj_v_xy(:,2).^2);
        
        % object classification
        %     define velocity thresholds
        v_stand_max_thresh = 0.1; % m/s absolute speed
        v_mov_max_thresh = -5;     % m/s in x direction

        % standing objetcs
        stand_condition = obj_v < v_stand_max_thresh;
        stand = obj(stand_condition,:);

        % moving objects
        mov_condition = logical((obj_v_xy(:,1) > v_mov_max_thresh).*(~stand_condition));
        mov = obj(mov_condition,:);
        
        % oncoming objects
        onc_condition = logical((obj_v_xy(:,1) <= v_mov_max_thresh).*(~stand_condition));
        onc = obj(onc_condition,:);

        assignin('base','obj_prev',obj);
    end
end
