function [obj,stand,mov,onc] = reconstruct_360_space(sensor,ego)
    % Reconstruct 360 EM space around the ego from the sensor measurements
    % @param    sensor An n-sensors-long array containing the standing,
    %           moving, and oncoming object detections in each sensor's FoV
    % @returns stand,mov,onc [x(:) y(:)]shaped arrays containing the
    %          standing, moving and oncoming detections in the
    %          reconstructed space

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
            obj_x = sensor.data(ii).obj(:,1);       % object position x
            obj_y = sensor.data(ii).obj(:,2);       % object position y
            obj = cat(1,obj,rd(obj_x,obj_y,theta)); % object rotated
        end
    end

end
