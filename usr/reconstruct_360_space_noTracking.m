function obj = reconstruct_360_space(sensor)
    % Reconstruct 360 EM space around the vehicle by converting object
    % detections from the sensor-specific frame to the vehicle frame
    %
    % @param   sensor: An n-sensors-long array containing object
    %                  detections in each sensor's Field of View
    %
    % @returns obj: [x(:) y(:)]-shaped arrays containing the object
    %               detections in the reconstructed space 
    %

    %       [x,y]   % code outputs
    obj =   [   ];  % list of objects with transformed cooedinates

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

end
