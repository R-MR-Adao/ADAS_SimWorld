function [stand,mov,onc] = reconstruct_360_space(sensor)
    % Reconstruct 360 EM space around the ego from the sensor measurements
    % @param    sensor An n-sensors-long array containing the standing,
    %           moving, and oncoming object detections in each sensor's FoV
    % @returns stand,mov,onc [x(:) y(:)]shaped arrays containing the
    %          standing, moving and oncoming detections in the
    %          reconstructed space
    
    % access sensor detections in base workspace
    assignin('base','sensor',sensor);

    % rotation matrix (degrees)
    rd = @(x,y,t) [x(:),y(:)]*[cosd(t) -sind(t);... 
                               sind(t)  cosd(t)];
    
    %       [x,y]
    stand = [   ];
    mov =   [   ];
    onc =   [   ];

    for ii = 1 : sensor.n
        if sensor.active(ii)
            theta = sensor.theta(ii);
            
            stand_x = sensor.data(ii).stand(:,1);
            stand_y = sensor.data(ii).stand(:,2);
            stand = cat(1,stand,rd(stand_x,stand_y,theta));
            
            mov_x = sensor.data(ii).mov(:,1);
            mov_y = sensor.data(ii).mov(:,2);
            mov = cat(1,mov,rd(mov_x,mov_y,theta));

            onc_x = sensor.data(ii).onc(:,1);
            onc_y = sensor.data(ii).onc(:,2);
            onc = cat(1,onc,rd(onc_x,onc_y,theta));
        end
    end
end
