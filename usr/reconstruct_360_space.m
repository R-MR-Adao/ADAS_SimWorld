function [stand,mov,onc] = reconstruct_360_space(sensor)
    % Reconstruct 360 EM space around the ego from the sensor measurements
    % @param    sensor An n-sensors-long array containing the standing,
    %           moving, and oncoming object detections in each sensor's FoV
    % @returns stand,mov,onc [x(:) y(:)]shaped arrays containing the
    %          standing, moving and oncoming detections in the
    %          reconstructed space
    
    assignin('base','sensor',sensor);
    
    stand = 0;
    mov = 0;
    onc = 0;
end
