function obj_prev = get_obj_prev(sensor,ego)
    
    % access sensor detections in base workspace
    assignin('base','sensor',sensor);
    assignin('base','ego',ego);
    
    try % previous object list already initialized
        obj_prev = evalin('base','obj_prev');
    catch % initialize object list
        obj_prev = [];
        for ii = 1 : sensor.n
            if sensor.active(ii)
                obj_prev = cat(1,sensor.data{ii});
            end
        end
    end

end