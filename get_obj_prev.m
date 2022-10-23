function obj_prev = get_obj_prev(sensor)
    
    try % previous object list already initialized
        obj_prev = evalin('base','obj_prev');
    catch % initialize object list
        obj_prev = [];
        for ii = 1 : sensor.n
            if sensor.active(ii)
                obj_prev = cat(1,sensor.data(ii).obj);
            end
        end
    end

end