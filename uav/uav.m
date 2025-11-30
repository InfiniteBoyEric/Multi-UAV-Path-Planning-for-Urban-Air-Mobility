classdef uav
   
    properties
        id
        name
        start_point
        end_point

        trajectory_color
    end

    methods
        function obj = uav(id, name, start_point, end_point, trajectory_color)
            obj.id = id;
            obj.name = name;
            obj.start_point = start_point;
            obj.end_point = end_point;
            obj.trajectory_color = trajectory_color;
        end
    end
end