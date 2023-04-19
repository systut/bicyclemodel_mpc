classdef BicycleModel
% Dinh Ngoc Duc - TUT
% Creat a Kinematic model of Bicycle robot
    properties
        % State (output)
        x;
        y;
        theta;
        delta;

        max_steer = 3.14; % vehicle's steering velocity limits [rad/s]
        L = 2;         % vehicle's wheelbase [m]
        sampling_time = 0.05;
    end
    
    methods
        % Constructor
        function obj = BicycleModel()
            % Default initial position [0, 0, 0, 0]
            obj.x = 0;
            obj.y = 0;
            obj.delta = 0;
            obj.theta = 0;
        end

        % Reset robot state 
        function obj = reset(obj)
            obj.x = 0;
            obj.y = 0;
            obj.delta = 0;
            obj.theta = 0;
        end

        % Moving robot
        function obj = step(obj, v_, delta_dot_)
            % constrain on input
            if delta_dot_ > 0
                delta_dot_ = min(delta_dot_, obj.max_steer);
            else
                delta_dot_ = max(delta_dot_, -obj.max_steer);
            end
            
            % implementing the differential equations
            % Input
            x_dot = v_ * cos(obj.theta);
            y_dot = v_ * sin(obj.theta);
            delta_dot = delta_dot_;
            theta_dot = (v_ * tan(obj.delta)) / obj.L;
            
            % Next step predicted state 
            x_new = obj.x + x_dot * obj.sampling_time;
            y_new = obj.y + y_dot * obj.sampling_time;
            delta_new = obj.delta + delta_dot * obj.sampling_time;
            theta_new = obj.theta + theta_dot * obj.sampling_time;
            
            % Returning new value
            obj.x = x_new;
            obj.y = y_new;
            obj.delta = delta_new;
            obj.theta = theta_new;
        end
    end
end