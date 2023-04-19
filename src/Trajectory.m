classdef Trajectory
% Dinh Ngoc Duc - TUT
% Create and Read trajectory reference csv file
    
    properties
        ref;
        
        % Returb
        t_ref;
        
        % output_ref
        state_ref;
        x;
        y;
        theta;
        delta;

        % input_ref
        input_ref;
        v;
        delta_dot;
    end
    
    methods
            % Constructor
        function trajectory = Trajectory()
            trajectory.ref = [];
            trajectory.t_ref = [];
            
            trajectory.state_ref=[];
            trajectory.x = [];
            trajectory.y = [];
            trajectory.theta = [];
            trajectory.delta = [];

            trajectory.input_ref=[];
            trajectory.v = [];
            trajectory.delta_dot = [];
        end
        function trajectory = read_ref(trajectory, file_name)

            % Read reference file
            ref_ = csvread(file_name, 2, 0);
            trajectory.ref = ref_';

            % get reference time
            trajectory.t_ref = trajectory.ref(1,:);
            
            % get reference state (output)
            trajectory.x = trajectory.ref(2,:);
            trajectory.y = trajectory.ref(3,:);
            trajectory.theta = trajectory.ref(4,:);
            trajectory.delta = trajectory.ref(5,:);
            trajectory.state_ref=[trajectory.x, trajectory.y, trajectory.theta, trajectory.delta];

            % get reference input
            trajectory.v = trajectory.ref(6,:);
            trajectory.delta_dot = trajectory.ref(7,:);
            % input_ref = 0.1*ones(2,length(ref_v));        
            trajectory.input_ref=[trajectory.v, trajectory.delta_dot];
        end
    end
end