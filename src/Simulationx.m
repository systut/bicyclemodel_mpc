classdef Simulationx
    % Simulation : Execute robot controller with reference trajectory
    % 
    properties
        model;
        trajectory;
        controller;
        
        x;
        y;
        theta;
    end
    
    methods
        % Constructor
        function simulation = Simulationx()
            simulation.model = BicycleModel();
            simulation.trajectory = Trajectory();
            simulation.controller = ModelPredictController();
            
        end
        
        function simulation = execute_trajectory(simulation)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

