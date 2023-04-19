classdef Measurement
    %MEASUREMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        measurement_param_1
        measurement_param_2
    end
    
    methods
        function measurement = Measurement()
            measurement.measurement_param_1 = [];
            measurement.measurement_param_2 = [];
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

