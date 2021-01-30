classdef results_collector
        properties
        C
        rms_error
        end
        
        methods
            
            function [fuel_rate_estimate] = estimate(obj,point)
                arguments
                    obj
                    point (1,2)
                end
                fuel_rate_estimate = obj.C*[point 1]';
            end
            
        end
end